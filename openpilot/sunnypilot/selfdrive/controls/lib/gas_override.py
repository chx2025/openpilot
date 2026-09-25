"""gas_override.py — 驾驶员踩油门时的纵向让位控制器（2026-09-25）

需求原文（用户 2026-09-25）
──────────────────────────────────────────────────────────────────────────
 1. 一旦踩油门，所有模型或者系统过来的减速动作需暂停或者减小减速动作使本车缓慢
    靠近前车（除非跟目标前车车距小于 4 米且比前车车速快 10 公里每小时以上，
    或者跟目标前车车距小于 2 米）
 2. 一旦踩完油门后松油门，如果当前车速比设定速度快，就以滑行的方式减速到设定
    速度，然后由原有逻辑接管
 3. 一旦踩完油门后松油门，如果当前车速比设定速度慢，0.5 秒后就由原系统正常接管，
    不要有空档期

实现方式（一句话）
──────────────────────────────────────────────────────────────────────────
不改 candidates 池、不改 MPC、不改 LongControl。只在 longitudinal_planner
算出 `output_a_target` 之后**抬升它的下限**：

    output_a_target = max(output_a_target, a_floor)

即「**只能少刹车，不能多给油**」。这条不变式带来三个直接好处：

  * 加速侧完全不动 —— 模型/MPC/cruise 想加速仍然加速。这是需求 3
    「不要有空档期」的必要条件：本模块永远不会成为"谁都不管"的那一段。
  * 永远产生不出比原逻辑更大的加速度 —— 不可能因为踩油门让车自己冲出去。
  * 安全例外一触发就 `a_floor = None`（一帧都不干预），原逻辑（含硬刹）
    100% 原样保留。

唯一的例外是**滑行相位**（需求 2）额外加了一条**加速度封顶**（默认 0.0），
因为"滑行"的物理语义就是"不加油"；理由见 GAS_RELEASE_COAST_A_CEIL 的注释。

四个相位
──────────────────────────────────────────────────────────────────────────
  inactive  不干预（a_floor = None）——绝大多数时间
  pressed   gas=1 且无安全例外 → floor = GAS_PRESSED_A_FLOOR（默认 −0.3 m/s²）
  coast     松油门且 vEgo > vCruise → clamp 到 [floor, ceil] =
            [滑行减速度, 0.0]，滑回到 vCruise 附近后交还原逻辑
  hold      松油门且 vEgo <= vCruise → floor = GAS_PRESSED_A_FLOOR，维持
            GAS_RELEASE_RESUME_DELAY_S（默认 0.5 s）后彻底交还原逻辑

开关（默认开启）
──────────────────────────────────────────────────────────────────────────
  * 运行期开关 = 参数 `GasPedalOverride`（GAS_OVERRIDE_PARAM_KEY）。
    在 `common/params_keys.h` 声明，默认值 "1" = 开；对应
    设置 → Toggles → "Gas Pedal Override"。plannerd 以 1 Hz 轮询，
    拨开关后约 1 s 内生效，**不需要重启**。
    ⚠️ `Params.get_bool()` 对不存在的键返回 False（C++ `Params::get` 不走
    `default_value` 回退），所以本模块用 `get(key, return_default=True)` 才能
    拿到声明里的 "1"，这是"默认开启"能成立的关键。
  * 代码级开关 = GAS_OVERRIDE_ENABLE（发版兜底）。
  * 两者取 AND：任一为关 → 相位恒为 inactive，a_target 与输入**逐位相同**，
    等价于没装这个模块。

安全例外（**任何相位下都优先**，成立时 floor = None、ceil = None）
──────────────────────────────────────────────────────────────────────────
  * 目标前车（radarState.leadOne）车距 < GAS_SAFE_D_REL_MIN_M（2 m）        …… 需求 1.b
  * 前车车距 < GAS_SAFE_D_REL_CLOSE_M（4 m）且本车比前车快 > 10 km/h        …… 需求 1.a
  * FCW 置位（GAS_OVERRIDE_FCW_GUARD）—— 碰撞即将发生，不参与让位
  * forceDecel（驾驶员监控强制减速，GAS_OVERRIDE_FORCE_DECEL_GUARD）
  * vCruise 未就绪 / <= GAS_OVERRIDE_MIN_V_CRUISE_MS（视为"没设巡航"）

设计取舍（为什么这样做）
──────────────────────────────────────────────────────────────────────────
  * **为什么用 max 抬地板，而不是替换 candidates？**
    替换会连带废掉 FCW / 前车 / MPC 的制动，属于安全回退；抬地板只削掉
    "比 GAS_PRESSED_A_FLOOR 更狠的那部分减速"，且例外条件一满足就归零。
  * **为什么踩油门时 floor 不是 0（完全暂停）而是 −0.3？**
    需求原文是"暂停**或者减小**"。取 −0.3 m/s²（0.03 g）：
      - 驾驶员几乎感觉不到（油门侧扭矩远大于它）
      - 但保留了"松油门瞬间不会毫无过渡"的最小收敛能力
      - 0.3 这个值同时是平路滑行的实测拟合值（见 get_coast_accel）
    要完全暂停减速 → 把 GAS_PRESSED_A_FLOOR 改成 0.0（一行）。
  * **为什么 coast 用 min(−0.3, accel_coast)？**
    accel_coast = get_coast_accel(pitch) 是"松油门后真实滑行"的加速度：
      - 平路  ≈ −0.3 → 与固定值一致
      - 上坡  更负（如 −0.8）→ 允许按真实滑行减速，收敛更快、不虚刹
      - 下坡  为正（如 +0.2）→ 取 −0.3 兜底，保证一定能收敛回 vCruise
    注意 floor 恒 <= −0.3 < 0，所以滑行阶段**一定**会收敛到设定速度。
  * **为什么需求 3 的 hold 窗口里 floor 还压着 −0.3？**
    需求 3 的 0.5 s 是"让位窗口的最短时长"，不是"什么都不做的空档"：
    窗口内**只禁减速、完全放行加速**，所以不影响加速响应；0.5 s 到点即彻底
    交还（对比 turn_decel 的松油门宽限是 2 s，这里刻意更短）。
    把这个窗口设成 0.0 就是"松油门当帧交还"。
  * **`suppress_should_stop` 是干什么的？**
    `should_stop(v_ego, a_target) = v_ego < 0.3 and a_target < 0.1`，只在
    接近静止时成立。LongControl 收到它会把状态机切到 `stopping`
    （`longcontrol.py::long_control_state_trans`），输出退化成衰减的
    last_output_accel —— 即"OP 不再跟随 a_target"。低速蠕行时踩油门再松开，
    这正是需求 3 说的「空档期」。所以让位期间把它一并清掉，状态机留在 pid。

公开 API
──────────────────────────────────────────────────────────────────────────
  GasOverrideController.update(...) -> GasOverrideResult
  GasOverrideResult.a_floor              : float | None  （None = 不干预）
  GasOverrideResult.a_target_out         : float         （已应用地板后的结果）
  GasOverrideResult.phase                : str
  GasOverrideResult.suppress_should_stop : bool
  GasOverrideResult.log                  : str | None    （需要落盘的日志行）
  GasOverrideController.reset()          : None

本模块**零外部依赖**（只用标准库 time/dataclasses），便于在无 cereal/capnp
的环境里直接跑单测。
"""
from __future__ import annotations

import time
from dataclasses import dataclass


# ==== 总开关 ================================================================
# 一键回退：False 时 update() 恒返回 inactive，与没装这个模块完全等价。
# 这是**代码级**总开关（发版兜底）；运行期开关是参数 GAS_OVERRIDE_PARAM_KEY，
# 两者取 AND —— 任一为关即不干预。
GAS_OVERRIDE_ENABLE: bool = True
# 运行期开关的参数名。在 common/params_keys.h 里声明，默认值 "1"（默认开启），
# 对应 设置 → Toggles 里的 "Gas Pedal Override"。
GAS_OVERRIDE_PARAM_KEY: str = "GasPedalOverride"
# 参数轮询周期（s）。1 Hz 足够让 UI 开关在下一帧内生效，又不至于每帧读文件。
GAS_OVERRIDE_PARAMS_PERIOD_S: float = 1.0

# ==== 需求 1：踩油门时抬升减速度下限 =========================================
# 踩油门时允许的**最大减速度**（m/s²，负值）。
#   -0.3 = 「减小」减速动作（默认，保留最小收敛能力）
#    0.0 = 「暂停」一切减速动作（完全交给驾驶员油门）
GAS_PRESSED_A_FLOOR: float = -0.3
# 安全例外 a（需求 1.a）：前车车距 < 4 m **且** 本车比前车快 > 10 km/h
GAS_SAFE_D_REL_CLOSE_M: float = 4.0
GAS_SAFE_CLOSING_KPH: float = 10.0
# 安全例外 b（需求 1.b）：前车车距 < 2 m（无条件）
GAS_SAFE_D_REL_MIN_M: float = 2.0
# 安全例外 c：FCW 置位时不让位（碰撞即将发生，属于安全系统而非"舒适减速"）
GAS_OVERRIDE_FCW_GUARD: bool = True
# 安全例外 d：forceDecel（驾驶员监控强制减速）时不让位
GAS_OVERRIDE_FORCE_DECEL_GUARD: bool = True
# vCruise <= 该值（m/s）视为"没有设定巡航速度"，不干预（3.6 km/h）
GAS_OVERRIDE_MIN_V_CRUISE_MS: float = 1.0

# ==== 需求 2：松油门且超速 -> 滑行回落 ======================================
GAS_RELEASE_COAST_ENABLE: bool = True
# 滑行阶段允许的最大减速度上限（m/s²，负值）。floor = min(该值, accel_coast)
GAS_RELEASE_COAST_MAX_DECEL: float = -0.3
# 滑行阶段允许的最大**加速度**上限（m/s²）。0.0 = 滑行期间不加油（标准滑行语义）；
# 设为 None = 不封顶（退回"只抬地板、绝不削加速"的纯净语义）。
# 为什么需要这条：`get_cruise_accel()` 对 a_cruise 做了 jerk 限幅，车速刚越过
# 设定速度的头 1~2 s 内 a_cruise 仍可能为正；若不封顶会出现"松油门后反而继续
# 加速"，滑行相位永远不收敛，只能等 GAS_RELEASE_COAST_MAX_S 兜底。
GAS_RELEASE_COAST_A_CEIL: float | None = 0.0
# 滑行退出裕量（m/s）：v_ego 回落到 v_cruise + 该值以内即交还原逻辑
# （提前一点点交还，避免在设定速度上反复穿越导致抖动）
GAS_RELEASE_COAST_EXIT_MARGIN_MS: float = 0.3
# 滑行阶段最长持续时间（s）：兜底，避免在长下坡上无限期压着地板
GAS_RELEASE_COAST_MAX_S: float = 60.0

# ==== 需求 3：松油门且未超速 -> GAS_RELEASE_RESUME_DELAY_S 后交还 ===========
GAS_RELEASE_RESUME_DELAY_S: float = 0.5

# ==== 低速"停车意图"清理（落地需求 3「不要有空档期」，见文件头说明）========
GAS_OVERRIDE_CLEAR_STOP_INTENT: bool = True

# ==== 日志 ==================================================================
# swaglog 最低只落盘 INFO(20)，所以一律用 cloudlog.info；节流见下。
# 状态跳变**必定**落盘，其余活动期按 GAS_OVERRIDE_LOG_HZ 节流。
GAS_OVERRIDE_LOG_HZ: float = 1.0
# 这些 reason 属于"常态不干预"，只在相位跳变时报一行，不按 LOG_HZ 反复刷
_QUIET_REASONS: frozenset[str] = frozenset({"disabled", "no_v_cruise"})


@dataclass
class GasOverrideResult:
  a_floor: float | None          # 要抬升到的下限；None = 不干预
  a_target_out: float            # 应用地板后的 a_target（a_floor is None 时 = 输入）
  phase: str                     # off / inactive / pressed / coast / hold
  active: bool                   # 是否正在让位（a_floor is not None）
  reason: str                    # 安全例外原因 / 关闭原因，'' 表示无
  suppress_should_stop: bool     # True = 建议清掉 should_stop 意图
  log: str | None                # 需要落盘时给出整行（含 [GasOverride] 标签）
  enabled: bool = True           # 运行期开关是否打开（代码开关 AND 参数开关）


class GasOverrideController:
  """每帧由 longitudinal_planner 调一次 update()。

  内部只维护四样东西：上次 gas 状态、hold 剩余时长、coast 已持续时长、
  以及从 Params 轮询到的运行期开关。没有 ramp / 没有滞回闩锁：地板是
  **纯静态映射**，相位只由 (gas, v_ego, v_cruise) 决定，行为可预测、可复现。

  `params` 用依赖注入而不是本模块 import，好处：
    - 模块保持零依赖，单测可以在没有 cereal/capnp 的机器上直接跑
    - 单测可以塞一个假的 params 对象来验证「UI 开关关掉 -> 完全不干预」
  """

  def __init__(self, params=None) -> None:
    self._params = params              # None = 纯离线/测试模式
    self._phase: str = "inactive"
    self._gas_last: bool = False       # 上一帧的 gasPressed（用于识别下降沿）
    self._hold_left: float = 0.0       # 松油门后 0.5 s 窗口剩余时长
    self._coast_elapsed: float = 0.0   # coast 已持续时长
    self._log_next_t: float = 0.0      # 下一次允许打印的时刻（节流）
    self._logged_phase: str = ""       # 上一次已落盘的相位
    self._param_t: float = GAS_OVERRIDE_PARAMS_PERIOD_S  # 让首帧就同步一次参数
    self._param_enabled: bool = True   # 从 Params 读到的运行期开关
    self._param_note: str | None = None  # 开关跳变时待落盘的一行（见 _poll_param）

  @property
  def enabled(self) -> bool:
    """代码级开关 AND 运行期（UI）开关。"""
    return bool(GAS_OVERRIDE_ENABLE and self._param_enabled)

  def reset(self) -> None:
    """外部强制重置（disengage / 未接管 / 车速未就绪）。"""
    self._phase = "inactive"
    self._gas_last = False
    self._hold_left = 0.0
    self._coast_elapsed = 0.0

  def _poll_param(self, dt: float) -> None:
    """按 GAS_OVERRIDE_PARAMS_PERIOD_S 轮询运行期开关。

    注意 `Params.get_bool()` 对**不存在的键返回 False**（C++ Params::get 不走
    default_value 回退），所以这里必须用 `return_default=True` 才能拿到
    params_keys.h 里声明的默认值 "1"（= 默认开启）。
    """
    self._param_t += dt
    if self._params is None or self._param_t < GAS_OVERRIDE_PARAMS_PERIOD_S:
      return
    self._param_t = 0.0
    try:
      value = self._params.get(GAS_OVERRIDE_PARAM_KEY, return_default=True)
    except Exception:  # 参数未声明 / 读取失败：保持当前值，绝不影响纵向控制
      return
    if value is not None:
      new_enabled = bool(value)
      if new_enabled != self._param_enabled:
        # 不开 cloudlog（本模块零依赖），把这条挂到下一次输出上，由调用方落盘
        self._param_note = (f"[GasOverride] param {GAS_OVERRIDE_PARAM_KEY} "
                            f"-> {'on' if new_enabled else 'off'}")
      self._param_enabled = new_enabled

  # ------------------------------------------------------------------------
  def update(
    self,
    *,
    gas_pressed: bool,
    v_ego: float,
    v_cruise: float,
    accel_coast: float,
    a_target_in: float,
    dt: float,
    lead_present: bool = False,
    d_rel: float = 0.0,
    v_lead: float = 0.0,
    fcw: bool = False,
    force_decel: bool = False,
  ) -> GasOverrideResult:
    """每帧调用一次（plannerd 20 Hz，dt = DT_MDL = 0.05）。

    Args:
      gas_pressed: carState.gasPressed
      v_ego: 当前车速（m/s）
      v_cruise: **原逻辑实际执行**的目标速度（m/s）。取
        LongitudinalPlannerSP.update_targets() 的返回值，而不是 UI 上的
        carState.vCruise —— 这样"滑行到设定速度"停下时，原逻辑刚好也是
        在那里停止制动，交接无阶跃。
      accel_coast: 当前坡度下的滑行加速度（get_coast_accel(pitch)）
      a_target_in: 本帧 candidates min()（+ turn_decel）算出的 a_target
      dt: 帧间隔（s）
      lead_present / d_rel / v_lead: radarState.leadOne 的 present / dRel / vLead
      fcw: 本车 FCW 是否置位
      force_decel: controlsState.forceDecel

    Returns:
      GasOverrideResult（a_target_out 恒为已应用地板/封顶后的值）
    """
    # 本帧的上下文，避免在每个 return 上重复 8 个参数
    ctx = (gas_pressed, v_ego, v_cruise, accel_coast, a_target_in, d_rel, v_lead, fcw)

    # ---- 0a. 运行期（UI）开关：1 Hz 轮询，关闭时一帧都不干预 ----
    self._poll_param(dt)
    if not self.enabled:
      return self._to_inactive(ctx, "disabled")

    # ---- 0b. 安全例外（需求 1 的两个例外 + FCW/forceDecel 守卫）----
    # 放在 v_cruise 守卫之前：forceDecel 会把 v_cruise 置 0，先查安全例外
    # 才能把 reason 记成 "forceDecel" 而不是含糊的 "no_v_cruise"。
    safe_reason = self._check_safety(v_ego, lead_present, d_rel, v_lead, fcw, force_decel)
    if safe_reason:
      return self._to_inactive(ctx, safe_reason)

    # ---- 0c. 没有可用的设定速度（未设巡航 / forceDecel 已把 v_cruise 清零）----
    if v_cruise <= GAS_OVERRIDE_MIN_V_CRUISE_MS:
      return self._to_inactive(ctx, "no_v_cruise")

    # ---- 2. 踩油门：需求 1 ----
    if gas_pressed:
      self._gas_last = True
      # 武装松油门后的短窗口（只在松油门的当帧被消费）
      self._hold_left = GAS_RELEASE_RESUME_DELAY_S
      self._coast_elapsed = 0.0
      self._phase = "pressed"
      return self._emit(ctx, GAS_PRESSED_A_FLOOR, None, "pressed", "", True)

    # ---- 3. 松油门：区分需求 2（超速）与需求 3（未超速）----
    if self._gas_last:
      self._gas_last = False
      if GAS_RELEASE_COAST_ENABLE and v_ego > v_cruise:
        self._phase = "coast"
        self._coast_elapsed = 0.0
      else:
        self._phase = "hold"

    if self._phase == "coast":
      self._coast_elapsed += dt
      if v_ego <= v_cruise + GAS_RELEASE_COAST_EXIT_MARGIN_MS or self._coast_elapsed >= GAS_RELEASE_COAST_MAX_S:
        return self._to_inactive(ctx, "")
      # 滑行：允许的减速度取「真实滑行」与「固定上限」里更狠的那个，
      # 且恒 <= GAS_RELEASE_COAST_MAX_DECEL < 0 → 一定收敛回 v_cruise。
      coast_floor = min(GAS_RELEASE_COAST_MAX_DECEL, accel_coast)
      return self._emit(ctx, coast_floor, GAS_RELEASE_COAST_A_CEIL, "coast", "", True)

    if self._phase == "hold":
      self._hold_left -= dt
      if self._hold_left <= 0.0:
        return self._to_inactive(ctx, "")
      return self._emit(ctx, GAS_PRESSED_A_FLOOR, None, "hold", "", True)

    # ---- 4. 常态：不干预 ----
    return self._to_inactive(ctx, "")

  # ------------------------------------------------------------------------
  @staticmethod
  def _check_safety(v_ego: float, lead_present: bool, d_rel: float,
                    v_lead: float, fcw: bool, force_decel: bool) -> str:
    """返回非空字符串 = 安全例外成立（不让位），字符串是原因（用于日志）。"""
    if GAS_OVERRIDE_FORCE_DECEL_GUARD and force_decel:
      return "forceDecel"
    if GAS_OVERRIDE_FCW_GUARD and fcw:
      return "fcw"
    if lead_present:
      if d_rel < GAS_SAFE_D_REL_MIN_M:
        return f"lead<{GAS_SAFE_D_REL_MIN_M:g}m({d_rel:.2f})"
      closing_kph = (v_ego - v_lead) * 3.6
      if d_rel < GAS_SAFE_D_REL_CLOSE_M and closing_kph > GAS_SAFE_CLOSING_KPH:
        return f"lead<{GAS_SAFE_D_REL_CLOSE_M:g}m+fast({d_rel:.2f}m,{closing_kph:.1f}kph)"
    return ""

  def _to_inactive(self, ctx: tuple, reason: str) -> GasOverrideResult:
    """进入/保持 inactive：清掉所有相位状态，本帧不干预。

    `reason` 非空表示"本来该干预但被挡下"（安全例外 / 开关关闭 / 没设巡航），
    只有日志用途 —— 输出与"没有本模块"逐位相同。
    """
    (gas_pressed, _v_ego, _v_cruise, _accel_coast, _a_target_in,
     _d_rel, _v_lead, _fcw) = ctx
    self._phase = "inactive"
    self._gas_last = bool(gas_pressed)
    self._hold_left = 0.0    # 安全例外/退化状态下不武装松油门窗口
    self._coast_elapsed = 0.0
    return self._emit(ctx, None, None, "inactive", reason, False)

  # ------------------------------------------------------------------------
  def _emit(self, ctx: tuple, a_floor: float | None, a_ceil: float | None,
            phase: str, reason: str, active: bool) -> GasOverrideResult:
    """组装结果并做日志节流。

    地板只抬不降：`max(a_target_in, a_floor)`；封顶只削不加：`min(..., a_ceil)`。
    常态（a_floor/a_ceil 均为 None）时 a_target_out 与输入**逐位相同**。
    """
    (gas_pressed, v_ego, v_cruise, _accel_coast, a_target_in,
     d_rel, v_lead, fcw) = ctx

    a_target_out = a_target_in
    if a_floor is not None:
      a_target_out = max(a_target_out, a_floor)
    if a_ceil is not None:
      a_target_out = min(a_target_out, a_ceil)

    suppress_should_stop = bool(GAS_OVERRIDE_CLEAR_STOP_INTENT and active)

    # 日志：相位跳变必落；活动期（或安全例外期）按 LOG_HZ 节流。
    # 常态 inactive 且无原因时不打印，避免 20 Hz 刷爆 swaglog；
    # "安静"原因（关掉功能 / 没设巡航）只在跳变时报一行，不按秒刷。
    now = time.monotonic()
    interesting = active or (reason != "" and reason not in _QUIET_REASONS)
    due = now >= self._log_next_t
    worth_logging = (interesting or reason != "" or self._logged_phase not in ("inactive", ""))
    log_line: str | None = self._param_note
    self._param_note = None
    if worth_logging and (phase != self._logged_phase or (interesting and due)):
      body = (
        f"[GasOverride] {phase} active={int(active)} enabled={int(self.enabled)} "
        f"gas={int(gas_pressed)} vEgo={v_ego * 3.6:.1f} vCruise={v_cruise * 3.6:.1f} "
        f"aIn={a_target_in:+.2f} floor={'None' if a_floor is None else f'{a_floor:+.2f}'} "
        f"ceil={'None' if a_ceil is None else f'{a_ceil:+.2f}'} "
        f"aOut={a_target_out:+.2f} dRel={d_rel:.1f} vLead={v_lead * 3.6:.1f} "
        f"fcw={int(fcw)} reason={reason or '-'}"
      )
      log_line = f"{log_line}\n{body}" if log_line else body
      self._log_next_t = now + 1.0 / max(GAS_OVERRIDE_LOG_HZ, 1e-3)
    self._logged_phase = phase

    return GasOverrideResult(
      a_floor=a_floor,
      a_target_out=a_target_out,
      phase=phase,
      active=active,
      reason=reason,
      suppress_should_stop=suppress_should_stop,
      log=log_line,
      enabled=self.enabled,
    )
