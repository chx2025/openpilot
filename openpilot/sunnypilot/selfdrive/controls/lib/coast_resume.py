"""松油门后的滑行回落（coast after gas release）。

【现象】
跟车时嫌距离远，踩一脚油门追上去，松开油门的瞬间被一脚大力刹车"顶"回来。

【机理】
gas 完全不改变 planner 的运算。controlsd 里
    CC.longActive = CC.enabled and (openpilotLongitudinalControl or not pcmCruiseSpeed)
与 gasPressed 无关 —— 踩油门既不 disengage、也不会让 planner 停算，
LongControl 也照常跑 PID 并输出 actuators.accel，只是执行层被驾驶员的油门盖住。
于是：踩油门期间自车加速逼近前车，MPC / e2e 候选的 a_target 一路变负，
这些负值一直"欠着"；松油门后的那几帧里，欠下的负值一次性兑现 —— 就是那脚重刹。

【人类怎么做】
松油门先滑行（自然减速度，平路约 -0.3 m/s²），让相对速度自己把距离消化掉；
只有前车明显减速、或已经进入危险车距，才真的踩刹车。

【做法】
gasPressed 下降沿开启一个滑行窗口（COAST_RESUME_HOLD_S）。窗口内给最终 a_target
加一条**地板**（下限），把"欠着的重刹"抬到自然滑行水平。窗口内每帧重新判定，
下列任一成立就立刻作废本次窗口（单调、无抖动，不会来回跳）：

  ① 有候选要求停车      : planner 的 should_stop / 红灯模块 active / FCW
  ② 前车构成危险        : 前车保持当前速度时「不追尾所需减速度」> 滑行能力
  ③ 前车自己在明显减速  : aLeadK < COAST_RESUME_LEAD_DECEL_MS2
  ④ 其它让位            : 驾驶员在踩刹车 / 车速太低 / 模型给出极激进减速
  ⑤ 没有前车            : COAST_RESUME_REQUIRE_LEAD=True 时不掺和

【安全边界 —— 为什么不会削弱必要的制动】
  - 只抬「负值」，正的 a_target 原样保留 -> 不干扰任何加速意图
  - 只抬到**自然滑行**水平 -> 不是"不刹车"，是"最多滑行"
  - ②③ 保证前车一旦危险/减速就立刻撤销，且撤销后本次窗口不再复活
  - 只作用于 gas 释放后的 COAST_RESUME_HOLD_S 秒
  - 无前车默认不启用（覆盖最窄、风险最低）

【实车核对】
  grep CoastResume /data/log/swaglog.*     # 状态跳变（含"没被抬会是多少"）
  grep LongDecel   /data/log/swaglog.*     # 每条带 coastRes= 字段

【调参】
  松油门后还是有点冲   -> 调小 COAST_RESUME_MAX_DECEL_MS2（地板更平）
  滑行太久 / 贴得太近  -> 调小 COAST_RESUME_HOLD_S 或调大 COAST_RESUME_LEAD_GAP_M
  跟车时该刹不刹       -> 调小 COAST_RESUME_LEAD_ALLOW_MS2
  想让它更早让位       -> 调大 COAST_RESUME_ABORT_A_MS2（例如 -2.5 -> -1.5）
  COAST_RESUME_ENABLE = False 一键回退成原行为
"""
from __future__ import annotations

from dataclasses import dataclass

# ==== 配置常量（模块级，改完重启即生效；本功能刻意不引入配置文件）============
COAST_RESUME_ENABLE: bool = True

# gas 释放后的滑行窗口长度（s）。窗口内才可能施加地板。
COAST_RESUME_HOLD_S: float = 2.0

# 地板的最大强度（m/s²）：地板值再深也不会超过它。
# 它是"滑行"与"刹车"的分界，同时也是前车危险判据的门限（两者必须一致，
# 否则会出现"判据说可以滑行、地板却比滑行更狠"的分裂）。
COAST_RESUME_MAX_DECEL_MS2: float = 1.0

# 前车危险门限：前车保持当前速度时，不追尾所需减速度 <= 它才允许滑行。
COAST_RESUME_LEAD_ALLOW_MS2: float = COAST_RESUME_MAX_DECEL_MS2

# 「危险车距」里那个最小净空（m）：挨到这个距离之前才算危险。
COAST_RESUME_LEAD_GAP_M: float = 6.0

# 前车自身减速度低于它 -> 前车明显在刹 -> 立刻让位（不要跟着"滑行"追尾）。
COAST_RESUME_LEAD_DECEL_MS2: float = -1.5

# 车速太低时不掺和（起步 / 停车阶段交给原有的停车逻辑）。
COAST_RESUME_MIN_V_KPH: float = 12.0
COAST_RESUME_MIN_V_MS: float = COAST_RESUME_MIN_V_KPH / 3.6

# 模型/MPC 给出比这更激进的减速 -> 认为前方确有情况，不抬。
COAST_RESUME_ABORT_A_MS2: float = -2.5

# 是否要求"有前车"才启用。True = 只在跟车场景生效（覆盖最窄、最安全）。
COAST_RESUME_REQUIRE_LEAD: bool = True


def lead_required_decel_m_s2(v_ego: float, v_lead: float, d_rel: float,
                             min_gap_m: float = COAST_RESUME_LEAD_GAP_M) -> float:
  """前车保持当前速度时，「不追尾」所需的最小减速度（>= 0，越大越危险）。

  这是「危险车距」的物理写法，而不是拍一个固定米数：
      required > c   <=>   d_rel < (v_ego - v_lead)^2 / (2c) + min_gap_m
  两者完全等价，但用减速度表达后门限会随车速自动缩放：60 km/h 追 40 km/h 的车
  与 30 km/h 追 10 km/h 的车，危险距离差了 4 倍，固定米数没法同时覆盖。
  """
  closing = max(0.0, float(v_ego) - float(v_lead))
  usable = max(float(d_rel) - float(min_gap_m), 0.5)
  return closing * closing / (2.0 * usable)


def coast_a_floor(accel_coast: float) -> float:
  """本帧的滑行地板值（<= 0）。

  直接用「零踏板自然加速度」accel_coast（平路约 -0.3，上坡更负，下坡为正）——
  那正是"滑行"的物理含义；只把它夹进 [-COAST_RESUME_MAX_DECEL_MS2, 0]：
    下坡 accel_coast 为正         -> 夹到 0（不给油，也不刹车）
    极陡上坡 / orientationNED 无效 -> 夹到 -MAX_DECEL_MS2（不超出滑行范畴）
  """
  return max(-COAST_RESUME_MAX_DECEL_MS2, min(0.0, float(accel_coast)))


@dataclass
class CoastResumeResult:
  a_floor: float | None   # 施加给 output_a_target 的下限（<= 0）；None = 本帧不干预
  active: bool            # 是否处于"滑行回落"生效状态
  reason: str             # 'hold' 生效 / 其余为不生效的原因（见 update 内注释）
  hold_left_s: float      # 本次窗口剩余时长（s）
  lead_required: float    # 诊断：前车所需减速度（m/s²）


class CoastResumeController:
  """每帧由 longitudinal_planner 调 update()，状态在内部维护。

  只记两件事：本次窗口还剩多久、上一帧 gas 是否踩下（用于检测下降沿）。
  """

  def __init__(self) -> None:
    self._hold_left: float = 0.0
    self._gas_last: bool = False

  def reset(self) -> None:
    """外部强制重置（disengage / vCruise 未就绪等）。"""
    self._hold_left = 0.0
    self._gas_last = False

  def _abort(self, reason: str, lead_req: float) -> CoastResumeResult:
    """让位：作废本次窗口（单调，不会下一帧又复活 -> 避免踏板抖振）。"""
    self._hold_left = 0.0
    return CoastResumeResult(None, False, reason, 0.0, lead_req)

  def update(self, *,
             gas_pressed: bool,
             brake_pressed: bool,
             v_ego: float,
             accel_coast: float,
             lead_present: bool,
             d_rel: float,
             v_lead: float,
             a_lead: float,
             should_stop: bool,
             traffic_stop_active: bool,
             fcw: bool,
             a_target_raw: float,
             dt: float) -> CoastResumeResult:
    """每帧调用一次（20 Hz）。

    Args:
      gas_pressed / brake_pressed: carState 的踏板状态
      v_ego: 车速 (m/s)
      accel_coast: 零踏板自然加速度（planner 里由俯仰算得，平路约 -0.3）
      lead_present / d_rel / v_lead / a_lead: radarState.leadOne 的前车量
      should_stop: planner 汇总的"任一候选要求停车"
      traffic_stop_active: 红灯/停止线模块是否在主动停等
      fcw: 前向碰撞告警
      a_target_raw: min(candidates) 之后的原始目标（尚未被地板抬升）
      dt: 帧间隔 (s)

    Returns:
      CoastResumeResult；调用方在 a_floor 非 None 时做 max(a_target, a_floor)
    """
    lead_req = 0.0
    if lead_present:
      lead_req = lead_required_decel_m_s2(v_ego, v_lead, d_rel)

    if not COAST_RESUME_ENABLE:
      self._gas_last = bool(gas_pressed)
      self._hold_left = 0.0
      return CoastResumeResult(None, False, "disabled", 0.0, lead_req)

    # ---- 1. gas 下降沿：开窗 ------------------------------------------------
    if self._gas_last and not gas_pressed:
      self._hold_left = COAST_RESUME_HOLD_S
    self._gas_last = bool(gas_pressed)

    if gas_pressed:
      # 还在给油：窗口不启动（驾驶员显然还在主导）
      self._hold_left = 0.0
      return CoastResumeResult(None, False, "gas", 0.0, lead_req)

    if self._hold_left <= 0.0:
      return CoastResumeResult(None, False, "idle", 0.0, lead_req)

    self._hold_left = max(0.0, self._hold_left - dt)

    # ---- 2. 让位判据（任一成立 -> 作废本次窗口）-----------------------------
    if brake_pressed:
      return self._abort("brake", lead_req)
    if should_stop:
      return self._abort("stop", lead_req)
    if traffic_stop_active:
      return self._abort("traffic", lead_req)
    if fcw:
      return self._abort("fcw", lead_req)
    if v_ego < COAST_RESUME_MIN_V_MS:
      return self._abort("slow", lead_req)
    if float(a_target_raw) < COAST_RESUME_ABORT_A_MS2:
      # 模型/MPC 给出很激进的减速 -> 前方多半确有情况（视觉比雷达看得多），不抬
      return self._abort("hard", lead_req)

    if lead_present:
      if float(a_lead) < COAST_RESUME_LEAD_DECEL_MS2:
        return self._abort("lead-decel", lead_req)
      if lead_req > COAST_RESUME_LEAD_ALLOW_MS2:
        return self._abort("lead-close", lead_req)
    elif COAST_RESUME_REQUIRE_LEAD:
      return self._abort("lead-none", lead_req)

    # ---- 3. 生效：给出地板 ------------------------------------------------
    return CoastResumeResult(coast_a_floor(accel_coast), True, "hold",
                             self._hold_left, lead_req)
