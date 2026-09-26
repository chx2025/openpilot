"""stop_soften.py — 「静止起步蠕动」场景下的减速柔化（2026-09-26 二版）

需求原文（用户 2026-09-26）
──────────────────────────────────────────────────────────────────────────
  一版：「我认为还要再柔化，前提是前车静止，本车静止」
  二版（本版）：
    「我要做的柔性刹车，有个前提就是，本车静止，前车静止（这二个条件满足
      才生效，其他情况一率跳过），本车加油启动柔性蠕动一点距离」
    「保证最小车距为 2，根据油门力度和油门持续时间定蠕动距离」

与一版的区别（为什么必须改）
──────────────────────────────────────────────────────────────────────────
  * 一版的触发条件是「本车**低速**（≤ 3 m/s）+ 前车静止」⇒ 行进中减速接近
    静止前车的**一整段**都在柔化窗口里。实车（09-26 15:39）表现为：柔化一路
    把上游 −0.8 ~ −1.0 抬到 −0.5（该刹的时候没刹够），等 d 压进 D_HARD 又
    整段撤销、上游全量透传 ⇒ 输出阶跃、最后一下"咬住" ⇒ 用户反馈
    "拐过去后面的刹车有点不连贯"。
  * 二版把触发条件换成**场景闸门**（crawl_gate.py）：只有「本车静止 +
    前车静止」才开门，其余工况一律跳过 ⇒ 上面的阶跃场景从根上不存在。
  * "停位目标"从 D_HARD / D_MARGIN = 3.0 / 3.0 收成单一参数
    STOP_SOFTEN_MIN_GAP_M = 2.0（用户："保证最小车距为 2"）。一版两个 3.0
    相等本身就是缺陷：柔化窗口下沿与物理目标重合 ⇒ d 刚过 3.0 时
    usable = d − 3.0 → 0 ⇒ 所需减速度爆掉、地板瞬间失效。现在只有一个下沿，
    且 d 越接近它 a_required 越负（**连续**收紧），从"柔化"到"原厂全量"的
    过渡是平滑的，不再有跳变。

适用工况（闸门打开 + 下面三条同时成立才干预）
──────────────────────────────────────────────────────────────────────────
  * crawl_gate.latched 为真（本车静止 + 前车静止，见 crawl_gate.py）
  * 前车仍然静止：v_lead <= STOP_SOFTEN_V_LEAD_STATIC_MS（1.0 m/s）
  * 距离 > STOP_SOFTEN_MIN_GAP_M（2.0 m）

动作（与 gas_override 同构，只抬下限）
──────────────────────────────────────────────────────────────────────────
    output_a_target = max(output_a_target, floor)

  floor = min(A_SOFT, a_required)      ← 取更负者（更保守的一方）
  a_required = -(v_ego²) / (2 * (d_rel - MIN_GAP))

  * A_SOFT = −0.5 m/s²：期望的柔化减速（≈0.05 g，体感"轻轻带住"）
  * a_required：**物理下界** —— 「刚好能在距前车 2 m 处停住」所需的减速度。
    距离越紧、车速越高，它越负，于是地板自动收紧。
  * ★ 蠕动距离本身**不由本模块决定** —— 用户明确要"根据油门力度和油门
    持续时间定蠕动距离"，即走多远由驾驶员说了算；本模块只负责
    「不猛冲、不点头、不硬按死」以及 2 m 的物理底线。

★ **安全论证（本设计的核心）**：
  柔化后的减速度**永远不低于**"恰好能在前车前 2 m 停住"所需的量。
  即：柔化只会让减速曲线更"前松后紧"，**不会导致停不住**。
  极端情况 a_required 比 ACCEL_MIN 还负时，取 ACCEL_MIN（= 原逻辑硬刹上限）。

不干预的情形（任何一条成立即原样透传，a_target 逐位不变）
──────────────────────────────────────────────────────────────────────────
  * 闸门关闭（不是"本车静止 + 前车静止"场景）  —— 本版最关键的收窄
  * FCW 置位 / forceDecel                     —— 真安全信号，不参与柔化
  * 前车在动（v_lead > 阈值）                 —— 常规跟车，交给 MPC 原逻辑
  * 本车不低速（v_ego > 3 m/s）               —— 中高速，交给 MPC 原逻辑
  * 距离 <= 2 m（最小车距）                  —— 更近则完全透传上游（回到原减速曲线）
  * 无前车（lead.present 为假）

日志
──────────────────────────────────────────────────────────────────────────
  * 活动期间按 STOP_SOFTEN_LOG_HZ（1 Hz）节流；
  * **进入与退出当帧必定落盘**（一版退出是不打日志的，导致"柔化何时撤"在
    swaglog 里是盲区 —— 15:39 那次只能靠推断）。

开关 / 回退
──────────────────────────────────────────────────────────────────────────
  * 代码级：STOP_SOFTEN_ENABLED = False  → 逐位等价于没装本模块（一行回退）
  * 场景级：CRAWL_GATE_ENABLED = False   → 本模块恒不干预（与 gas_override 一起退）
  * 调参：STOP_SOFTEN_A_SOFT 改 0.0      → 柔化力度归零（等于关闭）
  * 该模块**只抬下限**，所以 `GasPedalOverride=0` 等其他开关的语义不受影响。

放在纵向链路的位置
──────────────────────────────────────────────────────────────────────────
  candidates.min() → turn_decel → gas_override → **stop_soften** → np.clip
  与 gas_override 并列（两者都是"抬下限"，互不覆盖，先到先得取更宽松的一个）。
  ⚠️ 必须与 longcontrol 的「踩油门豁免 cruise_standstill」配套：
     stopping 分支**完全忽略 a_target**，只做柔化不改状态机是看不到效果的。
"""

from __future__ import annotations

from opendbc.car.interfaces import ACCEL_MIN

# ── 开关 ──────────────────────────────────────────────────────────────────
STOP_SOFTEN_ENABLED = True

# ── 触发条件 ──────────────────────────────────────────────────────────────
STOP_SOFTEN_V_EGO_MAX_MS = 3.0        # 本车蠕动速度上限 (≈10.8 km/h)，与闸门退出阈值一致
STOP_SOFTEN_V_LEAD_STATIC_MS = 1.0    # 前车视为静止 (≈3.6 km/h)，与闸门进入阈值一致
STOP_SOFTEN_MIN_GAP_M = 2.0           # 最小车距：柔化的物理目标点，也是关闭边界

# ── 柔化参数 ──────────────────────────────────────────────────────────────
STOP_SOFTEN_A_SOFT = -0.5             # 期望的柔化减速 (m/s²)

# 日志节流（与 gas_override 同款：跳变必落盘，其余按低频）
STOP_SOFTEN_LOG_HZ = 1.0


class StopSoftenResult:
  def __init__(self, a_target_out: float, active: bool, reason: str, log=None):
    self.a_target_out = a_target_out
    self.active = active
    self.reason = reason
    self.log = log


class StopSoftenController:
  """「静止起步蠕动」场景下的减速柔化（只抬下限）。"""

  def __init__(self):
    self._last_reason = ''
    self._log_accum = 0.0
    self._was_active = False

  def reset(self):
    self._last_reason = ''
    self._log_accum = 0.0
    self._was_active = False

  # -- 内部：算地板 --------------------------------------------------------
  @staticmethod
  def _floor(v_ego: float, d_rel: float) -> float:
    """返回柔化地板（负数）。含物理下界，保证停得住（距前车 MIN_GAP 之前）。"""
    usable = max(d_rel - STOP_SOFTEN_MIN_GAP_M, 0.1)
    a_required = -(v_ego * v_ego) / (2.0 * usable)
    floor = min(STOP_SOFTEN_A_SOFT, a_required)   # 更负者胜 = 更保守
    return max(floor, ACCEL_MIN)

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float,
             fcw: bool, force_decel: bool, a_target_in: float,
             dt: float, crawl_gate: bool = False) -> StopSoftenResult:
    if not STOP_SOFTEN_ENABLED:
      return self._ret(a_target_in, False, 'disabled', dt)

    # ★ 场景闸门：不是「本车静止 + 前车静止」⇒ 一帧都不干预
    if not crawl_gate:
      return self._ret(a_target_in, False, 'no_gate', dt)

    # 真安全信号：一帧都不干预
    if fcw or force_decel:
      return self._ret(a_target_in, False, 'guard', dt)
    if not lead_present:
      return self._ret(a_target_in, False, 'no_lead', dt)
    if v_lead > STOP_SOFTEN_V_LEAD_STATIC_MS:
      return self._ret(a_target_in, False, 'lead_moving', dt)
    if v_ego > STOP_SOFTEN_V_EGO_MAX_MS:
      return self._ret(a_target_in, False, 'v_high', dt)
    # 最小车距：贴到这个距离就不再柔化，完全交还上游（保证 2 m 底线）
    if d_rel <= STOP_SOFTEN_MIN_GAP_M:
      return self._ret(a_target_in, False, f'min_gap{d_rel:.1f}', dt)

    floor = self._floor(v_ego, d_rel)
    if a_target_in < floor:
      return self._ret(floor, True,
                       f'soften({a_target_in:.2f}->{floor:.2f},'
                       f'd{d_rel:.1f},v{v_ego:.2f})', dt)
    return self._ret(a_target_in, False, 'above', dt)

  def _ret(self, a_out: float, active: bool, reason: str, dt: float) -> StopSoftenResult:
    log = None
    if active:
      # 活动期间按 STOP_SOFTEN_LOG_HZ 节流落盘；**进入当帧必打**
      self._log_accum += dt
      if not self._was_active or self._log_accum >= 1.0 / STOP_SOFTEN_LOG_HZ:
        self._log_accum = 0.0
        log = f'[StopSoften] {reason}'
    else:
      # **退出当帧必打** —— 一版这里是盲区（退出无日志），
      # 导致"柔化是何时、因何撤掉的"只能靠推断。
      if self._was_active:
        log = f'[StopSoften] release({reason})'
      self._log_accum = 0.0
    self._was_active = active
    self._last_reason = reason
    return StopSoftenResult(a_out, active, reason, log)
