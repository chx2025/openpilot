"""stop_soften.py — 「停车/低速 + 前方静止目标」的减速柔化（2026-09-26）

需求原文（用户 2026-09-26）
──────────────────────────────────────────────────────────────────────────
  「我认为还要再柔化，前提是前车静止，本车静止」
  （承接上一轮：停车起步时踩油门"只动一下"，松油门"直接按死"，点头严重）

适用工况（三个条件**同时**成立才干预）
──────────────────────────────────────────────────────────────────────────
  * 本车低速：v_ego <= STOP_SOFTEN_V_EGO_MAX_MS（默认 3.0 m/s ≈ 10.8 km/h）
  * 前车静止：v_lead <= STOP_SOFTEN_V_LEAD_STATIC_MS（默认 1.0 m/s ≈ 3.6 km/h）
  * 目标距离 > STOP_SOFTEN_D_HARD_M（默认 2.0 m）

为什么用「本车低速」而不是「本车静止」
──────────────────────────────────────────────────────────────────────────
  用户描述的是"本车静止"，但柔化窗口必须覆盖**起步后的靠近过程**：
  若只在 v_ego == 0 时生效，车一起步柔化立刻消失 ⇒ 又变回"被按死" ⇒ 顿挫。
  所以取一个低速区间（≤3 m/s），并**复用** gas_override 的 LOW_SPEED_MS 语义。

动作（与 gas_override 同构，只抬下限）
──────────────────────────────────────────────────────────────────────────
    output_a_target = max(output_a_target, floor)

  floor = min(A_SOFT, a_required)      ← 取更负者（更保守的一方）
  a_required = -(v_ego²) / (2 * (d_rel - D_MARGIN))

  * A_SOFT = −0.5 m/s²：期望的柔化减速（≈0.05 g，体感"轻轻带住"）
  * a_required：**物理下界** —— 「刚好能在剩余 (d_rel − D_MARGIN) 米内停住」
    所需的减速度。距离越紧、车速越高，它越负，于是地板自动收紧。

★ **安全论证（本设计的核心）**：
  柔化后的减速度**永远不低于**"恰好能在前车前 D_MARGIN 米处停住"所需的量。
  即：柔化只会让减速曲线更"前松后紧"，**不会导致停不住**。
  极端情况 a_required 比 ACCEL_MIN 还负时，取 ACCEL_MIN（=原逻辑硬刹上限）。

不干预的情形（任何一条成立即原样透传，a_target 逐位不变）
──────────────────────────────────────────────────────────────────────────
  * FCW 置位 / forceDecel                  —— 真安全信号，不参与柔化
  * 前车在动（v_lead > 阈值）              —— 常规跟车，交给 MPC 原逻辑
  * 本车不低速（v_ego > 阈值）             —— 中高速，交给 MPC 原逻辑
  * 距离 <= D_HARD（默认 2 m）             —— 与 gas_override 的 2 m 底线对齐；更近则硬刹
  * 无前车（lead.present 为假）

开关 / 回退
──────────────────────────────────────────────────────────────────────────
  * 代码级：STOP_SOFTEN_ENABLED = False  → 逐位等价于没装本模块（一行回退）
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
STOP_SOFTEN_V_EGO_MAX_MS = 3.0        # 本车低速上限 (≈10.8 km/h)
STOP_SOFTEN_V_LEAD_STATIC_MS = 1.0    # 前车视为静止 (≈3.6 km/h)
STOP_SOFTEN_D_HARD_M = 2.0            # 绝对距离底线：更近则完全不柔化

# ── 柔化参数 ──────────────────────────────────────────────────────────────
STOP_SOFTEN_A_SOFT = -0.5             # 期望的柔化减速 (m/s²)
STOP_SOFTEN_D_MARGIN_M = 1.0          # 停住时在前车前保留的余量 (m)

# 日志节流（与 gas_override 同款：状态跳变必落盘，其余按低频）
STOP_SOFTEN_LOG_HZ = 1.0


class StopSoftenResult:
  def __init__(self, a_target_out: float, active: bool, reason: str, log=None):
    self.a_target_out = a_target_out
    self.active = active
    self.reason = reason
    self.log = log


class StopSoftenController:
  """「低速 + 前方静止目标」下的减速柔化（只抬下限）。"""

  def __init__(self):
    self._last_reason = ''
    self._log_accum = 0.0

  def reset(self):
    self._last_reason = ''
    self._log_accum = 0.0

  # -- 内部：算地板 --------------------------------------------------------
  @staticmethod
  def _floor(v_ego: float, d_rel: float) -> float:
    """返回柔化地板（负数）。含物理下界，保证停得住。"""
    usable = max(d_rel - STOP_SOFTEN_D_MARGIN_M, 0.1)
    a_required = -(v_ego * v_ego) / (2.0 * usable)
    floor = min(STOP_SOFTEN_A_SOFT, a_required)   # 更负者胜 = 更保守
    return max(floor, ACCEL_MIN)

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float,
             fcw: bool, force_decel: bool, a_target_in: float,
             dt: float) -> StopSoftenResult:
    if not STOP_SOFTEN_ENABLED:
      return StopSoftenResult(a_target_in, False, 'disabled')

    # 真安全信号：一帧都不干预
    if fcw or force_decel:
      return self._ret(a_target_in, False, 'guard', dt)
    if not lead_present:
      return self._ret(a_target_in, False, 'no_lead', dt)
    if v_lead > STOP_SOFTEN_V_LEAD_STATIC_MS:
      return self._ret(a_target_in, False, 'lead_moving', dt)
    if v_ego > STOP_SOFTEN_V_EGO_MAX_MS:
      return self._ret(a_target_in, False, 'v_high', dt)
    if d_rel <= STOP_SOFTEN_D_HARD_M:
      return self._ret(a_target_in, False, f'close{d_rel:.1f}', dt)

    floor = self._floor(v_ego, d_rel)
    if a_target_in < floor:
      return self._ret(floor, True,
                       f'soften({a_target_in:.2f}->{floor:.2f},'
                       f'd{d_rel:.1f},v{v_ego:.2f})', dt)
    return self._ret(a_target_in, False, 'above', dt)

  def _ret(self, a_out: float, active: bool, reason: str, dt: float) -> StopSoftenResult:
    log = None
    if active:
      # 活动期间按 STOP_SOFTEN_LOG_HZ 节流落盘（避免每帧刷日志）
      self._log_accum += dt
      if self._log_accum >= 1.0 / STOP_SOFTEN_LOG_HZ:
        self._log_accum = 0.0
        log = f'[StopSoften] {reason}'
    else:
      self._log_accum = 0.0
    self._last_reason = reason
    return StopSoftenResult(a_out, active, reason, log)
