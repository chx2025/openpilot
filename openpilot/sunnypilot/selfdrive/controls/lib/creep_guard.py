"""
creep_guard.py —— 「前车静止时，不踩油门就不许自己贴到 2 m」守卫（第 6 版，2026-09-26）

需求（用户 2026-09-26 22:05 实车反馈，原话）
  「前方在车等红灯（静止），我车开过去后慢慢刹停，竟然会慢慢的往 2 米车距靠，
    然后停止，这也停太近了，而且也不是我要的。
    我要改的是前车静止，本车静止，我踩油门能蠕动过去，然后柔和的停下」
  ⇒ 「靠近到 2 m」这件事必须**只由驾驶员的脚（油门）决定**；
     脚不动时，车应该停在常规跟车距离（≈3.5 m）就结束。

为什么需要这个模块（实车取证，日志 swaglog.0000001114/1115）
  * 2026-09-26 14:05:28 那次"开过去刹停"，`[LongCtrl]` 只有
      v=10.8 aTgt=-1.31 → 6.7 -0.85 → 3.6 -0.65 → 0.9 -0.28
    上游（ExperimentalMode=True 时的 e2e）在前车静止、距离尚可时只给很软的
    减速 ⇒ 10.8 km/h 滑行约 5 m 才停 ⇒ 最终贴到 ≈2 m。
  * 同一时段 `[CrawlGate]` / `[CrawlAid]` / `[StopSoften]` **一条日志都没有**
    ⇒ 三个柔性模块在"正常跟车刹停"里**完全没参与**，「贴到 2 m」不是它们干的，
    所以修法只能是"在它们之外补一道下游兜底"，而不是动它们。

机制（只在下游补一点减速，绝不放行）
    usable   = max(dRel - HOLD_GAP, 0.1)
    a_needed = -(v_ego²) / (2 · usable)     # 「在 HOLD_GAP 处正好停住」所需的减速度
    a_target = min(a_target, a_needed)      # 只在它比上游更保守时才生效

  * HOLD_GAP = 3.5 m ⇒ 停位落在约 3.0 ~ 3.5 m，而不是 2 m；
  * 距离越远 a_needed 越接近 0（30 m 外约 −0.15）⇒ 带死区，不介入正常的巡航减速；
  * v_ego ≤ 3 m/s（10.8 km/h）才生效 ⇒ 与既有"低速蠕动"语义一致，不碰高速工况。

与既有模块的关系（重要，改顺序前必读）
  * **踩油门时完全不介入**（gasPressed 优先）⇒ 「爬向 2 m」仍然只由驾驶员的脚触发；
    stop_soften 的 MIN_GAP=2.0 与 crawl_gate 的放行逻辑**一个字不改**；
  * 与 crawl_gate 的放行（max 抬地板）方向相反，所以**必须排在 crawl aid 与
    stop_soften 之后**（planner 里排在 stop_soften 之后、np.clip 之前），
    否则收紧量会被它们的 max 又抬回去；
  * 只**加**减速（min）⇒ 不可能削弱 FCW / forceDecel / MPC / turn_decel 任何一方
    "更保守"的结论；FCW / forceDecel 期间一帧都不参与。

安全
  * 生效条件：有前车 + 前车静止 + 本车未踩油门 + 本车 ≤10.8 km/h；
  * 前车起步（vLead > 1.0 m/s）立即退出 ⇒ 不拖慢正常跟车；
  * dRel 被压到 HOLD_GAP 以内后，a_needed 随可用距离平方倒数收紧（下限
    CREEP_GUARD_A_MIN）⇒ 距离很紧时仍会给足减速，不会因为"设了目标距离"而追尾。

回退
  CREEP_GUARD_ENABLED = False  ⇒ 立刻恢复第 5 版行为（无残留状态）。
"""

CREEP_GUARD_ENABLED: bool = False   # ★7版撤下：第 6 版基于错误诊断（见爬行门第 7 版说明）
CREEP_GUARD_HOLD_GAP_M: float = 3.5        # 目标停位（本车车头到前车车尾）
CREEP_GUARD_V_MAX: float = 3.0             # 本车速度上限 (≈10.8 km/h)，超过不介入
CREEP_GUARD_V_LEAD_STATIC_MS: float = 1.0  # 前车视为静止 (≈3.6 km/h)
CREEP_GUARD_DEADBAND: float = -0.15        # a_needed 高于此值（距离还远）视为不介入
CREEP_GUARD_A_MIN: float = -3.0            # 收紧量下限（不许超过这个强度）
CREEP_GUARD_LOG_HZ: float = 1.0            # 活动期间/低速巡检的日志频率
CREEP_GUARD_TRACE_GAP_M: float = 15.0      # 巡检日志：只在 dRel 小于此值时才打
CREEP_GUARD_TRACE_V_MAX: float = 5.0       # 巡检日志：只在本车低于此速度时才打


class CreepGuardResult:
  def __init__(self, a_target_out: float, active: bool, reason: str, log=None):
    self.a_target_out = a_target_out
    self.active = active
    self.reason = reason
    self.log = log


class CreepGuard:
  """「前车静止 + 本车未踩油门」时的自动靠近守卫（只压上限）。"""

  def __init__(self):
    self._last_reason = ''
    self._log_accum = 0.0
    self._trace_accum = 0.0
    self._was_active = False

  def reset(self):
    self._last_reason = ''
    self._log_accum = 0.0
    self._trace_accum = 0.0
    self._was_active = False

  # -- 内部：算收紧量 ------------------------------------------------------
  @staticmethod
  def _clamp_target(v_ego: float, d_rel: float) -> float:
    """返回「在 HOLD_GAP 处正好停住」所需的减速度（负数）。"""
    usable = max(d_rel - CREEP_GUARD_HOLD_GAP_M, 0.1)
    a_needed = -(v_ego * v_ego) / (2.0 * usable)
    return max(a_needed, CREEP_GUARD_A_MIN)

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float,
             gas_pressed: bool, fcw: bool, force_decel: bool,
             a_target_in: float, dt: float) -> CreepGuardResult:
    # 低速 + 有前车才落巡检日志（等红灯时约 1 条/秒），其余情况保持安静
    trace = bool(lead_present and v_ego <= CREEP_GUARD_TRACE_V_MAX
                 and 0.0 < d_rel <= CREEP_GUARD_TRACE_GAP_M)

    if not CREEP_GUARD_ENABLED:
      return self._ret(a_target_in, False, 'disabled', dt, trace)

    # ★ 驾驶员的脚优先：踩油门时本模块一帧都不参与（蠕动/靠近全靠油门）
    if gas_pressed:
      return self._ret(a_target_in, False, 'gas', dt, trace)

    # 真安全信号：一帧都不参与
    if fcw or force_decel:
      return self._ret(a_target_in, False, 'guard', dt, trace)

    if not lead_present:
      return self._ret(a_target_in, False, 'no_lead', dt, trace)
    if d_rel <= 0.0:
      return self._ret(a_target_in, False, 'no_dist', dt, trace)
    if v_lead > CREEP_GUARD_V_LEAD_STATIC_MS:
      return self._ret(a_target_in, False, 'lead_moving', dt, trace)
    if v_ego > CREEP_GUARD_V_MAX:
      return self._ret(a_target_in, False, 'v_high', dt, trace)

    a_guard = self._clamp_target(v_ego, d_rel)
    if a_guard > CREEP_GUARD_DEADBAND:
      # 距离还远：所需减速很轻，属正常巡航，不介入
      return self._ret(a_target_in, False, f'far d{d_rel:.1f}', dt, trace)
    if a_target_in > a_guard:
      return self._ret(a_guard, True,
                       f'hold d{d_rel:.1f} v{v_ego:.2f} '
                       f'aIn={a_target_in:+.2f} aOut={a_guard:+.2f}', dt, trace)
    return self._ret(a_target_in, False,
                     f'above d{d_rel:.1f} v{v_ego:.2f}', dt, trace)

  def _ret(self, a_out: float, active: bool, reason: str, dt: float,
           trace: bool = False) -> CreepGuardResult:
    log = None
    if active:
      # 活动期间按 CREEP_GUARD_LOG_HZ 节流；**进入当帧必打**
      self._log_accum += dt
      if not self._was_active or self._log_accum >= 1.0 / CREEP_GUARD_LOG_HZ:
        self._log_accum = 0.0
        log = f'[CreepGuard] {reason}'
    else:
      # **退出当帧必打**（否则"为什么不再收紧了"又成盲区）
      if self._was_active:
        log = f'[CreepGuard] release({reason})'
      self._log_accum = 0.0
      # 未介入时的低速巡检：用来核对"车到底停在了几米"
      if trace:
        self._trace_accum += dt
        if self._trace_accum >= 1.0 / CREEP_GUARD_LOG_HZ:
          self._trace_accum = 0.0
          log = f'[CreepGuard] idle({reason})'
    self._was_active = active
    self._last_reason = reason
    return CreepGuardResult(a_out, active, reason, log)
