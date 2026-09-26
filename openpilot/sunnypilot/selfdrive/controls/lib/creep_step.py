#!/usr/bin/env python3
"""
定量蠕动步（第 8 版）—— 「松油门后自动再往前蠕动固定距离」。

需求（用户 2026-09-26 23:2x 原话，逐字）：
  「改成松油门，如果车间距大于 4，蠕动 1 米；如果小于 2 m，直接刹死；
    二者之间，蠕动 0.5 米」+「像油车怠速蠕行」

场景：本车静止 + 前车静止（闸门 crawl_gate 已打开），驾驶员点一下油门起步，
      松油门那一刻本模块接手，自动把车往前「推」一个定量步，走完即止，
      随后把控制权交还上游 e2e/MPC 温和收尾。
  ⇒ 一次「踩-松」= 一个完整的蠕动步，不需要反复点油门。
     这正是用户抱怨的「间隙性蠕动 / 一次一次往前凑」的解药：旧版松油门后
     只靠惯性滑（实测 1.4 km/h 只能滑 1.8 m），于是必须反复点。

为什么不再用第 7 版的 `-K*v` 地板（实测证据，swaglog 1185/1190）：
  松油门那一帧发出 `aFloor=-0.89`，但同一帧 `aOut=+0.00`；
  另一帧 `aFloor=-0.20 aOut=-0.20`。即**地板从未真正参与过运算** ——
  接线是 `a_target = max(a_target, floor)`，而上游在低速给的是 0 ~ -0.5，
  总是不低于地板 ⇒ 这个「只抬下限」的写法在**减法方向上是死的**。
  真正让车停下的是上游 e2e 自己的 `-0.17 ~ -0.50`（实车 23:22:24 起的
  `aTgt=-0.50 → -0.17 → -0.24`）。所以第 7 版「未踩油门」那一档等于没工作。

安全论证：
  * 触发前提 = **闸门已打开** = 本车已连续停稳 1.0 s + 前车静止 + dRel > 2 m
    （见 crawl_gate.py）。行进中减速接近红灯、前车在动、高速 —— 闸门不开。
  * 蠕动量由**位移积分** `moved = ∫v·dt` 限定，不依赖 dRel 的绝对值，
    因此 dRel 测距抖动不会放大成位移误差。走完即止。
  * 输出被夹在 [A_MIN, A_MAX]，且**只在下落沿触发的一次步内接管**；
    任何一项成立立即放弃、把控制权原样交还上游：
    闸门关闭 / 再踩油门 / 踩刹车 / 前车丢失 / 走完 / 超时 / 停滞 /
    **上游给了强减速（a_target ≤ SAFE_A）或 FCW/forceDecel**。
  * ★独立的「近距硬底线」：前车静止 + dRel ≤ 2.0 m + 本车低速 ⇒ 直接 -2.0。
    闸门内外都生效（闸门本身在 dRel ≤ 2 m 就关了，所以这条必须独立）。
  * 本模块永不产生「比上游更激进」的减速：负数只用于本步自己的收尾，
    且一旦发现上游更强就立刻让位（见上面的 safe 例外）。
"""

# ── 总开关与分档 ────────────────────────────────────────────────────────────
CREEP_STEP_ENABLED: bool = True

CREEP_STEP_FAR_GAP_M: float = 4.0    # 「车间距大于 4」⇒ 走远档
CREEP_STEP_FAR_M: float = 1.0        #   远档 = 蠕动 1.0 m
CREEP_STEP_NEAR_GAP_M: float = 2.0   # 「小于 2 m」⇒ 不蠕、直接刹
CREEP_STEP_NEAR_M: float = 0.5       #   二者之间 = 蠕动 0.5 m

# ── 位置闭环 ────────────────────────────────────────────────────────────────
# a = KP·(剩余距离) − KD·v ：起步时推、接近目标时自然收、超速时先刹一下。
CREEP_STEP_KP: float = 1.0
CREEP_STEP_KD: float = 1.1
CREEP_STEP_A_MAX: float = 0.8        # 推进上限（怠速蠕行的力道）
CREEP_STEP_A_MIN: float = -1.2       # 收尾刹车上限（比上游温和；上游更强时让位）

# ── 结束判据 ────────────────────────────────────────────────────────────────
CREEP_STEP_DONE_M: float = 0.03      # 剩余 ≤ 3 cm 视为走完
CREEP_STEP_NEAR_DONE_M: float = 0.10  # 虽停住但已到 90% 以上 ⇒ 也算走完（不算异常）
CREEP_STEP_TIMEOUT_S: float = 8.0    # 单步最长 8 s
CREEP_STEP_STALL_S: float = 1.2      # 连续 1.2 s 没挪动 2 cm ⇒ 判停滞
CREEP_STEP_STALL_M: float = 0.02
CREEP_STEP_SAFE_A: float = -1.0      # 上游强减速 ⇒ 立刻让位（安全例外）

# ── 近距硬底线（独立生效）────────────────────────────────────────────────────
CREEP_HARD_GAP_M: float = 2.0
CREEP_HARD_A: float = -2.0
CREEP_HARD_V_MS: float = 1.5         # 只在低速时判（≈5.4 km/h）
CREEP_HARD_V_LEAD_MS: float = 1.5    # 前车静止（滞回口径与闸门一致）

CREEP_LOG_HZ: float = 1.0


class CreepStep:
    """松油门 ⇒ 定量蠕动一步。返回值语义见 planner 接线段。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._active = False
        self._step = 0.0
        self._moved = 0.0
        self._t = 0.0
        self._stall_t = 0.0
        self._moved_at_stall = 0.0
        self._prev_gas = False
        self._log_t = 0.0
        self._last_tag = 'off'
        # ── 对外输出 ──
        self.a_target = 0.0
        self.hard_stop = False
        self.log = None
        self.last_reason = ''

    @property
    def active(self) -> bool:
        return self._active

    @property
    def step_m(self) -> float:
        return self._step

    @property
    def moved_m(self) -> float:
        return self._moved

    # ── 内部 ────────────────────────────────────────────────────────────────
    def _finish(self, reason: str) -> None:
        self._active = False
        self.last_reason = reason

    def update(
        self,
        *,
        v_ego: float,
        gas_pressed: bool,
        d_rel: float,
        lead_present: bool,
        v_lead: float,
        crawl_gate: bool,
        brake_pressed: bool,
        fcw: bool,
        force_decel: bool,
        a_target_in: float,
        dt: float,
    ) -> None:
        """每一帧调用一次；结果写在 self.a_target / self.hard_stop / self.log。

        Args:
          v_ego: 本车车速 (m/s)
          gas_pressed: carState.gasPressed
          d_rel: radarState.leadOne.dRel (m)
          lead_present: radarState.leadOne.present
          v_lead: radarState.leadOne.vLead (m/s)
          crawl_gate: crawl_gate.update() 的返回值（闸门是否打开）
          brake_pressed: carState.brakePressed
          fcw: 前向碰撞预警是否置位
          force_decel: controlsState.forceDecel
          a_target_in: 上游（含此前各柔性模块）算出的 a_target，用于安全例外
          dt: 控制周期 (s)
        """
        self.log = None
        self.a_target = 0.0

        # ---- 近距硬底线（独立于蠕动步；闸门内外都生效）----
        # 用户：「如果小于 2 m，直接刹死」+「至少保持 2 米车距」。
        self.hard_stop = bool(
            CREEP_STEP_ENABLED
            and lead_present
            and d_rel <= CREEP_HARD_GAP_M
            and v_ego <= CREEP_HARD_V_MS
            and v_lead <= CREEP_HARD_V_LEAD_MS)
        if self.hard_stop:
            self.a_target = CREEP_HARD_A
            self._finish('hard_gap')
            self._prev_gas = bool(gas_pressed)
            self._emit(v_ego, d_rel, gas_pressed, dt)
            return

        gas_falling = self._prev_gas and not gas_pressed
        self._prev_gas = bool(gas_pressed)

        # ---- 状态推进 ----
        if self._active:
            # 每帧先积分位移（用非负速度，倒溜不计入「前进量」）
            self._moved += max(0.0, v_ego) * dt
            self._t += dt

            reason = ''
            if not crawl_gate:
                reason = 'gate_closed'
            elif gas_pressed:
                reason = 'gas'
            elif brake_pressed:
                reason = 'brake'
            elif not lead_present:
                reason = 'lead_lost'
            elif a_target_in <= CREEP_STEP_SAFE_A or fcw or force_decel:
                # 安全例外：上游已在强刹 / 碰撞预警 / 强制减速 ⇒ 立刻让位
                reason = 'safe'
            elif self._moved >= self._step - CREEP_STEP_DONE_M:
                reason = 'done'
            elif self._t >= CREEP_STEP_TIMEOUT_S:
                reason = 'timeout'
            else:
                self._stall_t += dt
                if self._stall_t >= CREEP_STEP_STALL_S:
                    if self._moved - self._moved_at_stall < CREEP_STEP_STALL_M:
                        # 车不再往前挪了。若已经基本到位（≥90%）⇒ 算正常走完，
                        # 免得"差最后几厘米"被记成异常；否则判停滞（坡道/阻力/被按住）。
                        reason = ('done' if self._moved >= self._step - CREEP_STEP_NEAR_DONE_M
                                  else 'stall')
                    self._stall_t = 0.0
                    self._moved_at_stall = self._moved

            if reason:
                self._finish(reason)
        else:
            # 触发：闸门内 + 油门下降沿 + 距离分档
            if (CREEP_STEP_ENABLED
                    and crawl_gate
                    and gas_falling
                    and lead_present
                    and d_rel > CREEP_STEP_NEAR_GAP_M):
                self._step = (CREEP_STEP_FAR_M if d_rel > CREEP_STEP_FAR_GAP_M
                              else CREEP_STEP_NEAR_M)
                self._moved = 0.0
                self._t = 0.0
                self._stall_t = 0.0
                self._moved_at_stall = 0.0
                self._active = True
                self.last_reason = ''
                self._log_t = 999.0        # 触发帧立刻留痕

        # ---- 输出（位置闭环；收尾时的负值由 planner 以 max 接线，上游更强时上游赢）----
        if self._active:
            a_cmd = (CREEP_STEP_KP * (self._step - self._moved)
                     - CREEP_STEP_KD * v_ego)
            self.a_target = float(min(max(a_cmd, CREEP_STEP_A_MIN), CREEP_STEP_A_MAX))

        self._emit(v_ego, d_rel, gas_pressed, dt)

    def _emit(self, v_ego: float, d_rel: float, gas_pressed: bool, dt: float) -> None:
        """日志：状态跳变必打 + 蠕动期间 1 Hz。（cloudlog 最低只落 INFO）"""
        self._log_t += dt
        tag = 'hard' if self.hard_stop else ('creep' if self._active else 'off')
        show = False
        if tag != self._last_tag:
            if tag != 'off' or self._last_tag != 'off':
                show = True
            self._last_tag = tag
            self._log_t = 0.0
        elif self._active and self._log_t >= CREEP_LOG_HZ:
            show = True
            self._log_t = 0.0
        if show:
            self.log = (f"[CreepStep] {tag} step={self._step:.2f} "
                        f"moved={self._moved:.2f} dRel={d_rel:.1f} "
                        f"vEgo={v_ego * 3.6:.1f} gas={int(bool(gas_pressed))} "
                        f"aOut={self.a_target:+.2f} why={self.last_reason or '-'}")
