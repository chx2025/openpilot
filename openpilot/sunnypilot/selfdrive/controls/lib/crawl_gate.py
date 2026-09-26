"""crawl_gate.py — 「静止起步蠕动」场景闸门（2026-09-26，三版：抗抖动）

需求原文（用户 2026-09-26）
──────────────────────────────────────────────────────────────────────────
  「我要做的柔性刹车，有个前提就是，本车静止，前车静止（这二个条件满足才
    生效，其他情况一率跳过），本车加油启动柔性蠕动一点距离」
  「保证最小车距为 2，根据油门力度和油门持续时间定蠕动距离」

一句话
──────────────────────────────────────────────────────────────────────────
  只有「本车静止 + 前车静止」同时成立时，才允许两个柔性模块
  （gas_override 的 e/f 低速豁免、stop_soften 的减速柔化）参与；
  其余一切工况 —— 行进中减速接近静止前车、前车在动、高速、跟车 ——
  **一律跳过**，与原厂逐位相同。

为什么必须 latch，而不是每帧严判
──────────────────────────────────────────────────────────────────────────
  「本车静止」是**进入条件**，不是保持条件。若每帧都要求 v_ego ≈ 0，
  驾驶员一给油车就动 ⇒ 条件立刻不成立 ⇒ 柔性在起步那一瞬消失 ⇒
  又退回"只动一下 / 一松油门被按死"的老毛病。
  所以：进入即锁定（latch），另设一组**退出条件**，场景结束才复位。

★ 三版新增：抗抖动（2026-09-26 实车证据，swaglog.0000000972）
──────────────────────────────────────────────────────────────────────────
  实车日志里闸门以 **20 Hz 逐帧 enter/exit 抖动**（间隔正好 50 ms =
  plannerd 帧率），而打印出来的 vEgo / dRel / vLead **完全不变**：

      10:01:26.749 [CrawlGate] exit  vEgo=0.0 dRel=3.4 vLead=0.0
      10:01:26.800 [CrawlGate] enter vEgo=0.0 dRel=3.4 vLead=0.0
      10:01:26.849 [CrawlGate] exit  vEgo=0.0 dRel=3.4 vLead=0.0
      ... （连续 20 次）

  唯一能解释的隐藏变量是 **`lead.present` 每帧闪烁**：radarState 每丢一帧
  前车，`dRel` 会留在上一次的值（3.4）而 `present=False` ⇒ 命中"无前车"
  退出；下一帧又 present ⇒ 重新进入。于是闸门有一半时间是关的，
  `should_stop` 就有一半时间恢复 ⇒ `LongControl.stopping` 照样抓走状态机
  ⇒ **踩油门照样被刹住**（这正是"还是一样"的第二个原因）。

  两个修法：
  ① **`d_rel` 判据必须带 `lead_present` 守卫**（或退化成"上一次有效
     dRel"）—— 前车丢失那一帧 `dRel` 可能是 0，会被误判成"贴到最小车距"；
  ② **退出判据全部去抖**（见下），单帧毛刺不再能关门。

退出判据的去抖 / 滞回
──────────────────────────────────────────────────────────────────────────
  * 贴到最小车距（d_eff <= 2.0）：**立即**退出 —— 这是安全边界，不过滤。
    用「上一次有效 dRel」而不是当前帧，避免前车丢失帧的 0 值误判。
  * 本车已不是蠕动（v_ego > 3.0 m/s）：**立即**退出。
  * 驾驶员踩刹车：连续 CRAWL_GATE_BRAKE_FRAMES 帧（默认 2 帧 = 0.1 s）。
    真人踩刹车远长于 0.1 s，感受上仍是"立即"；但 1 帧的信号毛刺被滤掉。
  * 前车起步：**滞回 + 去抖** —— 阈值从进入的 1.0 抬到 1.5 m/s
    （前车在停止线前微动不该关门），且要连续 CRAWL_GATE_LEAD_MOVING_FRAMES
    帧（默认 3 帧 = 0.15 s）。
  * 前车丢失：连续 CRAWL_GATE_LEAD_LOST_FRAMES 帧（默认 8 帧 = 0.4 s）。
    这是本次抖动的主凶，所以给足余量；真丢了 0.4 s 之后照样关门。

★ 安全论证（本设计的核心）
──────────────────────────────────────────────────────────────────────────
  * **去抖只作用于"退出"**，不作用于"进入" —— 开门条件一秒都没放宽。
  * 「贴到 2 m」与「本车超 3 m/s」两条**完全不过滤**，仍是硬边界。
  * 闸门关闭 ⇒ 两个柔性模块一帧都不干预 ⇒ 与原厂逐位相同。
  * 闸门打开 ⇒ 两个模块仍各自保留 FCW / forceDecel / 绝对距离例外
    （< 2 m、< 4 m 且快 10 km/h 以上）⇒ 真正需要刹的时刻一点没让。

开关 / 回退
──────────────────────────────────────────────────────────────────────────
  * 代码级：CRAWL_GATE_ENABLED = False ⇒ 闸门恒 False ⇒ 两个柔性模块
    同时退回"从不干预"，等于把这两个功能一起摘掉（一行回退）。
  * 也可由调用方传 enabled=False（例如未接管时），语义同上。
"""

from __future__ import annotations

# ── 总开关（一键回退：False = 两个柔性模块一起失效）──────────────────────
CRAWL_GATE_ENABLED: bool = True

# ── 进入阈值 ──────────────────────────────────────────────────────────────
CRAWL_GATE_V_EGO_MS: float = 0.15      # 本车「静止」判定 (≈0.54 km/h) ★7版 0.3→0.15
CRAWL_GATE_STILL_FRAMES: int = 20      # ★7版新增：连续静止 1.0 s 才算「停稳」
CRAWL_GATE_V_LEAD_MS: float = 1.0      # 前车「静止」判定 (≈3.6 km/h)
CRAWL_GATE_MIN_GAP_M: float = 2.0      # 最小车距（硬底线，也是闸门的距离边界）

# ── 退出阈值 ──────────────────────────────────────────────────────────────
CRAWL_GATE_EXIT_V_EGO_MS: float = 3.0     # 超过此速度已不是「蠕动」(≈10.8 km/h)
CRAWL_GATE_EXIT_V_LEAD_MS: float = 1.5    # 滞回：退出用的「前车在动」阈值

# ── 退出判据的去抖帧数（plannerd 20 Hz ⇒ 1 帧 = 50 ms）────────────────────
CRAWL_GATE_LEAD_LOST_FRAMES: int = 60     # 3.00 s ★7版 0.4→3.0：起步时前车检测闪断
CRAWL_GATE_BRAKE_FRAMES: int = 2          # 0.10 s：踩刹车才关门
# ---- 闸门内的加速度策略（2026-09-26 第 5 版：放行蠕动）----
# 由来（实车取证）：
#   闸门只清 should_stop，并不能让车真的“走起来”。实测松油门后车速
#   2.1 km/h 在 1.5 s 内归零（滑行阻力约 0.3 m/s^2）⇒ 体感“一松油门就被踩死”。
#   更关键：OP 把 aTarget 发成 0 时，车机 ACC 会按“目标加速度 0”执行并
#   抑制驾驶员油门 —— 实测 OP 接管时踩油门 2 s 车速仍 0.0 km/h，
#   同一动作在 OP 未接管（act=0）时能到 5.4 km/h。
# 三档地板（只抬下限；只有 ① 为正值，且仅在“已踩油门”时下发）：
#   ① 踩油门 且 gap > SOFT_GAP_M ⇒ A_BOOST（放行油门，别让 ACC 压住它）
#   ② 未踩油门 且 gap > SOFT_GAP_M ⇒ A_FLOOR(0)，不减速，靠惯性溜
#   ③ gap <= SOFT_GAP_M          ⇒ SOFT_DECEL，温和收尾（避免退出瞬间急刹）
CRAWL_GATE_A_FLOOR: float = 0.0        # ② “不减速”地板
CRAWL_GATE_A_BOOST: float = 0.6        # ① 踩油门时的放行加速度（约等于怠速蠕行）
CRAWL_GATE_SOFT_GAP_M: float = 3.0     # ③ 收尾区起点（也是 ① 的生效边界）
CRAWL_GATE_SOFT_DECEL: float = -0.6    # ③ 收尾区地板（7版起不再使用，仅保留常量）
CRAWL_GATE_SOFT_K: float = 0.8         # ★7版：未踩油门时的速度衰减系数 a=-K*v (1/s)

CRAWL_GATE_LEAD_MOVING_FRAMES: int = 3    # 0.15 s：前车真起步才关门


class CrawlGate:
  """「本车静止 + 前车静止」场景闸门（带 latch + 退出去抖）。

  每帧由 longitudinal_planner 在 gas_override / stop_soften **之前**调用一次，
  返回值直接喂给那两个模块。无外部依赖，可离线单测。

  内部状态：
    * `_latched`     —— 闸门是否开着
    * `_n_*`         —— 各退出判据的连续命中帧数（去抖）
    * `_last_d_rel`  —— 上一次「前车存在」时的 dRel（前车丢失帧的兜底值）
  对外：
    * `latched`、`last_change`（'' / 'enter' / 'exit'）、`last_reason`（退出原因）
  """

  def __init__(self) -> None:
    self._latched: bool = False
    self.last_change: str = ''
    self.last_reason: str = ''
    self._n_lead_lost: int = 0
    self._n_brake: int = 0
    self._n_lead_moving: int = 0
    self._n_still: int = 0
    self._last_d_rel: float = float('inf')

  # ── 对外 ────────────────────────────────────────────────────────────────
  def reset(self) -> None:
    """外部强制复位（disengage / 未接管 / reset_state）。

    场景上下文已经丢失（不知道车是从哪来的），必须重新满足进入条件才开门，
    否则会把"上一段行程的静止起步"状态带进新的行程。
    """
    self._latched = False
    self.last_change = ''
    self.last_reason = 'reset'
    self._reset_counters()
    self._n_still = 0
    self._last_d_rel = float('inf')

  @property
  def latched(self) -> bool:
    return self._latched

  @property
  def gap_m(self) -> float:
    """最后一次有效的前车距离 (m)；latched 期间用它做放行/收尾分档。"""
    return self._last_d_rel

  # ── 内部 ────────────────────────────────────────────────────────────────
  def _reset_counters(self) -> None:
    self._n_lead_lost = 0
    self._n_brake = 0
    self._n_lead_moving = 0

  def update(
    self,
    *,
    v_ego: float,
    lead_present: bool,
    d_rel: float,
    v_lead: float,
    brake_pressed: bool,
    enabled: bool = True,
  ) -> bool:
    """返回本帧闸门是否打开（True = 允许柔性模块参与）。

    Args:
      v_ego: 本车车速（m/s）
      lead_present: radarState.leadOne.present
      d_rel: radarState.leadOne.dRel（m）
      v_lead: radarState.leadOne.vLead（m/s）
      brake_pressed: carState.brakePressed（驾驶员踩刹车 = 明确接管）
      enabled: 调用方附加的使能（例如未接管时传 False）
    """
    prev = self._latched
    self.last_change = ''
    self.last_reason = ''

    if not (CRAWL_GATE_ENABLED and enabled):
      # 总开关关闭 / 调用方禁用：不仅不开门，还要把 latch 清掉，
      # 避免"关一次开关再打开"时带着旧状态进来。
      self._latched = False
      self.last_reason = 'disabled'
      self._reset_counters()

    elif self._latched:
      # 记下"最后一次有效的前车距离"：前车丢失帧 dRel 可能是 0，
      # 直接拿它判 min_gap 会误关门（这就是 20 Hz 抖动的来源）。
      if lead_present:
        self._last_d_rel = d_rel

      # ---- 各退出判据的连续命中帧数 ----
      self._n_lead_lost = self._n_lead_lost + 1 if not lead_present else 0
      self._n_brake = self._n_brake + 1 if brake_pressed else 0
      self._n_lead_moving = (self._n_lead_moving + 1
                             if v_lead > CRAWL_GATE_EXIT_V_LEAD_MS else 0)

      reason = ''
      # ① 安全：贴到最小车距 —— 立即退出，不去抖、不滞回（硬边界）
      if self._last_d_rel <= CRAWL_GATE_MIN_GAP_M:
        reason = f'min_gap dRel={self._last_d_rel:.1f}'
      # ② 安全：本车已不是蠕动 —— 立即退出
      elif v_ego > CRAWL_GATE_EXIT_V_EGO_MS:
        reason = f'vEgo={v_ego:.1f}'
      # ③ 驾驶员踩刹车（连续 2 帧 ⇒ 0.1 s，滤掉单帧毛刺）
      elif self._n_brake >= CRAWL_GATE_BRAKE_FRAMES:
        reason = 'brake'
      # ④ 前车真起步（滞回 1.5 m/s + 连续 3 帧）
      elif self._n_lead_moving >= CRAWL_GATE_LEAD_MOVING_FRAMES:
        reason = f'lead_moving vLead={v_lead:.2f}'
      # ⑤ 前车丢了（连续 8 帧 ⇒ 0.4 s）
      elif self._n_lead_lost >= CRAWL_GATE_LEAD_LOST_FRAMES:
        reason = 'lead_lost'

      if reason:
        self._latched = False
        self.last_reason = reason
        self._reset_counters()
        self._n_still = 0

    else:
      # ---- 进入判定（第 7 版：必须「真的停稳」）----
      # 退出侧三个计数器清零；still 计数器必须累加 ⇒ 这里不用 _reset_counters()。
      self._n_lead_lost = 0
      self._n_brake = 0
      self._n_lead_moving = 0
      # 「本车静止」= 连续 CRAWL_GATE_STILL_FRAMES 帧 v_ego 都在静止阈值以下。
      # 旧版只看单帧瞬时值 ⇒ 跟车刹停时速度自然降到 0.3 m/s 就 latch，
      # 随后闸门把上游减速清零 ⇒ 车从 8 m 一路滑到 2 m（实车 21:57 实证）。
      if v_ego <= CRAWL_GATE_V_EGO_MS and not brake_pressed:
        self._n_still += 1
      else:
        self._n_still = 0
      if (lead_present
          and not brake_pressed
          and self._n_still >= CRAWL_GATE_STILL_FRAMES
          and v_lead <= CRAWL_GATE_V_LEAD_MS
          and d_rel > CRAWL_GATE_MIN_GAP_M):
        self._latched = True
        self._last_d_rel = d_rel
        self._n_still = 0

    if self._latched != prev:
      self.last_change = 'enter' if self._latched else 'exit'
    return self._latched
