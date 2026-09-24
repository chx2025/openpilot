#!/usr/bin/env python3
"""e2e 候选「欠速让位」门控

============================================================================
现象（实车取证 2026-09-21 14:28~15:10 北京，CTM 大模型 + 全实验模式）
============================================================================
用户反馈：「直行踩油门后要 5 秒以上才恢复加速；无前车或前车远时才出现」。
探针原文（1 Hz 留痕 [E2eTrace]）：

  06:28:38 UTC  aE2e=+0.159 | aTgt=+0.159 src=4 aMpc=+1.80 aCruise=+2.00 |
                vEgo=41 vCruiseInt=55 gap=14.3 | rLead=0 mLeadProb=0.00 planA0=+0.0
  ……连续 31 秒，输出始终 +0.09~+0.16，而 MPC 一直想 +1.75、cruise 一直想 +2.00。

归档回放统计（2238 帧）：
  判据「gap ≥ 5 km/h 且 aTgt < 0.2 且 无前车/前车 > 50 m」命中 **502 帧 (23%)**；
  这些片段时长 **中位 6.1 s、均值 7.8 s、最长 32 s** —— 与用户体感「5 秒以后才恢复」吻合；
  其中 **489 帧 src=4（e2e 候选胜出，97%）**；
  aMpc 中位 **+1.73**、aCruise 中位 **+2.00**，而 aE2e 中位 **−0.04**、> +0.5 只占 4%。

============================================================================
机理：min() 把「我不想加速」误读成「不许你加速」
============================================================================
planner 的候选集是 `min(candidates, key=a_target)`：
    candidates = [MPC, cruise, (traffic_stop), (e2e)]
这个结构对**减速类**候选是对的（保守取最负）。但 e2e 输出不是「上限」而是**意图**：
CTM / BMV6 属「2026 Deep RL Models」，其 action 头的输入里**根本没有设定速度**
（input_shapes = img / big_img / desire_pulse / traffic_convention / action_t /
 features_buffer），它学的是「像人一样开车」——**空路上人类基本维持车速，所以它给 ≈0**。

于是 `min(aMpc=+1.80, aCruise=+2.00, aE2e=+0.10) = +0.10`，
车以 0.1 m/s² 往上爬，**永远追不上设定速度**；踩油门把速度推上去、松开后又掉回来。

模型差异（解释了「换回 CD210 就正常」）：
  · C210M（Release Models，**无 action 头**）→ `desiredAcceleration` 由 plan 轨迹反推，
    实测正值占 49.5%、峰值 +1.60 ⇒ 不压制加速 ⇒ 体感正常；
  · CTM / BMV6（2026 Deep RL Models，**有 action 头**）→ 直接输出「人类式」意图 ≈0
    ⇒ 压制 ⇒ 体感「加速无力」。

注意这与「IFE 硬件缩图」「CTM 适配」都无关：07-28（既无 CTM 也无 IFE）的
aE2e 中位就已经是 −0.810。这是**上游语义 + 本 fork 的 min() 组合**共同造成的。

============================================================================
改法
============================================================================
只在「**确实该加速** + **空路** + **模型自己没在减速/没看到前车**」三条同时成立时，
e2e 候选**不参与 min()** —— 把速度跟踪权还给 cruise / MPC。
其余任何情况（跟车、减速、模型要求制动、模型说要停）行为与改动前**完全一致**。

============================================================================
安全性（逐条论证）
============================================================================
1. 只在欠速 ≥ `E2E_GATE_MIN_GAP_MS`(2.0 m/s ≈ 7.2 km/h) 时开闸
   ⇒ 正常巡航、跟车、减速场景一概不受影响。
2. `a_e2e < E2E_GATE_MIN_A`(−1.0) 时**不开闸** ⇒ 模型 1.0 m/s² 以上的真实制动意图
   照旧压加速；（2026-09-21 由 −0.3 放宽到 −1.0，依据与核对见文末「修订记录」。）
3. `e2e_should_stop` 为真不开闸 ⇒ 模型说要停，绝不放行。
4. 雷达前车 ≤ `E2E_GATE_LEAD_M`(50 m) 不开闸。
5. **模型自己**的 `leadsV3[0].prob ≥ E2E_GATE_LEAD_PROB`(0.5) 不开闸
   ⇒ 覆盖「雷达看不到、模型看到」的前车。
6. 模型 plan 首点加速度 < `E2E_GATE_MIN_PLAN_A`(−1.0) 不开闸
   ⇒ 模型轨迹**明确**在减速时尊重它（远处行人/电瓶车/弯道等）。
7. 闸门只删掉 e2e **一条**候选；min() 里仍有
   MPC（带 radar 前车 + traffic_stop 虚拟停止线）与 cruise，
   且 `E2E_ACCEL_MAX_M_S2` 封顶、FCW、traffic_stop 均不受影响
   ⇒ **不可能**比改动前更激进地冲向障碍物。
8. 闸门只影响**加速侧**：被删的 e2e 值满足 `a_e2e ≥ E2E_GATE_MIN_A`(−1.0)，
   即那一条候选最多是 1.0 m/s² 以内的缓减速，不是紧急制动项；
   且同帧必须同时满足「欠速 + 雷达无近车 + 模型无前车 + e2eStop=0」。

量化（回放 2238 帧归档）：
  · 开闸 317 帧 (14%)，全部落在欠速段，车速分布 0-20:25 / 20-40:159 / 40-60:133；
  · 找回加速度 **中位 +1.57 m/s²**、均值 +1.53、最大 +1.90；
  · 被开闸帧原本的 aE2e：中位 +0.187、仅 22% 为负。

============================================================================
调参 / 回退
============================================================================
  · 一键回退：`E2E_ACCEL_GATE_ENABLE = False`（完全恢复原行为）。
  · 想更保守：把 `E2E_GATE_MIN_A` / `E2E_GATE_MIN_PLAN_A` 改回 −0.3
    （2026-09-21 之前的取值），或把 `E2E_GATE_MIN_GAP_MS` 提到 3.0。
  · 想更激进：把两者降到 −1.5（注意：会开始吃掉模型 1.5 m/s² 以内的减速意图）。
  · 关掉后如果「该加速却不加速」回来，先 grep [E2eTrace] 看 src 是不是 4。

============================================================================
修订记录
============================================================================
[2026-09-21 · 第 2 次修订] `E2E_GATE_MIN_A` 与 `E2E_GATE_MIN_PLAN_A`：−0.3 → −1.0

用户反馈（原话）：「方向盘大于 15 度回正后，有个空窗恢复期，这个恢复期感觉有点长」，
追问后澄清为 **「这个不用改，但我明明方向盘已经回正，前方还没有车也没有加速」**
（即：否掉了 turn_decel 方向，问题重新定位到「回正后仍不加速」）。

定位（归档回放：1292 帧带 `e2eGate=` 字段的补丁后帧）：
  · **不是 turn_decel** —— 整圈仅 13 帧被它压制（8 `paused` + 5 `big_angle_guard`，
    |steer| 中位 87.6°），而用户反馈的时段 `turnBlk=0`。
  · 真凶是 e2e 候选在「**前车刚走、本车已降到 5 km/h**」时仍滞后给负值，
    两个 −0.3 门限同时把它否掉：
        gap=50.2  aE2e=−0.784  mLeadProb=0.11  planA0=−0.631
        aMpc=+1.87  aCruise=+2.00   →   旧配置 aTgt=−0.78（挡住）→ 该起步却静止约 4 s

敏感性（同一份归档，四种组合，`min()` 之后的输出口径）：

  | 组合（a_e2e / planA0） | 全量开闸 | 占比  | 安全违规 | 「该加速却不加速」帧内开闸 |
  |-----------------------|---------|-------|---------|--------------------------|
  | 旧 −0.3 / −0.3        | 58      | 4.5%  | 0       | 10 (30%)                 |
  | 新 −1.0 / −1.0        | 70      | 5.4%  | 0       | 22 (67%)                 |
  | 只放宽 a_e2e          | 61      | 4.7%  | 0       | 13 (39%)                 |
  | 只放宽 plan_a0        | 60      | 4.6%  | 0       | 12 (36%)                 |

  两个门限各有独立贡献；合起来把「该加速却不加速」的开闸率从 30% 提到 **67%**。

新增开闸的 **12 帧**（旧否决、新放行）：
  · 找回加速度：中位 **+2.152**、均值 +1.867、最大 +2.421 m/s²；
  · **92% 的帧 `aMpc > 0.5`**（MPC 同帧也在喊加速）⇒ 是真能加上去的，不是硬顶；
  · 车速全落在 0~40 km/h ⇒ 正是起停/低速恢复段，与用户体感吻合；
  · **安全违规 0 帧**。

为什么放宽后仍然安全：门限放宽只增加「缓减速被忽略」的可能，而被忽略的那一条
候选本身 ≤ 1.0 m/s²（属模型滞后，不是紧急制动项）；同帧还必须同时满足
「欠速 ≥ 2 m/s + 雷达 50 m 内无车 + 模型自报前车 prob < 0.5 + e2eStop=0」，
且 `min()` 里 MPC（带 radar 前车与 traffic_stop 虚拟停止线）与 cruise 始终在。

回退：把这两个常量改回 −0.3；或 `E2E_ACCEL_GATE_ENABLE = False` 整体关闭。

----------------------------------------------------------------------------
[2026-09-21 · 第 3 次修订] 加**滞回 + 最短保持**，消除门控「抖动（flapping）」
----------------------------------------------------------------------------

用户反馈（原话）：「当前方没车，本车要开始加速的那一下，感觉有突然加速又突然停顿
又开始加速的情况，基本上每次都是，**感觉有东西在打架**。」

归档取证（本文件同目录 `analyze_flap.py`，1292 帧带 `e2eGate=` 的帧，32.7 min）：

  · 门控**翻转 38 次**，其中**单帧游程 11 次**（开一帧就关 / 关一帧就开）；
  · `aTgt` 相邻帧阶跃：`|Δ| ≥ 1.0` 共 **29 次 (2.3%)**、最大 **3.016 m/s²**；
  · 「加速→停顿→加速」三段往复片段 **12 个**，往复幅度中位 0.744、最大 1.805 m/s²；
  · 大阶跃归因：**门控翻转 + `src` 4↔0 切换**占绝对多数，典型形态是
      前帧 aTgt=−0.645 (src=4, gate=0) → 后帧 aTgt=+1.600 (src=0, gate=1)
    即「被 e2e 压着缓减速」**一步跨到**「满油门 +1.60」，Δ=+2.245 m/s²。

根因：`should_yield` 是**纯逐帧硬阈值判定** —— 无滞回、无最短保持时间。
三个布尔判据（`a_e2e`、`plan_a0`、`rLead`）的翻转频次实测分别为
108 / 146 / 42 次，且 `a_e2e`、`plan_a0` 的阈值恰好落在其分布最密集处
（`aE2e` 中位 −0.04 ~ −0.8），于是候选集里 e2e 一帧进一帧出：

    aTgt(开闸) = min(aMpc, aCruise, 1.6) ≈ +1.6
    aTgt(关闸) = min(aMpc, aCruise, aE2e) ≈ −0.6      ← 两个值差 2.2 m/s²

注意：**这是本门控自带的缺陷，不是 −1.0 引入的**。实测同一份归档：
旧阈值 −0.3 翻转 38 次，新阈值 −1.0 翻转 42 次 —— 放宽阈值只是让更多帧
够到开闸条件，**略微放大了本来就存在的抖动**，不是根因。

改法（两句）：给「是否开闸」这个布尔量加**状态**。

  1. **滞回（施密特触发器）**：进入开闸用严格阈值，**保持开闸用放宽阈值**。
     于是判据在两条线之间摆动时不会翻转，只有真正越过「退出线」才关闸。
  2. **最短保持时间**：开闸后 `E2E_GATE_MIN_HOLD_S`(2.0 s) 内，
     **软条件不再能关闸**（硬闸除外）。这直接消灭「开一帧就关」的单帧游程。

  | 判据 | 进入（关→开，严格） | 保持（开→不关，放宽） |
  |------|---------------------|----------------------|
  | 欠速 gap      | ≥ `E2E_GATE_MIN_GAP_MS` 2.0 | ≥ `E2E_GATE_HYST_GAP_MS` 1.2 |
  | `a_e2e`       | ≥ `E2E_GATE_MIN_A` −1.0      | ≥ `E2E_GATE_HYST_A` −1.6 |
  | `plan_a0`     | ≥ `E2E_GATE_MIN_PLAN_A` −1.0 | ≥ `E2E_GATE_HYST_PLAN_A` −1.6 |
  | 雷达前车 dRel | > `E2E_GATE_LEAD_M` 50      | > `E2E_GATE_HYST_LEAD_M` 40 |
  | 模型前车 prob | < `E2E_GATE_LEAD_PROB` 0.5  | < `E2E_GATE_HYST_LEAD_PROB` 0.6 |

  **硬闸**（任何时刻立即关闸，**不受最短保持保护**，因为它们是安全底线）：
    · `e2e_should_stop`          模型说要停
    · 雷达前车 `dRel ≤ 20 m`      近距前车
    · 模型前车 `prob ≥ 0.8`       模型高置信度看到前车
    · `a_e2e < −2.0`             模型要求 2.0 m/s² 以上制动
    · `plan_a0 < −2.0`           模型轨迹首点 2.0 m/s² 以上减速
    · `gap_ms < 0`               已超速

  最后两条是**实现最短保持时才发现必须补的**：没有它们，「保持 2 秒」会把
  模型突然给出的强烈制动意图也一起兜住 —— 那是不可接受的延迟。
  于是两条线的语义是：`−1.6` 是软退出线（越过后最多再保持 2 s），
  `−2.0` 是硬退出线（越过当帧关闸）；夹在中间的最坏情况是
  「模型给 2.0 m/s² 以内的减速意图、被忽略至多 2 s」，
  而此时同帧依然满足「雷达 40 m 内无车 + 模型前车 prob < 0.6」。

安全性论证（相对第 2 次修订版的变化面）：
  · 滞回只影响**「什么时候把 e2e 放回候选集」的时机**，不放宽任何进入条件
    ⇒ 该不开闸的场景（跟车/减速/模型要停）行为和之前**逐条一致**。
  · 最短保持期间若出现硬闸（近车/要停/高概率前车）→ **立即关闸**，
    不享受保持 ⇒ 安全底线不因防抖而被延迟。
  · 保持态最宽松处也只到「模型 1.6 m/s² 以内的缓减速被忽略」，
    仍然是**缓减速**，不是紧急制动项；且同帧依然要求雷达 40 m 内无车、
    模型前车 prob < 0.6。
  · 一键回退：`E2E_GATE_HYST_ENABLE = False`（回到第 2 次修订的纯阈值行为）。

实现：新增有状态类 `E2eAccelGate`（planner 侧持有实例），
`should_yield()` 纯函数**保留不变**（新增可选阈值覆盖参数），单测与离线回放照旧可用。
"""

# ── 总开关 ──
E2E_ACCEL_GATE_ENABLE: bool = True

# ── 开闸条件（全部成立才让位给 cruise/MPC）──
E2E_GATE_MIN_GAP_MS: float = 2.0      # 欠速门限（m/s）；v_cruise - v_ego 要大于它
# [2026-09-21 修订 −0.3 -> −1.0] 用户反馈「方向盘已经回正、前方没有车，也没有加速」。
#   回放定位：本条把「前车刚走、车已降到 5 km/h、MPC 想 +1.87」的帧挡住了 ——
#   此时 e2e 仍给 −0.78（模型比实车滞后）⇒ 不放闸 ⇒ 该起步时静止 4 秒。
#   敏感性：放宽到 −1.0 新增 12 帧开闸（58→70），**安全违规帧仍为 0**
#   （违规 = 雷达近车 / 模型前车概率≥0.5 / e2eStop / 欠速不足）。
E2E_GATE_MIN_A: float = -1.0          # e2e 低于此值 = 真实制动意图 ⇒ 不开闸
E2E_GATE_LEAD_M: float = 50.0         # 雷达前车距离门限（m）
E2E_GATE_LEAD_PROB: float = 0.5       # 模型自报前车概率门限（leadsV3[0].prob）
# [2026-09-21 同步 −0.3 -> −1.0] 同因：模型 plan 首点在「前车刚走」时会滞后为负
#   （实测该场景 plan_a0 仍在 −0.63~−1.24，而 aMpc 已经 +1.87）。
E2E_GATE_MIN_PLAN_A: float = -1.0     # 模型 plan 首点加速度门限（m/s²）

# ── 防抖：滞回 + 最短保持（2026-09-21 第 3 次修订，见文末修订记录）──
E2E_GATE_HYST_ENABLE: bool = True     # 一键回退：False = 回到第 2 次修订的纯阈值行为
E2E_GATE_MIN_HOLD_S: float = 2.0      # 开闸后最短保持（s）：软条件在此期间不能关闸
E2E_GATE_MIN_OFF_S: float = 1.0       # 关闸后最短冷却（s）：防止「关一帧就开」

# 「保持开闸」阈值：一律比进入阈值更宽松，判据在两条线之间摆动时不翻转
E2E_GATE_HYST_GAP_MS: float = 1.2     # 比进入 2.0 松（车已接近巡航速度也可保持）
E2E_GATE_HYST_A: float = -1.6         # 比进入 −1.0 松（更负 = 更宽松）
E2E_GATE_HYST_PLAN_A: float = -1.6    # 比进入 −1.0 松
E2E_GATE_HYST_LEAD_M: float = 40.0    # 比进入 50 松（前车 40~50 m 之间维持原状态）
E2E_GATE_HYST_LEAD_PROB: float = 0.6  # 比进入 0.5 松

# 硬闸：任何时刻立即关闸，**不受最短保持保护**（安全底线）
E2E_GATE_HARD_LEAD_M: float = 20.0    # 雷达前车近至此距离 -> 立刻关闸
E2E_GATE_HARD_LEAD_PROB: float = 0.8  # 模型高置信度前车 -> 立刻关闸
# 【重要】没有这两个，最短保持会在模型突然给出强烈制动意图时「兜住」2 秒不放行
# —— 那是不能接受的安全延迟。越过这两条线一律当帧关闸。
E2E_GATE_HARD_A: float = -2.0         # 模型要求 2.0 m/s² 以上制动 -> 立刻关闸
E2E_GATE_HARD_PLAN_A: float = -2.0    # 模型轨迹首点 2.0 m/s² 以上减速 -> 立刻关闸


def should_yield(
  *,
  enabled: bool,
  gap_ms: float,
  a_e2e: float,
  e2e_should_stop: bool,
  lead_present: bool,
  d_rel: float,
  model_lead_prob: float,
  plan_a0: float,
  min_gap_ms: float | None = None,
  min_a: float | None = None,
  min_plan_a: float | None = None,
  lead_m: float | None = None,
  lead_prob: float | None = None,
) -> bool:
  """e2e 候选是否应当「让位」（即不参与 min()）。**无状态纯函数**。

  返回 True 表示开闸：本次不把 e2e 加入候选集，速度跟踪权交给 cruise / MPC。
  返回 False 表示保持原行为（e2e 照旧与 MPC/cruise 一起取 min）。

  参数全部为**当帧原始量**，本函数不读 sm、不做平滑，便于单测与离线回放：
    enabled         : E2E_ACCEL_GATE_ENABLE（总开关）
    gap_ms          : v_cruise - v_ego（m/s，正值 = 欠速）
    a_e2e           : modelV2.action.desiredAcceleration
    e2e_should_stop : modelV2.action.shouldStop
    lead_present    : radarState.leadOne.present
    d_rel           : radarState.leadOne.dRel（无前车时可传任意值）
    model_lead_prob : modelV2.leadsV3[0].prob（无 leads 时传 -1.0）
    plan_a0         : modelV2.acceleration.x[0]（模型 plan 的首点加速度）

  可选阈值覆盖（None = 用模块级常量）。`E2eAccelGate` 用它在「进入/保持」两种
  阈值之间切换，实现滞回；单独调用时不传即等价于第 2 次修订的行为。

  安全论证见模块文件头。核心：开闸只可能发生在「欠速 + 空路 + 模型没在减速」，
  且只删掉 e2e **这一条**候选，min() 里仍保留 MPC（含前车约束）与 cruise。
  """
  if not enabled:
    return False
  _min_gap = E2E_GATE_MIN_GAP_MS if min_gap_ms is None else min_gap_ms
  _min_a = E2E_GATE_MIN_A if min_a is None else min_a
  _min_plan = E2E_GATE_MIN_PLAN_A if min_plan_a is None else min_plan_a
  _lead_m = E2E_GATE_LEAD_M if lead_m is None else lead_m
  _lead_prob = E2E_GATE_LEAD_PROB if lead_prob is None else lead_prob

  # ① 必须确实该加速（否则正常巡航/跟车/减速场景一概不动）
  if gap_ms < _min_gap:
    return False
  # ② e2e 有真实制动意图 -> 尊重它
  if a_e2e < _min_a:
    return False
  # ③ e2e 说要停 -> 绝不放行
  if e2e_should_stop:
    return False
  # ④ 雷达前车近 -> 不开闸
  if lead_present and d_rel <= _lead_m:
    return False
  # ⑤ 模型自己看到前车 -> 不开闸（雷达看不到、模型看到的情况）
  if model_lead_prob >= _lead_prob:
    return False
  # ⑥ 模型轨迹在减速 -> 尊重它
  if plan_a0 < _min_plan:
    return False
  return True


class E2eAccelGate:
  """带**滞回**与**最短保持时间**的门控状态机（2026-09-21 第 3 次修订）。

  为什么必须是有状态的：纯逐帧阈值判定下，`a_e2e` / `plan_a0` / `rLead` 三个
  布尔判据在各自分布最密集处反复穿越（实测翻转 108 / 146 / 42 次），
  候选集里 e2e 一帧进一帧出，`aTgt` 在 +1.6 与 −0.6 之间阶跃（最大 3.016 m/s²），
  用户体感「突然加速又突然停顿，像有东西在打架」。详见模块文件头第 3 次修订。

  用法（planner 侧）：
      # __init__ 里建一次
      self.e2e_gate = e2e_accel_gate.E2eAccelGate()
      # 每帧
      self.e2e_yield = self.e2e_gate.update(
        t=time.monotonic(), enabled=..., gap_ms=..., a_e2e=..., e2e_should_stop=...,
        lead_present=..., d_rel=..., model_lead_prob=..., plan_a0=...)
      if not self.e2e_yield:
        candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, ...))

  `reason` 属性给出本帧判定结果来源，便于探针诊断：
      off / hardStop / hardLead / hardProb / hardBrake / hardPlanBrake / overspeed
                    —— 关闸（硬闸，当帧生效）
      enter         —— 本帧由关转开
      keep          —— 保持开闸（软条件仍成立）
      hold          —— 保持开闸（靠最短保持兜住，软条件已不成立）
      release       —— 软条件不成立且保持期已过，关闸
      blocked       —— 关闸态且未满足进入条件
      cooldown      —— 关闸后最短冷却期内
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._gate: bool = False
    self._t_enter: float = -1e18
    self._t_exit: float = -1e18
    self.reason: str = 'init'

  @property
  def yielding(self) -> bool:
    return self._gate

  def _close(self, t: float, reason: str) -> None:
    if self._gate:
      self._t_exit = t
    self._gate = False
    self.reason = reason

  def update(
    self,
    *,
    t: float,
    enabled: bool,
    gap_ms: float,
    a_e2e: float,
    e2e_should_stop: bool,
    lead_present: bool,
    d_rel: float,
    model_lead_prob: float,
    plan_a0: float,
  ) -> bool:
    if not enabled:
      self._close(t, 'off')
      return False

    # ── 硬闸：立即关闸，不受最短保持保护（安全底线）──
    if e2e_should_stop:
      self._close(t, 'hardStop')
      return False
    if lead_present and d_rel <= E2E_GATE_HARD_LEAD_M:
      self._close(t, 'hardLead')
      return False
    if model_lead_prob >= E2E_GATE_HARD_LEAD_PROB:
      self._close(t, 'hardProb')
      return False
    if a_e2e < E2E_GATE_HARD_A:
      self._close(t, 'hardBrake')
      return False
    if plan_a0 < E2E_GATE_HARD_PLAN_A:
      self._close(t, 'hardPlanBrake')
      return False
    if gap_ms < 0.0:
      self._close(t, 'overspeed')
      return False

    if self._gate:
      # ── 保持态：用放宽的「退出」阈值 ──
      # 注意 HYST_ENABLE=False 时必须**连阈值放宽一起关掉**，否则「一键回退」只关了
      # 最短保持与冷却，滞回仍在生效 —— 那就不等于第 2 次修订的纯阈值行为。
      # （此缺陷由 test_hyst_disable_restores_pure_behaviour 抓到。）
      hyst_kw = (dict(min_gap_ms=E2E_GATE_HYST_GAP_MS, min_a=E2E_GATE_HYST_A,
                      min_plan_a=E2E_GATE_HYST_PLAN_A, lead_m=E2E_GATE_HYST_LEAD_M,
                      lead_prob=E2E_GATE_HYST_LEAD_PROB)
                 if E2E_GATE_HYST_ENABLE else {})
      keep = should_yield(
        enabled=True, gap_ms=gap_ms, a_e2e=a_e2e, e2e_should_stop=False,
        lead_present=lead_present, d_rel=d_rel, model_lead_prob=model_lead_prob,
        plan_a0=plan_a0, **hyst_kw)
      if not keep:
        if E2E_GATE_HYST_ENABLE and (t - self._t_enter) < E2E_GATE_MIN_HOLD_S:
          self.reason = 'hold'      # 软条件已不成立，但最短保持兜住
          return True
        self._close(t, 'release')
        return False
      self.reason = 'keep'
      return True

    # ── 关闸态：需要满足「进入」严格阈值，且过了最短冷却 ──
    if E2E_GATE_HYST_ENABLE and (t - self._t_exit) < E2E_GATE_MIN_OFF_S:
      self.reason = 'cooldown'
      return False
    enter = should_yield(
      enabled=True, gap_ms=gap_ms, a_e2e=a_e2e, e2e_should_stop=False,
      lead_present=lead_present, d_rel=d_rel, model_lead_prob=model_lead_prob,
      plan_a0=plan_a0)
    if enter:
      self._gate = True
      self._t_enter = t
      self.reason = 'enter'
    else:
      self.reason = 'blocked'
    return self._gate
