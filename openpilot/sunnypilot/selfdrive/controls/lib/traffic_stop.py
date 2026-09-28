#!/usr/bin/env python3
"""
红灯 / 停车标志「虚拟停止线」辅助（下称红灯辅助）。

来源：移植自 DP（dragonpilot，`openpilot-dptest`）分支的 traffic_stop 机制
（`dragonpilot/selfdrive/controls/lib/traffic_stop.py`，对照
`PORTING_NOTE_dp_traffic_stop.md`）。那套又是从 carrot(cp) fork 移植的，
纯函数层逐字对照 cp 官方测试数值。

机制（**没有**独立的红绿灯识别模型）：
  把「驾驶模型自己预测的轨迹」当作它对前方是否要停的证据 ——
  模型预测轨迹的终点 x 很近、且末端速度很低 ⇒ 认为模型看到了红灯 / 停止标志。
  然后造一条**虚拟停止线**（虚拟障碍物）喂给纵向 MPC，让 MPC 像对待一台
  停在那里的车一样，一次求解出完整的 jerk-limited 减速曲线去逼近它。
  ⇒「柔和停车」的来源是 MPC 整段轨迹求解，而不是每帧重算软限速去追。

两条输出（冗余设计，与 DP 一致）：
  1. `stop_dist_m` → 主 planner 传给
     `LongitudinalMpc.update(traffic_stop_obstacle_m=...)`，作为
     `x_obstacles` 的第 3 栏（与 lead0 / lead1 并列）⇒ 影响 aMpc。
  2. `output_v_target` → SP `update_targets()` 的 targets 字典 ⇒ 压低 v_cruise
     ⇒ 影响 `get_cruise_accel()` 产出的 a_cruise。

★「实验模式下也有用」的依据（用户 2026-09-26 明确要求）：
  上面两条输出**都不需要**干预 e2e 候选，靠主 planner 的
  `min(candidates, key=a_target)` 自动生效：
    * 第 1 条让 aMpc 更保守（MPC 轨迹里已经含停车段）；
    * 第 2 条让 `a_cruise = clip(v_cruise − v_ego, −1.2, …)` 变成负值。
  实验模式只是往 candidates 里**多加了** aE2e 一项，而 min() 取更小者
  ⇒ 只有当模型自己刹得比我们更狠时才轮到它 ⇒ 对 e2e 天然有效。
  且 min() 只会取更小的值 ⇒ 本模块**从不削弱** e2e 更保守的判断，
  不可能制造"本模块要求减速、却被 e2e 放行"的缺口。
  ⛔ 因此本模块**绝不**把 e2e 排除出候选池 —— 那是 2026-09-24 被实车
  否决过的动作（详见 MEMORY.md §0 铁律），这里一个字都不碰。

与既有柔性模块的边界：
  * 只在本车**没有前车**（`lead_present == False`）时**进入**停等。
    已经有前车时，"跟车"本身就会停在该停的位置，红灯辅助应让位。
  * 进入之后，若前车距离与虚拟停止点的差距 < `CANCEL_LEAD_MARGIN_M`
    ⇒ 取消停等（cp 原设计；阈值 2.0 → 4.0，避免最终跟车距离卡在 2 m）。
  * 驾驶员踩油门 ⇒ 立刻退出停等，并武装 10 s 抑制计时器
    （与 gas_override / creep_step 的"驾驶员优先"一致）。

⛔ 本模块不做的事：不改候选池组成、不动 e2e 让位、不改 MPC 求解约束。

── 与 DP 原版的差异（有意为之，改前务必读）──────────────────────────────
  ① **偏移值方向**（2026-09-26 用户要求；2026-09-27 实车定案）：
     用户原话「正值是停止位置往前移，负值是停止位置往后移」。
     DP 原版是 `stop_dist + CAMERA_TO_FRONT + adjust` —— **它已经就是用户要的
     语义**：`stop_dist` 是喂给 MPC 的**障碍物距离**（`long_mpc.py` 里
     `params[:,2] = min(x_obstacles)`），距离越大 ⇒ 障碍物越远 ⇒ 允许走得越远
     ⇒ 车**停得更靠前**；模块自己的 `v_limited = sqrt(2·a·(d−1))` 同向
     （d 大 ⇒ 限速高 ⇒ 晚减速）。
     ⚠️ 2026-09-27 之前的注释把这条判反了（误以为「加正值 ⇒ 停得更靠后」），
     于是本实现刻意写成**减** —— 结果把用户要的方向**整体翻反**。实车证据：
       * 用户 02:05 停车等灯、车静止时逐档改参：`adj 0 → +3.0` 对应
         `d 3.4 → 0.4` ⇒ **Δd = −Δadj**，即代码确实在做 `d = raw − 1.5 − adj`；
       * 停稳落点：`adj=-0.5` 时 `d ≈ 0.0~0.6`（≈车头压线/微过线），
         而 `adj=-2.0` 时是 `0.7~1.4` ⇒ 用户报的「红灯还是容易超过停止线」
         正是本常量造成的。
     现取 `ADJUST_POSITIVE_MOVES_STOP_FORWARD = False`，方向与 DP 原版、
     与 UI 文案三者一致。
  ② 参数走 sunnypilot 的 Params（`TrafficStopAssist` /
     `TrafficStopDistanceAdjust`，单位**分米**），不再是 dp 的
     `dp_lon_traffic_stop*`。
  ③ 阈值/增益**不改**：动过的那几个数（CANCEL_LEAD_MARGIN_M 2.0→4.0）保留
     DP 的结论，并保留了它注释里"上一轮改错又改回"的教训。
  ④ 增加 `[TrafficStop]` 探针（跳变必打 + 激活期 1 Hz）。本链路**没有**
     每帧日志，实车验收只能靠探针，所以这里必须留痕。
  ⑤ 2026-09-27 补探针（纯留痕，**不参与任何控制判断**）。起因：用户报
     「左转车道里左转红灯、直行绿灯时不容易刹停」，而原探针只打
     `sig/d/adj/vEgo/aOut/vTgt` ⇒ **三道嫌疑在日志里长得一模一样**，
     无法区分到底是哪条挡住的：
       * 模型压根没判红灯（`sig != red`）；
       * 判了，但因为**前方有车**（`lead_present`）让位给跟车；
       * 进了停等、但被 `MPC_OBSTACLE_ONLY_WHEN_MUST_BRAKE` 门槛
         压到最后一刻才给 MPC 障碍物（"介入太晚"）。
     现在补：`lead/dRel/blink/steer/xEnd/yEnd/mv/mv0`（原始观测量）、
     `c=子条件掩码`、`block=没进停等是被哪几条挡住的`、
     `d=None(raw=…)`（被门槛挡住时的原始停止距离）、`vLim`（离介入还差多少）。
     另把「CRUISE 且模型判红灯」也纳入 1 Hz 打印 —— 这正是"该停没停"的
     现场，原实现只在**状态跳变**时留痕，整段只有第一帧有日志。
"""

from collections import deque

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

# ── 参数（Params，声明见 common/params_keys.h）──────────────────────────────
PARAM_ENABLED = "TrafficStopAssist"                  # BOOL，默认 "0"（关）
PARAM_DISTANCE_ADJUST = "TrafficStopDistanceAdjust"  # INT，单位**分米**，默认 "0"
PARAM_POLL_FRAMES = round(1.0 / DT_MDL)              # 1 s 轮询一次（热生效）

DM_PER_M = 10.0        # Params 存分米 ⇒ 米 = dm / 10
ADJUST_LIMIT_M = 5.0   # 用户微调范围（±5 m），与 UI 的 min/max 对应

# ★ 方向约定（2026-09-26 用户要求；2026-09-27 实车定案 —— 勿再翻，依据见文件头 ①）：
#   False ⇒ 正值 = 停止位置**往前移**（车头更靠前、离停止线更近、更易过线）；
#            负值 = 停止位置**往后移**（更早停、更安全）      ← 当前取值，与 UI 文案一致
#   True  ⇒ 与上面完全相反（2026-09-27 之前的取值；当时 UI 文案是反的，
#            用户按文案把值调到 -5，反而被往前推了 0.5 m ⇒「还是过线」）
#   结论：改这一个布尔值即可，**不要去改公式**。
ADJUST_POSITIVE_MOVES_STOP_FORWARD = False

# ── 给 MPC 的虚拟停止线（第二路）门槛 —— 2026-09-27 用户拍板 ────────────────
#   识别到虚拟停止线后有**两条**路会让车减速：
#     ①v_target 进候选池（上面那个修法管的）
#     ②stop_dist_m 交给 MPC 当第 3 栏障碍物（`mpc.update(traffic_stop_obstacle_m=)`
#       ⇒ MPC 按障碍物规划 jerk-limited 减速）← **这才是真正让车减速的那一路**
#   2026-09-27 之前 ②无条件生效 ⇒ 模型只要误预测"前方会收速"，车就真的减速
#   （用户报的"半路看见红旗也减速"）。
#   True  ⇒ ②也只在"几何上刹不住"（v_limited < v_ego）时才给 ⇒ 彻底消除半路误减速；
#           代价：远处红灯要等到真刹不住的距离（约 45 m @50 km/h）才开始减速。
#   False ⇒ 退回旧行为（识别到就提前减速，含模型误预测）。
MPC_OBSTACLE_ONLY_WHEN_MUST_BRAKE = True

# ── 状态机 ─────────────────────────────────────────────────────────────────
CRUISE, STOPPING, STOPPED = 0, 1, 2
OFF, RED, GREEN = 0, 1, 2

# 未激活时对外发的 v_target 哨兵：故意取一个**远大于任何真实巡航速度**的值，
# 让 SP 的 `min(targets, key=v_target)` 永远选不中本模块
# ⇒ 未激活 / 关闭 = 对纵向链路严格零影响。
V_TARGET_SENTINEL = 1e6

# ── 常量（对照 cp 原始码，DP 已逐项核过；此处仅改名与加注）──────────────────
TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0

TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH = (0.0, 100.0)
TRAFFIC_STOP_DISTANCE_RATIO = (1.0, 0.7)
TRAFFIC_STOP_DISTANCE_FADE_BP_M = (0.0, 50.0)

COMFORT_BRAKE_BASE = 2.4          # m/s² —— 是 2.4 不是 2.5（cp 表 8.1）
STOPPING_BRAKE_DISCOUNT = 0.9     # 只在 STOPPING 用，STOPPED 不用（8.2）

STOPPED_SPEED_MS = 0.3            # 单帧触发，无防抖（8.6）
STOPPED_GRACE_FRAMES = round(0.5 / DT_MDL)   # STOPPED 后 0.5 s 绿灯冷却
GREEN_CONFIRM_FRAMES = round(0.2 / DT_MDL)   # 红灯无防抖，绿灯需 4 帧 @20Hz
GAS_SUPPRESS_FRAMES = round(10.0 / DT_MDL)   # 踩油门后 10 s 不再重识别

RECALIBRATE_MIN_DISTANCE_M = 10.0  # 只有候选距离 > 10 m 才重新标定 dead-reckoning

# 前车「取消」判断阈值。cp 原始码固定 2.0，且刻意与「进入」判断（有前车就挡）
# 用不同条件 —— 这是 cp 的既有设计。上一轮曾误改成「只要有任何前车就无条件取消」，
# 那会引入新副作用：正在进行的停等，会被雷达范围内任何跟停止线无关的车
# （隔壁车道 / 路径外）整个取消掉。现维持 cp 原有的"距离阈值"架构，只把阈值
# 由 2.0 调大到 4.0，让最终跟车距离不要卡在 2 m 附近。
# ⚠ 最终跟车距离还会叠加 long_mpc.py 的 get_safe_obstacle_distance() /
#   LEAD_DANGER_FACTOR 这层与障碍物来源无关的舒适距离软惩罚，确切数字要实车看。
CANCEL_LEAD_MARGIN_M = 4.0

MEDIAN_WINDOW = 3
MOVING_AVG_WINDOW = 15
MODEL_V_WINDOW = 10

RATE_LIMIT_CLOSING_MARGIN_M = 0.5  # 只限制"逼近"速度；后退不限

STOP_SIGN_MAX_SPEED_KPH = 82.0
STOP_SIGN_DETECT_DIST_BP_KPH = (60.0, 80.0)
STOP_SIGN_DETECT_DIST_M = (120.0, 150.0)
STOP_SIGN_LATERAL_TOLERANCE_M = 5.0

# cp 默认把「相机→车头」的固定物理修正与用户可调值混在一个没有标注的参数里。
# 按 DP 的拆法分两层：这里是**不给用户改**的固定物理修正，用户微调走
# TrafficStopDistanceAdjust，两者在 get_traffic_stop_obstacle_distance() 里叠加。
# ⚠ 该 -1.5 m 是照 cp 默认值移植的假设值，**没有**针对本车安装独立标定过
#   ⇒ 首次实车务必核对「车头与停止线的实际距离」，必要时调这里。
CAMERA_TO_FRONT_M = -1.5

TRAFFIC_STOP_LOG_HZ = 1.0

# ★ 诊断开关（2026-09-27 加）：CRUISE 态是否也按 1 Hz 留痕。
#   True  ⇒ **行车中连续留痕**。复盘"该停没停"必须有这个：模型没判红灯的
#           时刻，模块一直停在 CRUISE（不跳变）⇒ 原实现整段只有第一帧日志，
#           等于事后完全没有证据。这是本次排查"左转车道不停车"的关键前提。
#   False ⇒ 恢复 DP 原意（只在状态跳变 / 激活期留痕）的安静模式。
#   问题查清后改回 False 即可，其它地方一行都不用动。
#   量级：~200 B/帧 × 1 Hz ≈ 12 KB/分钟，对 swaglog（分钟一文件）可忽略。
LOG_CRUISE_HEARTBEAT = True


# ── Param 读取容错（**不要删**，理由见下）──────────────────────────────────
# 本控制器的构造发生在 plannerd 的**启动路径**上。这两个 Param 由
# params_keys.h 声明，必须重编译 libparams_c.so 才真正存在；若在哪一次部署里
# 「文件推上去了、.so 还没编」，未声明的 key 会让 openpilot 的 Params 抛
# `UnknownKeyName` ⇒ **plannerd 起不来**（车直接不能开）。
# 所以这里一律做降级读取：读不到就退回默认值（= 功能关闭），绝不让异常逃逸。
def _read_bool(params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key, default))
  except Exception:  # noqa: BLE001 - 见上方说明，这里是刻意的兜底
    return default


def _read_raw(params, key: str, default=None):  # noqa: ANN001
  try:
    value = params.get(key, return_default=True)
    return default if value is None else value
  except Exception:  # noqa: BLE001
    return default


# ── 纯函数（数值与 cp 官方测试逐项对齐，可独立测试）─────────────────────────

def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """方向盘角度大 ⇒ 这是在转弯，不是在接近停止点。只挡**新进入** STOPPING，
  不影响已经在进行的停等。"""
  return abs(steering_angle_deg) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_reference_speed(v_ego_kph: float, previous_reference_kph: float | None) -> float:
  """锁存本次停等事件里见过的最高 v_ego。单调不减。"""
  return max(0.0, v_ego_kph, previous_reference_kph or 0.0)


def get_virtual_traffic_stop_distance(model_distance: float, v_ego_kph: float) -> float:
  """接近速度越快，开始刹车的比例越提前（0kph 时 100%，100kph 时 70%）；
  但在最后 50 m 内比例淡回 100%，保证无论接近速度多少，车都停在正确位置。"""
  distance_ratio = np.interp(v_ego_kph, TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH, TRAFFIC_STOP_DISTANCE_RATIO)
  applied_ratio = np.interp(model_distance, TRAFFIC_STOP_DISTANCE_FADE_BP_M, [1.0, distance_ratio])
  return max(0.0, model_distance * applied_ratio)


def get_traffic_stop_obstacle_distance(stop_distance: float, distance_adjust: float) -> float:
  """施加（固定物理 + 用户）距离修正。**每帧只允许调用一次** ——
  cp bug 8.3 就是这里被调用两次或干脆没调。"""
  return max(0.0, stop_distance + distance_adjust)


def get_stop_sign_breakdown(model_x_end: float, model_y_end: float, model_v: float,
                            model_v_start: float, v_ego_kph: float,
                            d_rel: float) -> tuple[bool, str]:
  """把「判红灯」的每个子条件摊开（**诊断用**）。返回 (stop_sign, mask)。

  mask 是逐位布尔串（`1` = 该条通过，`-` = 该分支不适用）：
    停稳分支（v < 1 kph）→ `c1c2--`
      c1 模型预测末端点 < 20 m；c2 末端速度 < 10 m/s
    行车分支（1 ≤ v < 82 kph）→ `c1c2c3c4`
      c1 末端点比前车更近 3 m（无前车时 d_rel 已被换成 1000 ⇒ 恒真）
      c2 末端点在探测距离包线内（随速度 120→150 m）
      c3 末端速度低（< 3 m/s 或 < 起始速度的 70%）
      c4 末端点**横向在本车道内**（|y| < 5 m）← 左转车道的主要嫌疑
    高速分支（v ≥ 82 kph）→ `----`（该分支恒不判停）

  ★ **唯一公式来源**：`check_model_stopping` 也调本函数 ⇒ 探针与判据永远同源，
    不会出现"日志显示全通过、判据却不判停"的漂移。
  """
  if v_ego_kph < 1.0:
    c1 = model_x_end < 20.0
    c2 = model_v < 10.0
    return (c1 and c2), f"{int(c1)}{int(c2)}--"
  if v_ego_kph >= STOP_SIGN_MAX_SPEED_KPH:
    return False, '----'

  max_detect_dist = np.interp(model_v_start * 3.6, STOP_SIGN_DETECT_DIST_BP_KPH, STOP_SIGN_DETECT_DIST_M)
  c1 = model_x_end < d_rel - 3.0
  c2 = model_x_end < max_detect_dist
  c3 = model_v < 3.0 or model_v < model_v_start * 0.7
  c4 = abs(model_y_end) < STOP_SIGN_LATERAL_TOLERANCE_M
  return (c1 and c2 and c3 and c4), f"{int(c1)}{int(c2)}{int(c3)}{int(c4)}"


def check_model_stopping(hist: deque, stop_sign_count: int, start_sign_count: int, state: int,
                         v_cruise: float, model_v_traj, v_ego: float, a_ego: float,
                         model_x_end: float, model_y_end: float, d_rel: float) -> tuple[int, int, int]:
  """返回 (traffic_state, 新 stop_sign_count, 新 start_sign_count)。"""
  v_ego_kph = v_ego * 3.6

  hist.append(model_v_traj[-1])
  model_v = sum(hist) / len(hist)

  # 模型预测自己在加速 ⇒ 这是"起步"信号（绿灯 / 前车走了）
  start_sign = model_v > 5.0 or model_v > (model_v_traj[0] + 2)

  # 子条件判定统一走 get_stop_sign_breakdown（两分支都已收进去）。
  # ★ 已停稳分支（v < 1 kph）只看模型预测末端是否近且慢 —— 这是**唯一能成立的
  #   入口**（高速红灯的模型输入本身不可用，实测 >45kph 时 stopPt 缺失率 91.7%）。
  stop_sign, _ = get_stop_sign_breakdown(model_x_end, model_y_end, model_v,
                                         float(model_v_traj[0]), v_ego_kph, d_rel)

  # ★ cp 原设计的附加抑制，**只在行车分支生效** —— 别把它挪到分支外面：
  #   停稳分支没有这条，挪出去会让"已停稳但 v_cruise≠0 且在减速"时不再判停，
  #   属于行为改变。探针里 `c=1111` 却 `sig≠red` 就是被这条吃掉的。
  if 1.0 <= v_ego_kph < STOP_SIGN_MAX_SPEED_KPH:
    if v_cruise != 0 and state == CRUISE and a_ego < -1.0:
      stop_sign = False

  stop_sign_count = stop_sign_count + 1 if stop_sign else 0
  start_sign_count = start_sign_count + 1 if (start_sign and not stop_sign) else 0

  if stop_sign_count * DT_MDL > 0.0:
    return RED, stop_sign_count, start_sign_count          # 红灯：单帧即确认，无防抖
  if start_sign_count * DT_MDL > 0.2:
    return GREEN, stop_sign_count, start_sign_count        # 绿灯：需 0.2 s 确认
  return OFF, stop_sign_count, start_sign_count


class TrafficStopController:
  """每帧调用一次 update()（20 Hz）。结果读 `stop_dist_m` / `output_v_target` /
  `output_a_target`；探针字符串在 `log`（由 planner 打，cloudlog 最低只落 INFO）。"""

  def __init__(self) -> None:
    self.params = Params()
    self.is_enabled = _read_bool(self.params, PARAM_ENABLED)
    self.distance_adjust_m = self._read_adjust_m()
    self._poll_frame = 0

    self.state = CRUISE
    self.traffic_state = OFF
    self.stop_sign_count = 0
    self.start_sign_count = 0
    self.model_v_hist: deque = deque(maxlen=MODEL_V_WINDOW)

    # median(3) → moving-average(15)，**跨停等事件永不清空**（8.9）
    self._median_hist: deque = deque(maxlen=MEDIAN_WINDOW)
    self._avg_hist: deque = deque(maxlen=MOVING_AVG_WINDOW)
    self.stop_model_x_raw = 0.0
    self.stop_model_x_rl = 0.0

    self.reference_speed_kph = 0.0
    self.actual_stop_distance = 0.0
    self.gas_suppress_frames = 0
    self.stopped_grace_frames = 0

    # ── 对外输出 ──
    self.stop_dist_m: float | None = None
    self.output_v_target = V_TARGET_SENTINEL
    self.output_a_target = 0.0
    self.log = None
    self._log_t = 0.0
    self._last_tag = 'off'

    # ── 诊断字段（2026-09-27 加；**只给探针用，不参与任何控制判断**）──────────
    #   为什么进不去停等 / 进去得多晚，全靠这几个数才能事后复盘。
    self._log_blk_t = 0.0          # 「CRUISE & sig=red」这条日志的独立节流
    self._dbg_lead = False
    self._dbg_d_rel = 0.0
    self._dbg_steer = 0.0
    self._dbg_x_end = 0.0
    self._dbg_y_end = 0.0
    self._dbg_model_v = 0.0
    self._dbg_v_start = 0.0
    self._dbg_mask = '----'
    self._dbg_block = 'none'
    self._dbg_blinker = '-'
    # ★ 2026-09-27 加：**独立的**右灯诊断位。刻意不复用 `_dbg_blinker`——
    #   后者在 `_heartbeat_on()` 里被当行为判断用（`== 'L'`），改成 'LR'/'R'
    #   会让「双闪 / 同时打灯」时的心跳判定翻转。新增字段 ⇒ 零行为改动。
    self._dbg_blink_r = False
    self._dbg_stop_raw = None      # 被门槛挡住（stop_dist_m=None）时的原始停止距离
    self._dbg_v_limited = float('inf')   # 门槛判据用的限速值（与 vEgo 比大小）

  # ── 参数轮询（1 Hz 热生效）────────────────────────────────────────────────
  def _read_adjust_m(self) -> float:
    raw = _read_raw(self.params, PARAM_DISTANCE_ADJUST, 0)
    try:
      dm = float(raw)
    except (TypeError, ValueError):
      dm = 0.0
    return float(np.clip(dm / DM_PER_M, -ADJUST_LIMIT_M, ADJUST_LIMIT_M))

  def _poll_params(self) -> None:
    self._poll_frame += 1
    if self._poll_frame >= PARAM_POLL_FRAMES:
      self._poll_frame = 0
      self.is_enabled = _read_bool(self.params, PARAM_ENABLED)
      self.distance_adjust_m = self._read_adjust_m()

  # ── 复位 ──────────────────────────────────────────────────────────────────
  def reset(self) -> None:
    self.state = CRUISE
    self.traffic_state = OFF
    self.stop_sign_count = 0
    self.start_sign_count = 0
    # ⚠ 滤波器（_median_hist/_avg_hist/model_v_hist）**故意不复位**（8.9）
    self.actual_stop_distance = 0.0
    self.reference_speed_kph = 0.0
    self.stopped_grace_frames = 0
    self.stop_dist_m = None
    self.output_v_target = V_TARGET_SENTINEL
    self.output_a_target = 0.0
    # ⚠ 故意**不**重置 _last_tag：让"激活 → off"的跳变仍能被 _emit_tag 捕获，
    #   否则关闭/复位那一刻的留痕会丢。

  # ── 内部：模型停止点滤波 + 逼近限速 ────────────────────────────────────────
  def _update_stop_model_x(self, raw_x: float, v_ego: float) -> tuple[float, float]:
    self._median_hist.append(raw_x)
    median_val = float(np.median(self._median_hist))
    self._avg_hist.append(median_val)
    stop_model_x_raw = float(np.mean(self._avg_hist))

    # 停止点只允许以「本车最快可能逼近的速度」靠近；后退不限速。
    max_step = v_ego * DT_MDL + RATE_LIMIT_CLOSING_MARGIN_M
    if stop_model_x_raw < self.stop_model_x_rl:
      stop_model_x_rl = max(stop_model_x_raw, self.stop_model_x_rl - max_step)
    else:
      stop_model_x_rl = stop_model_x_raw  # 后退不限
    return stop_model_x_raw, stop_model_x_rl

  # ── 主更新 ────────────────────────────────────────────────────────────────
  def update(self, *,
             model_x_traj,
             model_y_traj,
             model_v_traj,
             steering_angle_deg: float,
             gas_pressed: bool,
             left_blinker: bool,
             lead_present: bool,
             d_rel: float,
             v_ego: float,
             a_ego: float,
             v_cruise: float,
             dt: float = DT_MDL,
             # [2026-09-27] 仅供诊断探针区分左右灯。**不参与任何控制判断**
             # （原有 left_blinker 语义 0 改动；本字段只影响日志的 blinkR= 字段）。
             # 默认 False ⇒ 旧调用方（含 ts_patch/sim_*.py）行为逐位不变。
             right_blinker: bool = False) -> None:
    """每帧一次。参数由 planner 从 sm 里取出后显式传入（便于离线仿真）。

    Args:
      model_x_traj: modelV2.position.x（模型预测轨迹的纵向位置）
      model_y_traj: modelV2.position.y（横向位置，用于判断停止点是否在本车道）
      model_v_traj: modelV2.velocity.x（模型预测速度）
      steering_angle_deg: carState.steeringAngleDeg（已去 angleOffset）
      gas_pressed: carState.gasPressed
      left_blinker: carState.leftBlinker
      right_blinker: carState.rightBlinker（**2026-09-27 加，仅诊断探针用**）
      lead_present: radarState.leadOne.present
      d_rel: radarState.leadOne.dRel (m)
      v_ego: 本车车速 (m/s)
      a_ego: 本车加速度 (m/s²)
      v_cruise: 当前巡航设定速度 (m/s，已含 forceDecel 归零)
      dt: 控制周期 (s)
    """
    self.log = None
    self._poll_params()

    if not self.is_enabled:
      self.reset()
      self._emit_tag(v_ego, dt)
      return

    if len(model_x_traj) < 2 or len(model_v_traj) == 0:
      # 模型数据不完整：本帧不给任何约束（保守地"当作没激活"）
      self.stop_dist_m = None
      self.output_v_target = V_TARGET_SENTINEL
      self.output_a_target = 0.0
      return

    model_x_end = float(model_x_traj[-1])
    model_y_end = float(model_y_traj[-1])

    # ★★ 无前车时 `leadOne.dRel` 是无意义的 0。`check_model_stopping` 里有一条
    #    cp 原设计判据 `model_x_end < d_rel - 3.0`（模型预测的停止点必须比雷达最远
    #    探测到的车更近）。d_rel=0 会让它**恒为假** ⇒ 车速 >1 km/h 时**永远进不了
    #    停等** ⇒ 整个模块静默失效。而本模块的核心场景恰恰是「**没有**前车」
    #    （有前车就让位给跟车了，见文件头边界）。
    #    ⇒ 必须照 DP 的写法，把无前车时的 d_rel 换成一个大数（DP 取 1000.0）。
    #    ⚠ 别把这一行"顺手删掉" —— 删了功能就彻底不工作，而且**不会有任何报错**，
    #      表现为"开关打开也没用"，极难排查。
    d_rel_eff = d_rel if lead_present else 1000.0

    self.stop_model_x_raw, self.stop_model_x_rl = self._update_stop_model_x(float(model_x_traj[-2]), v_ego)

    self.traffic_state, self.stop_sign_count, self.start_sign_count = check_model_stopping(
      self.model_v_hist, self.stop_sign_count, self.start_sign_count, self.state,
      v_cruise, model_v_traj, v_ego, a_ego, model_x_end, model_y_end, d_rel_eff)

    # ── 诊断留痕（纯记录，见文件头 ⑤）────────────────────────────────────────
    #   ⚠ 以下字段**不参与任何判断** —— 即使写错也只影响日志，不影响驾驶。
    #   `model_v` 的复算与 check_model_stopping 同源：它刚刚 append 过本帧值，
    #   所以这里 sum/len 拿到的就是同一个数。
    self._dbg_lead = bool(lead_present)
    self._dbg_d_rel = float(d_rel)
    self._dbg_steer = float(steering_angle_deg)
    self._dbg_x_end = model_x_end
    self._dbg_y_end = model_y_end
    self._dbg_v_start = float(model_v_traj[0])
    self._dbg_model_v = (float(sum(self.model_v_hist) / len(self.model_v_hist))
                         if len(self.model_v_hist) > 0 else 0.0)
    self._dbg_blinker = 'L' if left_blinker else '-'
    self._dbg_blink_r = bool(right_blinker)
    self._dbg_mask = get_stop_sign_breakdown(model_x_end, model_y_end, self._dbg_model_v,
                                             self._dbg_v_start, v_ego * 3.6, d_rel_eff)[1]
    if self.traffic_state != RED and '0' not in self._dbg_mask and '-' not in self._dbg_mask:
      # 子条件**全通过**却没判红灯 ⇒ 被行车分支那条「v_cruise≠0 且在减速」的
      # 附加抑制（cp 原设计）吃掉了。加个 `!` 让日志一眼可辨。
      self._dbg_mask += '!'
    self._dbg_block = (self._entry_block(lead_present, steering_angle_deg)
                       if self.state == CRUISE else '--')

    # 停等中踩油门 ⇒ 武装 10 s 抑制（8.11）
    if gas_pressed and self.state == STOPPING:
      self.gas_suppress_frames = GAS_SUPPRESS_FRAMES
    elif self.gas_suppress_frames > 0:
      self.gas_suppress_frames -= 1

    lead_cancels = bool(lead_present and (d_rel - self.stop_model_x_raw) < CANCEL_LEAD_MARGIN_M)

    # ── 状态机 ──
    if self.state == CRUISE:
      entry_allowed = is_traffic_stop_entry_allowed(steering_angle_deg)
      if (not lead_present and self.traffic_state == RED and
          entry_allowed and self.gas_suppress_frames == 0):
        self.state = STOPPING
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, None)
        self.actual_stop_distance = get_virtual_traffic_stop_distance(
          self.stop_model_x_rl, self.reference_speed_kph)

    elif self.state == STOPPING:
      if gas_pressed:
        self.state = CRUISE
      elif lead_cancels:
        self.state = CRUISE
      elif self.traffic_state == GREEN:
        self.state = CRUISE
      else:
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, self.reference_speed_kph)
        candidate = get_virtual_traffic_stop_distance(self.stop_model_x_rl, self.reference_speed_kph)
        if candidate > RECALIBRATE_MIN_DISTANCE_M:
          self.actual_stop_distance = candidate
        if v_ego < STOPPED_SPEED_MS:
          self.state = STOPPED
          self.stopped_grace_frames = STOPPED_GRACE_FRAMES

    elif self.state == STOPPED:
      if gas_pressed:
        self.state = CRUISE
      elif lead_cancels:
        self.state = CRUISE
      else:
        if self.stopped_grace_frames == 0:
          if self.traffic_state == GREEN and not left_blinker:
            self.state = CRUISE
        self.stopped_grace_frames = max(0, self.stopped_grace_frames - 1)

    # ── 障碍物释放与状态机切换是**两条独立路径**，不能共用同一处 reset（8.7）──
    if self.state == CRUISE:
      self.actual_stop_distance = 0.0
      self.reference_speed_kph = 0.0
      self.stopped_grace_frames = 0
      self.stop_dist_m = None
      self.output_v_target = V_TARGET_SENTINEL
      self.output_a_target = 0.0
      self._emit_tag(v_ego, dt)
      return

    # dead-reckoning：停等期间停止点随本车前进而缩短
    self.actual_stop_distance = max(0.0, self.actual_stop_distance - v_ego * dt)

    if self.traffic_state in (OFF, GREEN):
      # 红灯证据消失 ⇒ 放开 v 路径（软限速不再压 v_cruise）。
      # ★ 但**故意不清 `stop_dist_m`** —— 这一条是从 DP 继承来的保守设计，
      #   别"顺手修掉"：
      #   * OFF 的含义是"不确定"（模型预测模糊 / 正在等绿灯冷却），
      #     此时状态机仍停在 STOPPING/STOPPED，**障碍物保持** ⇒ 车继续保守地停在
      #     原处；下一帧若重新确认红灯，就没有"放开→再刹"的抖动。
      #   * 真正的释放只走状态机回到 CRUISE 那条路（下面 state==CRUISE 分支把
      #     stop_dist_m 清成 None），而回 CRUISE 需要 GREEN 且冷却期已过 /
      #     驾驶员踩油门 / 前车太近 —— 都是明确的"可以走"证据。
      #   * 方向上是安全的：宁可多刹一帧，也不要在识别闪断时松开刹车。
      self.actual_stop_distance = 0.0
      self.output_v_target = V_TARGET_SENTINEL
      self.output_a_target = 0.0
      self._emit_tag(v_ego, dt)
      return

    # 正在停等 ⇒ 每帧强制同步滤波值
    self.stop_model_x_rl = self.stop_model_x_raw

    contribution = 0.0 if self.actual_stop_distance > 0.0 else self.stop_model_x_rl
    stop_dist = max(0.0, contribution + self.actual_stop_distance)

    # ★ 距离修正 = 固定物理修正(相机→车头) + 用户微调
    #   ★★ 符号：用户要求「正值 = 停止位置往前移（更靠前）」⇒ 正值要**减小**
    #      虚拟停止线距离 ⇒ 这里用**减**。DP 原版是加（正值=停更靠后），
    #      两者语义相反，勿照抄。
    adjust_term = -self.distance_adjust_m if ADJUST_POSITIVE_MOVES_STOP_FORWARD else self.distance_adjust_m
    stop_dist = get_traffic_stop_obstacle_distance(stop_dist, CAMERA_TO_FRONT_M + adjust_term)
    self._dbg_stop_raw = stop_dist      # 诊断：被门槛挡住时也要留原始值（文件头 ⑤）
    self.stop_dist_m = stop_dist

    if self.state == STOPPED:
      self.output_v_target = 0.0
      self.output_a_target = 0.0
    else:
      comfort_brake = COMFORT_BRAKE_BASE * STOPPING_BRAKE_DISCOUNT
      # ★ 2026-09-27 修（实车 swaglog 1270-1299 证据）：**只有几何上刹不住时
      #   才参与竞选**。
      #   旧写法 `self.output_v_target = min(v_limited, v_ego)` 在 v_limited >= v_ego
      #   时退化为 `v_ego`，而 `v_ego` 恒小于 cruise 候选的 `v_cruise` ⇒
      #   **只要进入 STOPPING 就必然抢赢 SP 层 `min(targets, key=v_target)`**，
      #   并把 `a_target=-2.16` 一起带出去（说"保持当前速度"却给减速，自相矛盾）。
      #   实车表现：26~54 km/h 正常巡航被大量判为 STOPPING（一趟 140 条探针）、
      #   用户体感"半路莫名减速"。
      #   现在：`v_limited < v_ego`（按舒适减速度已刹不住）才压 v_target；否则发
      #   哨兵严格不参与，且 a_target 与 v_target 永远自洽。
      #   停车能力不受影响：最后一段压制由 ①MPC 虚拟障碍物（stop_dist_m，主机制）
      #   ②STOPPED 态的 `output_v_target = 0.0` 完成。
      #   `stop_dist < 300` 守卫照抄 DP：更远时 v_limited 已高于任何合法 v_ego。
      v_limited = ((2 * comfort_brake * max(stop_dist - 1.0, 0.0)) ** 0.5
                   if stop_dist < 300.0 else float('inf'))
      self._dbg_v_limited = v_limited   # 诊断：离"该介入"还差多少（文件头 ⑤）
      if v_limited < v_ego:
        self.output_v_target = v_limited
        self.output_a_target = -comfort_brake
      else:
        self.output_v_target = V_TARGET_SENTINEL
        self.output_a_target = 0.0
        # ★ 2026-09-27 第二路（用户拍板）：同一个门槛也管住**交给 MPC 的虚拟停止线**。
        #   它是真正让车减速的那一路 —— MPC 拿到障碍物就会规划减速轨迹。
        #   置 None 后主 planner 传 `traffic_stop_obstacle_m=None` ⇒ MPC 不加第 3 栏
        #   障碍物；探针里的 `d=None` 正是"被门槛挡住"的可见证据。
        if MPC_OBSTACLE_ONLY_WHEN_MUST_BRAKE:
          self.stop_dist_m = None

    self._emit_tag(v_ego, dt)

  # ── 探针 ──────────────────────────────────────────────────────────────────
  def _entry_block(self, lead_present: bool, steering_angle_deg: float) -> str:
    """CRUISE 态下「没进入停等」是被哪几条挡住的。**纯诊断，不参与控制。**

    列出**所有**不满足项（逗号分隔）；`none` = 四门全开（照理就该进了）。
    顺序与 update() 里 CRUISE → STOPPING 那个与门一致。
    ★ `block=gas` 就是「10 s 踩油门抑制」还在计时中（见 GAS_SUPPRESS_FRAMES）。
    """
    fails = []
    if lead_present:
      fails.append('lead')
    if self.traffic_state != RED:
      fails.append('sig')
    if not is_traffic_stop_entry_allowed(steering_angle_deg):
      fails.append('steer')
    if self.gas_suppress_frames > 0:
      fails.append('gas')
    return ','.join(fails) if fails else 'none'

  def _heartbeat_on(self, v_ego: float) -> bool:
    """CRUISE（`tag=off`）这一帧要不要留痕。**纯诊断判断，不参与控制。**

    三种高价值现场一律留痕（哪怕没开车）：模型已判红灯却停在 CRUISE、
    前方有车、打着左转灯 —— 正好覆盖"该停没停"的三道嫌疑。
    其余情况按 `LOG_CRUISE_HEARTBEAT` 决定是否在行车中连续留痕。
    """
    if self.traffic_state == RED or self._dbg_lead or self._dbg_blinker == 'L':
      return True
    return LOG_CRUISE_HEARTBEAT and v_ego * 3.6 > 1.0

  def _emit_tag(self, v_ego: float, dt: float) -> None:
    """状态跳变必打 + 激活期 1 Hz + 「CRUISE 态」1 Hz（见 LOG_CRUISE_HEARTBEAT）。

    ★ 第三条是 2026-09-27 加的：原实现只在**状态跳变**时留痕 ⇒ 长时间
      CRUISE + `sig=red`（= 该管没管，正是"左转车道该停不停"的现场）整段
      只有第一帧有日志，事后无从复盘。
    cloudlog 最低只落 INFO ⇒ 用 info 级别。
    """
    tag = {CRUISE: 'off', STOPPING: 'stop', STOPPED: 'held'}[self.state]
    self._log_t += dt
    self._log_blk_t += dt
    show = False
    if tag != self._last_tag:
      self._last_tag = tag
      self._log_t = 0.0
      show = True
    elif tag != 'off' and self._log_t >= TRAFFIC_STOP_LOG_HZ:
      self._log_t = 0.0
      show = True
    elif tag == 'off' and self._log_blk_t >= TRAFFIC_STOP_LOG_HZ and self._heartbeat_on(v_ego):
      self._log_blk_t = 0.0
      show = True

    if show:
      sig = {OFF: 'off', RED: 'red', GREEN: 'green'}[self.traffic_state]
      if self.stop_dist_m is None:
        # ★ 被门槛挡住时也把**原始**停止距离打出来。原实现只给 `d=None` ⇒
        #   分不清"模型没识别到停止点"与"识别到了但判还刹得住"，定阈值没依据。
        raw = self._dbg_stop_raw
        d = 'None' if raw is None else f'None(raw={raw:.1f})'
      else:
        d = f'{self.stop_dist_m:.1f}'
      v_lim = 'inf' if self._dbg_v_limited == float('inf') else f'{self._dbg_v_limited:.1f}'
      self.log = (f"[TrafficStop] {tag} sig={sig} d={d} "
                  f"adj={self.distance_adjust_m:+.1f} vEgo={v_ego * 3.6:.1f} "
                  f"aOut={self.output_a_target:+.2f} "
                  f"vTgt={'inf' if self.output_v_target >= V_TARGET_SENTINEL else f'{self.output_v_target:.1f}'} "
                  f"vLim={v_lim} block={self._dbg_block} c={self._dbg_mask} "
                  f"lead={int(self._dbg_lead)} dRel={self._dbg_d_rel:.1f} blink={self._dbg_blinker} "
                  f"blinkR={int(self._dbg_blink_r)} "
                  f"steer={self._dbg_steer:+.1f} xEnd={self._dbg_x_end:.1f} "
                  f"yEnd={self._dbg_y_end:+.1f} mv={self._dbg_model_v:.1f} mv0={self._dbg_v_start:.1f}")
