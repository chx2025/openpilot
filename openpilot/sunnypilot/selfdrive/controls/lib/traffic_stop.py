"""
红灯 / 停止标志 —— 虚拟停止线模块（sunnypilot 适配 + 缺陷修正版）

来源
----
上游：herizon1054/openpilot @ dpagel
      dragonpilot/selfdrive/controls/lib/traffic_stop.py
它自身又是从 carrot (cp) fork 的 selfdrive/carrot/carrot_functions.py 移植的。

设计意图：**不需要独立的红绿灯识别模型**。把驾驶模型预测的轨迹里
「**模型打算停在哪儿**」当作证据（[第三轮]；首版/上游用的是轨迹末点），
一旦判定「模型确实要停、且现在不刹就要过线」，就凭空造出一条虚拟停止线障碍物
喂给纵向 MPC，让求解器像对待一辆停在路上的车那样规划整条 jerk-limited 刹车曲线。

本文件相对上游的改动
--------------------
A. 基底适配（sunnypilot / 新版 openpilot 布局）
   - import 路径 openpilot.*
   - LeadData 字段是 `present`（新版），不是上游的 `status`
   - 上游用 Params 存开关；本分支要新增 Param key 必须改 common/params_keys.h
     并重编译 libparams_c.so（key 白名单是编译进 C 库的，实测未登记 key 直接抛
     UnknownKeyName）。为做到**零编译上线**，开关改为独立 JSON 配置文件。
   - 上游的候选选择架构是 min(targets, key=v_target)（老 MPC）；
     本分支是 min(candidates, key=a_target)（新 MPC）——集成落点见
     longitudinal_planner.py / long_mpc.py 的改动。

B. 缺陷修正（编号与 port/traffic_stop_analysis.md 一致）
   [P0-1] 统一 `_deactivate()`。上游在两条提前返回路径上只清了 output_*，
          没清 stop_dist_m，导致 MPC 层继续被一条「已经不存在的停止线」约束
          （幽灵障碍物）。本版改为：只要 state == CRUISE 就必定释放；
          只要还在停等流程中，障碍物就始终有效（用航位推算推进），
          不再因为信号瞬间读不出来(OFF)而放开。
   [P0-2] 绿灯起步判据。上游静止时也要求「模型预测末速 > 5 m/s」且连续 5 帧，
          静止状态下这个门槛很难满足，会出现「红灯变绿但车不动、必须人工踩油门」。
          本版在 v_ego < 1 km/h 改用「模型给出的停止点在哪」做**双阈值滞回**：
            停止点 < 20 m  → 维持红灯（就停在这条线前）
            停止点 > 30 m  → 判定绿灯（模型已经把停止点放到很远/没有了）
            20~30 m        → 既不红也不绿，保持现状（天然去抖）
          另外要求「模型确实还有停止点」才维持 hold —— [第三轮](E2)。
   [P1-3] 停车点补偿。MPC 的障碍物硬约束是
            x_ego <= x_obstacle - LEAD_DANGER_FACTOR * desired_dist_comfort
          停稳时 desired_dist_comfort = STOP_DISTANCE(=4.5)，即车会停在虚拟障碍物
          前 0.75*4.5 = 3.375 m，再叠加 CAMERA_TO_FRONT_M(-1.5) 共约 4.9 m ——
          这正是上游不得不加「±5m 距离微调」参数、并承认「需要路测验证确切数字」
          的根因。本版直接把这一项补偿回去，让 stop_dist_m 回到它本来的语义
          （模型预测的停止线位置），用户的微调参数只承担 ±1~2 m 的个人偏好。
   [P1-4] 复核后**判为误判，保留上游 50 m**（详见 DISTANCE_FADE_BP_M 处的推导）。
          我最初按「40 m 处仍收缩到 85% → 停车点系统性偏近」把它当成缺陷，
          把窗口收到 10 m；数值复核发现方向正好相反：冻结发生在
          get_virtual(d) = RECALIBRATE_MIN_DISTANCE_M(10 m) 处，
          终端停车误差 = d_freeze - 10 = (1-applied(d_freeze)) * d_freeze，
          窗口**越大** applied 在近端越贴近 1.0、误差越小：
            50 m 窗口：0.00 m @0kph → 0.68 m @100kph
            10 m 窗口：0.00 m @0kph → 4.29 m @100kph   ← 改动是劣化
          已撤回，并把这次自我纠正写进注释，避免以后有人重复走这条路。
   [P1-5] 去掉「主动停车时 force sync」。上游在主动刹车路径上每帧
            self.stop_model_x_rl = self.stop_model_x_raw
          把上面的 rate limit 完全绕过，模型距离瞬间跳近 30 m 会直接穿透、
          造成额外重刹。本版保留 rate limit（其步长 v*dt+0.5 恒大于实际接近量
          v*dt，正常接近完全不受限，只挡抖动）。
   [P2-6] 横向容差 5.0 m → 2.0 m。模型 position 是自车路径坐标系，正常行驶
          |y_end| < 1 m，5 m 的容差几乎恒真、等于没判。
   [P2-7] `d_rel - 3.0` 只在**确实有前车**时才施加。上游在无前车时 d_rel 被置
          1000，该子条件恒真，是死代码。
   [P2-8] 左右转向灯对称判断（上游只判 leftBlinker）。
   [P2-9] STOPPED 超时提示日志（**不自动释放**，避免把安全兜底做成风险）。

C. 可观测性
   - `[TrafficStop]` 量化日志（INFO 级 + 1 Hz 节流；行车时 20 Hz 会把
     swaglog 刷爆。注意 cloudlog.debug 不落盘，swaglog 最低收 INFO）
   - 状态跳变单独打一条，不受节流
   - `[TrafficStopProbe]` 标定探针（见下）

D. 【第二轮修正】实车反馈「只需要减速的地方直接刹停了」
   理念：**斑马线等只需要减速的地方不介入，只在真正的红灯处介入停车。**
   首版把「模型要减速」当成了「模型要停车」，四层叠加导致只要有减速就刹停：

     (A) 判据错：`model_v < model_v_traj[0] * 0.7` 是「10 秒后比现在慢 30%」，
         纯减速判据 —— 弯道/斑马线/下坡/限速变化全部满足。
     (B) 动作错：保底候选写死 -2.16 m/s²，外层 min(candidates) 必然选中，
         误触发直接变成硬刹。
     (C) 去抖缺失：进入只要 1 帧 RED。
     (D) 只进不出：释放要求 GREEN（末速 > 5 m/s），门槛过高；叠加 [P0-1]
         「停等中障碍物始终有效」，一旦误触发就一路刹到停。

   对应修法（每一条都有独立常量/可调项，可单独回退）：
     (A) 只认「模型真的规划了一段静止」：末速 ≈ 0 **或** 轨迹静止段占比达标
         （轨迹尾部整段静止，时长域证据）。减速但速度曲线不触零 -> 不介入。
     (B) 保底候选改为按需反解 `a = v²/2d`（即「正好停在障碍物前所需的减速度」），
         上限仍是 comfort_brake。60 m 外只需 ~0.6 m/s²，误触发只值「轻抬油门」。
     (C) 改成 1 s 滚动窗口 + 帧数门槛（默认 0.6 s 证据）。用窗口而不是连续计数
         是因为实测停意图会单帧抖动，连续计数一抖即归零、真红灯前反而失效。
     (D) 释放条件加入「模型明确预测车还在走」（末速 > release_terminal_v_ms）。
         并与「没证据(OFF)」严格区分：OFF 继续 hold（[P0-1] 本意），
         只有模型**主动**说「在走」才释放。
     (E) 【同类缺陷，从实车日志里发现的】停稳后的维持判据。
         旧版只有 `model_x_end < STOP_SIGN_HOLD_X_M(20m)` —— 慢速跟车时模型预测
         10 秒只走 ~14 m，条件恒成立，模块一直 hold，`output_v_target = 0`
         把巡航目标压成 0，**ACC 跟不上车流**（日志实锤：
         `state CRUISE -> STOPPING (vEgo=0.0kph)` + `STOPPED/RED ... lead=1`）。
         现在改成合取：`predicts_standstill and x_end < 20`。
         静止签名用的是**轨迹尾部整段静止**（tail_standstill），
         不是「全轨迹静止采样占比」—— 后者会把线性爬升轨迹（0 → 1.4 m/s，
         36% 采样低于阈值）误判成要停。真停车一定表现为**结尾**平在 0。
         于是：路口停住（frac≈1.0）稳定 hold；车流一动立刻释放交还 ACC。
E. 【第三轮修正】判据从「速度轨迹形状」整体换成「模型预测的停止位置」
   第二轮实车反馈暴露出旧判据的物理死角，且两个方向是同一个根因：

     低速（< 约 25 km/h）：斑马线前模型只是减速，轨迹尾部偶尔贴近 0
                           -> 被误判成「要停车」-> 在不该停的地方停住
     高速（> 约 45 km/h）：10 s 视界装不下「停车」这件事
                           -> 末速永远不触零 -> 真红灯反而判不出来、刹不住

   **根因**：盯着「速度轨迹的末端形状」，而这个形状随车速变化极大 ——
   低速时它太容易被判成 0，高速时它根本到不了 0。

   第三轮改看**位置量**「模型打算停在哪儿」（停止点），它不受视界长度限制。
   四步合取，全部成立才介入：
     ① 模型在减速   : 末速 < 起点速度 × decel_ratio
     ② 停止点稳定   : 停止位置在 stop_pt_stable_s（默认 2 s）内基本没变
     ③ 停止点够近   : 不超过 assist_max_dist_m（安全上限，防模型给出离谱远点）
     ④ 来不及了     : required = v²/(2d) > comfort_decel_ms2
                      ——「以舒适刹车已经要过线」，此刻介入才叫**辅助**
     ⑤ 动作         : 给出 required 本身，上限 max_brake_ms2
                      （车速 ≥ high_speed_kph 时用更硬的 high_speed_brake_ms2）

   ⚠️ 用户提的「离停止点只有 8 米」在第 ④ 步里是**自动蕴含**的：
      8 m 正好是 20 km/h 的舒适刹车距离（5.56² / (2×2.0) = 7.7 m）。
      写成 required = v²/(2d) 就自动覆盖所有车速：40 km/h 时门限 31 m、
      80 km/h 时门限 123 m。固定 8 m 在高速上物理刹不住
      （8 m 内停住 50 km/h 需要 12 m/s²，已超出轮胎能力）。
   ⚠️ 第 ② 步已经要求「连续 2 s 一致」，所以进入 STOPPING **不再叠加确认窗口** ——
      省下那 0.6 s，在 50 km/h 上就是 8 m 刹车距离。
   ⚠️ 障碍物的锚点也从「轨迹末点」改成「停止点」：高速时末点还在半路上，
      拿它当停止线会把障碍物摆错位置。

   (E2) 停稳后的维持判据（[第二轮]发现、[第三轮]随判据一起修正）：
        旧版只有 `x_end < 20 m`，慢速跟车时恒成立 -> 一直 hold，
        `output_v_target = 0` 把巡航目标压成 0 -> **ACC 跟不上车流**。
        现在要求「模型确实还有停止点」才 hold。

   阈值全部做成 JSON 可热调（tuning 段，1 s 生效），并保留 [TrafficStopProbe]
   探针日志 —— 这些「模型预测要停」的量化定义取决于具体模型的轨迹行为，
   本机是 TSFDOM (recompiled20)，**只能靠实车数据标定，不能靠猜**。

回退
----
配置文件里 "enabled": false 即完全关闭（等价于模块不存在）；
所有修正项都有独立常量，可单独回退。

F. 【第九轮 方案 A】跟车让位判据重做（用户选项 A）
   问题：>30 kph 时仍然「停过线」。
   本轮动的不是刹车力，是**让位逻辑**——三处门限原本互不相同：
     · check_model_stopping 的 ⑥ 让位块：`d_rel <= 30 m` 一律让位
     · stop_sign 的 ahead_of_lead    ：`d_to_stop < d_rel - 3 m`
     · update() 的入口闸门           ：`(not lead_present) or (d_rel > 30 m)`
     · update() 的 lead_cancels      ：`d_rel - stop_model_x_raw < 4 m`
   四个表达式描述同一件事，边界必然打架。真正的后果是**方向性错误**：
   停止线在 12 m、前车在 25 m（前车明明在停止线**之后**）时，⑥ 与入口闸门
   都因为「25 < 30」判让位 -> 辅助整个关闭 -> 眼睁睁越过 12 m 的停止线。

   改成：让位 ⟺ `d_rel < d_to_stop + lead_yield_margin_m`（默认 4 m），
   即「前车确实在停止线之前才让位」——这是唯一正确的物理判据：
     d_rel < d_to_stop -> 车会先被前车挡住，虚拟停止线没有意义 -> 交给 ACC
     d_rel > d_to_stop -> 停止线更近，本就该停在前车之前 -> 保持介入
   三处（stop_sign / ⑥ / 入口闸门）共用 is_lead_yielding_stop()，
   30 m 退为「模型没给停止点、无从比较」时的兜底。

   另：`reason='lead'` 的退出**不计入**短命停等 6 s 封锁 —— 让位是正常移交，
   不是抖动。把它算成抖动会让模块在前车驶离后被封锁 6 s，
   高速段那是 ~84 m 的盲区，本身就是「停过线」的放大器。
   防抖改由入口闸门自身承担（前车还在线前就持续挡住再介入）。

   代价（已在选方案时说明）：跟车时可能多一次轻刹 ——
   即前车在停止线之后的那类工况，模块会按停止线刹，而不是一路跟到前车跟前。
   新旋钮 tuning.lead_yield_margin_m 可热调（1 s 生效）：
   调大更积极让位，取负值要求前车「明确更近」才让位。
"""
import time
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.common.traffic_stop_config import (
  CONFIG_PATH,
  DEFAULT_DISTANCE_ADJUST_M,
  DEFAULT_ENABLED,
  DISTANCE_ADJUST_LIMIT_M,
  StopTuning,
  load_config as _load_shared_config,
  load_tuning as _load_shared_tuning,
)
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX

# ── 状态机 ──────────────────────────────────────────────────────────────
CRUISE, STOPPING, STOPPED = 0, 1, 2
OFF, RED, GREEN = 0, 1, 2

_STATE_NAME = {CRUISE: "CRUISE", STOPPING: "STOPPING", STOPPED: "STOPPED"}
_SIGNAL_NAME = {OFF: "OFF", RED: "RED", GREEN: "GREEN"}

# ── 开关 / 微调（独立配置文件，绕开 Params 白名单以免重编译 C 库）──────
# 文件读写的实现在 openpilot/common/traffic_stop_config.py，与 UI 侧共享同一份：
#   - 设置页的 "Traffic Light Stop" 开关直接改 /data/traffic_stop.json
#   - 本模块每 CONFIG_POLL_FRAMES 读一次 → 改动 1 s 内热生效，无需重启
# CONFIG_PATH / DEFAULT_ENABLED / DEFAULT_DISTANCE_ADJUST_M / DISTANCE_ADJUST_LIMIT_M
# 均从该模块 import（见文件头导入区），此处不再重复定义，保证两侧默认值永远一致。
CONFIG_POLL_FRAMES = round(1.0 / DT_MDL)   # 1 s 轮询一次
CAMERA_TO_FRONT_M = -1.5                    # 固定物理修正（相机 → 前保险杠），非用户可调

# ── 进入条件 ────────────────────────────────────────────────────────────
TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0
RATE_LIMIT_CLOSING_MARGIN_M = 0.5
RECALIBRATE_MIN_DISTANCE_M = 10.0
CANCEL_LEAD_MARGIN_M = 4.0                  # [第九轮] 已并入 tuning.lead_yield_margin_m 的默认值
                                            #   （两者必须相等，verify_traffic_stop.py 有断言守住）。
                                            #   这里保留常量只为 verify 脚本复刻上游行为时引用。
LEAD_CLEARANCE_M = 3.0                      # [P2-7] 仅在有前车时施加
                                            #   [第九轮] 判据里已不再直接使用它 ——
                                            #   三处门限（ahead_of_lead / 让位块 / 入口闸门）
                                            #   统一收敛到 is_lead_yielding_stop()。
                                            #   常量保留，供 verify 复刻「上游旧行为」用。
# [第四轮 2026-09-19] 用户实车要求「30 m 内有前车就不触发红灯停车辅助」。
# [第九轮 2026-09-20] 方案 A：该要求从「固定 30 m 距离门限」细化为
#   「前车**确实在虚拟停止线之前**才让位」——
#     判据公式   : 让位 ⟺ d_rel < d_to_stop + tuning.lead_yield_margin_m
#     30 m 的用途: 退为兜底（模型没给停止点、无从比较时）
#   实车依据：两次抱怨的红灯，接近段 LongDecel 全是 lead=1 / dRel=5~10 m 的
#   紧跟前车，而 TrafficStop 在整段减速里一次都没打日志 —— 介入对象本来就该是
#   「停止线」，不是「前车」。但反过来，前车在停止线**之后**时让位同样错：
#   停止线更近，我们本就该停在前车之前。详见 is_lead_yielding_stop() 的说明。

# ── 判据 ────────────────────────────────────────────────────────────────
STOP_SIGN_MAX_SPEED_KPH = 82.0              # 超过这个车速不介入（保留上游语义）
# [第三轮] 下面两个常量**只剩对照价值**：判据已从「按车速插值探测距离」改成
# 「按剩余刹车能力 required > comfort」，它们仍被 verify 脚本用来复刻上游行为。
STOP_SIGN_DETECT_DIST_BP_KPH = (60.0, 80.0)
STOP_SIGN_DETECT_DIST_M = (120.0, 150.0)
STOP_SIGN_LATERAL_TOLERANCE_M = 2.0         # [P2-6] 5.0 -> 2.0
STOP_SIGN_HOLD_X_M = 20.0                   # [P0-2] 已停稳时维持红灯的阈值
GREEN_START_X_M = 30.0                      # [P0-2] 已停稳时判定起步的阈值（滞回）
MOVING_START_V_MS = 5.0                     # 行驶中的起步判据（保留上游语义）
MOVING_START_DELTA_MS = 2.0
STOP_POINT_MIN_TRAJ_POINTS = 4              # 轨迹点太少就不做停止点外推

# ── [第九轮 方案 A] 跟车让位的贴近下限 ─────────────────────────────────
# 前车进到这个距离以内就**无条件**让位，与模型给出的停止点无关。
# 这是安全属性（不是调参项），所以写死成常量：
#   模型在车流跟停时会预测「就地停住」（stopPoint ≈ 0~1 m）。若只看
#   「谁更近」的字面规则，前车 6 m + 停止线 1 m 会判「停止线更近」，
#   于是模块在一个 6 m 外有车的场合抢走纵向控制、反解出巨大减速度。
#   10 m 以内「谁更近」已无意义 —— 跟随前车是唯一合理做法。
LEAD_CLOSE_YIELD_M = 10.0

# ── 距离整形 ────────────────────────────────────────────────────────────
DISTANCE_RATIO_SPEED_BP_KPH = (0.0, 100.0)
DISTANCE_RATIO = (1.0, 0.7)
DISTANCE_FADE_BP_M = (0.0, 50.0)            # [P1-4] 保留上游值（见下方推导，收窄窗口是劣化）
# [P1-4] 为什么不能用小窗口（这段是踩过坑之后留下的推导，请勿再收窄）：
#   applied(d) = np.interp(d, DISTANCE_FADE_BP_M, [1.0, distance_ratio])
#   实车停稳前的最后一个「重标定」发生在 get_virtual(d) = RECALIBRATE_MIN_DISTANCE_M
#   的地方（再近就不再重标定，改由航位推算推进），因此
#       终端停车误差 = d_freeze - RECALIBRATE_MIN_DISTANCE_M
#                   = (1 - applied(d_freeze)) * d_freeze
#   窗口越小 → applied 越早被钳到 distance_ratio → d_freeze 越大 → 误差越大：
#       50 m（上游）: 0.00 m @0kph  0.40 m @62kph  0.68 m @100kph
#       10 m        : 0.00 m @0kph  2.28 m @62kph  4.29 m @100kph
#   窗口够大时 applied 在近端 ≈ 1，误差被压到亚米级，且对接近车速基本不敏感。

# ── 刹车 ────────────────────────────────────────────────────────────────
COMFORT_BRAKE_BASE = 2.4
STOPPING_BRAKE_DISCOUNT = 0.9
STOPPED_SPEED_MS = 0.3

# ── 计时/滤波 ───────────────────────────────────────────────────────────
STOPPED_GRACE_FRAMES = round(0.5 / DT_MDL)
START_CONFIRM_FRAMES = round(0.2 / DT_MDL)   # 起步/绿灯证据所需帧数（1 s 窗口内）
GAS_SUPPRESS_FRAMES = round(10.0 / DT_MDL)
MEDIAN_WINDOW = 3
MOVING_AVG_WINDOW = 15
MODEL_V_WINDOW = 10                          # 仅探针日志用的平滑窗口

# ── 证据窗口 ────────────────────────────────────────────────────────────
# 「模型在走」/「模型想起步」这两个证据仍需 1 s 滚动窗口来去抖
# （模型停意图实测会单帧抖动，连续计数一抖即归零）。
EVIDENCE_WINDOW_FRAMES = round(1.0 / DT_MDL)        # 20 帧 = 1 s
# [第三轮] 停止点的稳定性窗口上限（deque 长度）。真正用多少帧由
# tuning.stop_pt_stable_s 决定（可热调 0.2~3.0 s），这里留 3 s 的余量。
STOP_PT_WINDOW_FRAMES = round(3.0 / DT_MDL)         # 60 帧 = 3 s

# ── 保底候选（[本次修正]：由「恒定值」改为「按需值」）──────────────────
# 上游/首版给外层 planner 的候选是**恒定** -2.16 m/s²，一旦误触发就硬刹。
# 现在按 stop_dist 反解「正好停在障碍物前所需的减速度」，只补差值、不越权。
# 反解时预留的落脚余量由 tuning.assist_margin_m 控制（默认 1.0 m）。

# ── [P1-3] MPC 裕度补偿 ─────────────────────────────────────────────────
# long_mpc.py 的约束 x_ego <= x_obstacle - LEAD_DANGER_FACTOR * desired_dist_comfort
# 停稳时 desired_dist_comfort = STOP_DISTANCE，即虚拟障碍物前方还得留
# 0.75 * 4.5 = 3.375 m。这里把障碍物往后放同样的量，使最终停车点正好落在
# 「模型预测的停止线」上。若 long_mpc.py 的这两个常量改了，这里必须同步
# （verify_traffic_stop.py 里有断言守住）。
FROM_LONG_MPC_LEAD_DANGER_FACTOR = 0.75
FROM_LONG_MPC_STOP_DISTANCE = 4.5
MPC_STOP_MARGIN_COMP_M = FROM_LONG_MPC_LEAD_DANGER_FACTOR * FROM_LONG_MPC_STOP_DISTANCE

# ── 日志 ────────────────────────────────────────────────────────────────
LOG_INTERVAL_S = 1.0
STOPPED_TIMEOUT_LOG_S = 20.0


def _load_config():
  """读独立配置文件，返回 (enabled, distance_adjust_m)。

  实现与 UI 侧共享（openpilot/common/traffic_stop_config.py）：
  文件不存在 / 损坏 / 字段缺失 → 默认值（默认开启）。
  """
  return _load_shared_config()


def _load_tuning() -> StopTuning:
  """读判据阈值。缺失/非法项回落到 StopTuning 的默认值（纯标准库实现，永不抛）。"""
  return _load_shared_tuning()


# ── 纯函数（可离线单测）────────────────────────────────────────────────

def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """大转角说明这是在转弯、不是接近停止线。只拦「新进入」STOPPING，不影响已激活的停等。"""
  return abs(steering_angle_deg) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_lead_yield_threshold(stop_point_m: float | None, tuning: StopTuning) -> float:
  """让位判据的比较基准：前车距离 **小于** 它就让位。

  = max(模型停止点 + lead_yield_margin_m, LEAD_CLOSE_YIELD_M)

  `LEAD_CLOSE_YIELD_M` 这条下限不是调参项，是**安全属性**，所以做成常量：
  模型给出的停止点偶尔会离谱地近（车流跟停时它会预测「就地停住」，
  stopPoint ≈ 0~1 m）。若只按「前车比停止线远就让位」的字面规则，此时
  前车在 6 m、停止线在 1 m -> 判「停止线更近」-> 模块在一个 6 m 外
  有车的场合抢过纵向控制权、按 v²/2d 反解出一个巨大减速度。
  前车进了 10 m 以内，「谁更近」这个问题本身就没有意义了 ——
  这么近的距离上跟随前车是唯一合理的做法，与模型说了什么无关。

  （同一处推导也解释了 M4 用例：模型一路说「停在 D」，而夹具让自车
  开环匀速穿过 D，跑到最后 D≈1 m —— 正是上面这个退化情形。）
  """
  if stop_point_m is None:
    return float(tuning.lead_suppress_dist_m)
  return max(float(stop_point_m) + tuning.lead_yield_margin_m, LEAD_CLOSE_YIELD_M)


def is_lead_yielding_stop(lead_present: bool, d_rel: float, stop_point_m: float | None,
                          tuning: StopTuning) -> bool:
  """[第九轮 方案 A] 前车是否应当「让位」（= 本模块放手，把纵向交还给常规跟车）。

  ── 判定让位的正确问题是：谁才是更近的那个约束？────────────────────────
    · 前车在停止线**之前**（`d_rel < d_to_stop`）—— 车会先被前车挡住，
      虚拟停止线毫无意义。此时继续拿它约束 MPC，等于把「跟前车」换成
      「跟一条我们猜出来的线」，停车位置反而离谱。
      实车依据（2026-09-19）：两次抱怨的红灯，接近段 LongDecel 全是
      lead=1 / dRel=5~10 m 的紧跟前车，模块全程零日志 —— 本来就该让位。
    · 前车在停止线**之后**（`d_rel > d_to_stop`）—— 停止线更近，我们本就该
      停在**前车之前**（至少 lead_yield_margin_m 米）。让位反而是放弃辅助，
      车会一直开到前车跟前，于是「偏离停止线」变成「停过线」。
      这正是 >30 kph 时「停过线」的现场工况之一。

  旧判据（`d_rel <= lead_suppress_dist_m`，即 30 m 内一律让位）把这两种情况
  混为一谈 —— 停止线在 12 m、前车在 25 m 时照样让位，辅助整个消失。

  ── 为什么必须和「入口闸门」用**同一个**函数 ──────────────────────────
  第五轮踩过「判据说可以介入、状态机说不能让进」的分裂：入口写 `not lead_present`、
  判据写距离门限。现在三处（stop_sign 的 ahead_of_lead、⑥ 让位块、入口闸门）
  全部调用本函数 —— 只要它们还是两个表达式，就一定会在某个工况下漂开。

  ── 兜底 ──────────────────────────────────────────────────────────────
  模型这帧没给停止点（高速下 74% 的帧如此）就没有比较基准，
  退回固定距离 `lead_suppress_dist_m`。只在这一种情况下生效。
  """
  if not lead_present:
    return False
  return bool(float(d_rel) < get_lead_yield_threshold(stop_point_m, tuning))


def is_lead_close_in(lead_present: bool, d_rel: float) -> bool:
  """前车压进 LEAD_CLOSE_YIELD_M 以内 —— **立即**交还，不等 0.6 s 取消窗口。

  与 is_lead_yielding_stop 不是竞争关系：它是后者的**子集**
  （让位阈值恒 >= LEAD_CLOSE_YIELD_M），只是同一个决定里的「紧急版」。
  旧版之所以会立即释放，靠的是另一套 `lead_cancels`（`d_rel - 平滑停止点 < 4 m`）
  —— 那套表达式对「有车突然切入」反应更快，直接删掉会丢掉这个安全性；
  这里用「前车 < 10 m」这个明确得多、且与让位判据同源的量把它接回来。
  """
  return bool(lead_present) and float(d_rel) < LEAD_CLOSE_YIELD_M



def get_traffic_stop_reference_speed(v_ego_kph: float, previous_reference_kph: float | None) -> float:
  """latch 住本次停等事件期间见过的最高车速，单调不减。"""
  return max(0.0, v_ego_kph, previous_reference_kph or 0.0)


def get_virtual_traffic_stop_distance(model_distance: float, v_ego_kph: float) -> float:
  """车速越高，起始刹车点越靠前（100% @ 0kph → 70% @ 100kph）。

  [P1-4] 在最后 DISTANCE_FADE_BP_M 米内快速回到 100%，使最终停车位置不依赖接近速度。
  """
  distance_ratio = np.interp(v_ego_kph, DISTANCE_RATIO_SPEED_BP_KPH, DISTANCE_RATIO)
  applied_ratio = np.interp(model_distance, DISTANCE_FADE_BP_M, [1.0, distance_ratio])
  return max(0.0, model_distance * applied_ratio)


def get_traffic_stop_obstacle_distance(stop_distance: float, distance_adjust: float) -> float:
  """施加固定物理修正 + 用户微调。整条管线里只能调用一次（上游 cp 的历史 bug 8.3 是调了两次）。"""
  return max(0.0, stop_distance + distance_adjust)


def get_model_stop_point(v_traj, x_traj, tuning: StopTuning) -> tuple[float | None, dict]:
  """模型「打算停在哪儿」= 停止点（米，自车位置起算）。None = 没给出可信的停意图。

  ── 为什么把判据从「速度轨迹形状」换成「停止位置」──────────────────────
  模型的速度轨迹只有 10 s 视界那么长。车速一高，视界内速度根本降不到 0
  （50 km/h 舒适刹停就要 10 s 以上）——「末速触零」这条判据在高速上必然失效。
  这正是「高速容易红灯刹不住」的根因；而低速时相反，轨迹尾部偶尔接近 0
  就会被误判成「要停车」（斑马线场景）。
  **两者是同一个根因：盯着速度轨迹的末端形状，而这个形状随车速变化极大。**

  「停止位置」是个**位置量**：模型很早就会给出、而且会保持稳定，不受视界
  长度限制。所以第三轮把判据整体换成它。

  两条取停止点的路径：
    ① 视界内已经出现静止段 -> 静止段的起点位置就是停止点（模型明确规划了停车）
    ② 视界内没触零        -> 用轨迹末段自身的减速度外推「继续这样减会停在哪」
  两条都拿不到 -> None。

  路径②的时间用「距离 / 平均速度」反推，不依赖视界的步长常量 ——
  模型换版本、换输出点数都不会算错。路径②还额外要求「确实在明显减速」
  且「末速已经很低」，否则「慢下来继续走」会被外推成一个假停止点。
  """
  info = {"stop_point": None, "stop_pt_src": "none", "a_tail": 0.0,
          "extrap_span_s": 0.0, "standstill_span": 0.0}
  v = np.asarray(v_traj, dtype=float).reshape(-1)
  x = np.asarray(x_traj, dtype=float).reshape(-1)
  n = int(min(v.size, x.size))
  if n < STOP_POINT_MIN_TRAJ_POINTS:
    return None, info
  v, x = v[:n], x[:n]

  # ── 路径①：视界内已静止 ──
  below = v < tuning.stop_terminal_v_ms
  if bool(below[-1]):
    i = n - 1
    while i > 0 and bool(below[i - 1]):
      i -= 1
    info.update(stop_point=float(x[i]), stop_pt_src="horizon",
                standstill_span=1.0 - i / float(n))
    return float(x[i]), info

  # ── 路径②：视界内没触零 -> 用末段减速度外推 ──
  if float(v[-1]) > tuning.extrap_max_v_end_ms:
    return None, info                     # 末速还高 -> 无从判断「是停还是只是慢下来」
  k = min(max(2, int(round(tuning.extrap_frac * n))), n - 1)
  v_end, v_ref = float(v[-1]), float(v[-1 - k])
  dx = float(x[-1]) - float(x[-1 - k])
  v_mean = 0.5 * (v_end + v_ref)
  if dx <= 0.0 or v_mean < 1e-3:
    return None, info                     # 轨迹不自洽 -> 不给停止点
  span = dx / v_mean
  a_tail = (v_end - v_ref) / span
  info.update(a_tail=a_tail, extrap_span_s=span)
  if a_tail > -tuning.min_decel_ms2:
    return None, info                     # 没在明显减速 -> 只算「减速」，不算「要停」
  stop_point = float(x[-1]) + v_end * v_end / (2.0 * -a_tail)
  info.update(stop_point=stop_point, stop_pt_src="extrap")
  return stop_point, info


def get_stop_point_spreads(present: list[float]) -> tuple[float, float]:
  """返回 (原始极差, 稳健极差)。

  稳健极差 = 先剔掉离中位数最远的那**一个**样本，再算 max-min。
  为什么要剔：实车实测车停稳整 3 s（窗口 60/60 全满）后原始极差仍有 3.1 m，
  即模型给出的是「一条基本不变、偶尔跳一帧」的曲线。用原始 max-min 会被单帧尖峰
  一票否决，所以先剔掉离中位数最远的那一个样本再算极差。

  [第七轮] 抽成独立纯函数：日志里要同时打出两个值 —— 只有拿到**稳健极差**的实测
  分布，stop_pt_tol_speed_gain 才有得标定（此前日志只打了原始极差，正是这一点
  导致固定容差 3.0 m 在高速段全军覆没却看不出来）。
  """
  if not present:
    return 0.0, 0.0
  raw_spread = max(present) - min(present)
  if len(present) < 4:
    return raw_spread, raw_spread
  srt = sorted(present)
  med = 0.5 * (srt[(len(srt) - 1) // 2] + srt[len(srt) // 2])
  worst = max(range(len(present)), key=lambda i: abs(present[i] - med))
  kept = [v for i, v in enumerate(present) if i != worst]
  return raw_spread, max(kept) - min(kept)


def get_stop_point_tolerance(v_ego_kph: float, tuning) -> float:
  """[第七轮] 实际容差 = 静止容差 + 车速增益 × 车速。

  物理依据：模型的停止点估计带**推理延迟**，延迟期间自车仍在前进，
  于是位置误差 ≈ v_ego × τ 且**与车速成正比**（τ ≈ 0.2 s → 50 km/h 约 2.8 m）。
  实车实测原始极差 ≈ 0.8 + 0.22 × v_kph，与此一致。

  固定容差（旧行为）等价于「假设误差与车速无关」—— 结果是：低速（≤8 kph）
  能通过、高速整段被否。实车 50+ km/h 两次红灯因此全程 outA=0.00，越过停止线约 2 m。
  """
  return tuning.stop_pt_tol_m + tuning.stop_pt_tol_speed_gain * max(0.0, v_ego_kph)


def get_decision_window(stop_pt_win, min_frames: int) -> list[float]:
  """判据实际使用的窗口 —— 最近 min_frames 帧（= stop_pt_stable_s 秒）内的停止点。

  [第七轮] 修一个**隐藏不一致**：stop_pt_win 这个 deque 的长度是 3 s
  （STOP_PT_WINDOW_FRAMES = 60），而 min_frames 此前只被用来做「攒够帧数了吗」
  的计数 —— 极差却是在整个 60 帧上算的。等于**偷偷拿 3 s 的容差判 2 s 的稳定性**。
  50 km/h 时 3 s 与 2 s 相差 14 m 自车位移，模型对停止点的修正在这多出来的 1 s 里
  也在累积，极差被系统性放大 —— 日志里 (60/60) 却显示「窗口全满」，看不出来。

  修法：极差只在「判据自称的那个窗口」上算。判据、日志、DEFAULT 语义三者统一。
  """
  if not stop_pt_win:
    return []
  tail = list(stop_pt_win)[-max(1, int(min_frames)):]
  return [float(v) for v in tail if v is not None]


def is_stop_point_stable(stop_pt_win, tol_m: float, present_ratio: float,
                         min_frames: int, tol_extra_m: float = 0.0,
                         hard_m: float | None = None) -> bool:
  """用户第 2 步：模型给出的停止位置在一段时间内「基本没变」。

  ⚠️ 窗口里存的必须是**绝对坐标系**下的停止点（见 check_model_stopping 里那段
  「坐标系」注释）。若存自车坐标系的原始值，自车每前进 1 s 就给自己造出 v_ego 米的
  「漂移」，判据在行驶中永远不成立 —— 那正是本模块此前「只在停稳后才介入」的根因。

  三条同时成立才算稳定：
    - 窗口里已经攒够 min_frames 帧（= stop_pt_stable_s）
      —— 刚启用/刚复位时窗口是空的，不能因为「只有 1 帧且那一帧很稳」就放行
    - 该窗口里**有停止点的帧**占比 >= present_ratio
    - 该窗口里有值帧的停止点稳健极差 <= tol_m + tol_extra_m

  ⚠️ [第八轮 2026-09-20] present_ratio 的原设计前提是错的。
  旧文档写「模型一会儿说有停意图、一会儿没有，说明它自己也没想清楚」，据此把
  present_ratio 默认设成 0.9 —— 即要求 3 s 窗口里 90% 的帧都输出停止点。
  但实车（2026-09-20 夜间一圈，[TrafficStopProbe] 全量统计）：
      **行驶中（v > 5 kph）模型有 74% 的帧根本不给停止点**（src=none 170 / 231 帧）。
  26% 的输出率 vs 90% 的要求 —— stable 在高速段**数学上不可能成立**，与容差无关。
  于是介入被推迟到车慢下来、模型输出变密之后（实测 5 次 engage 全发生在
  v ≤ 36.5 kph，其中 3 次 ≤ 20 kph），「来不及了」才补刹车，用户看到的就是
  「介入晚 + 刹不住 + 过线」。第七轮改容差因此完全无效 —— 根本走不到算极差那一步。

  正确认识：**模型在高速时按帧稀疏地输出停止点，是它的不确定性表达方式，不是
  「没想清楚」。** 判据该问的是「在它开口的那些帧里，说法一致吗」—— 极差本来就只
  在有值帧上算，所以只要把「开口帧数」的门槛降到与真实输出率相容即可。
  防误触发不靠这道门，而靠它的下游：required > comfort_decel_ms2（真的快停不住）、
  decelerating（模型自己确实在减速）、横向容差、以及 ⑥ 的跟车让位（第九轮）。
  present_ratio 默认值因此从 0.9 放到 0.3（= 窗口里 ≥ 0.6 s 的稳定停止点输出）。
  回退：present_ratio = 0.9 即精确回到第七轮行为。

  tol_extra_m / hard_m（[第七轮] 新增，默认值保持第五/六轮行为不变）：
    - tol_extra_m 由调用方按车速给出（见 get_stop_point_tolerance）——
      抵消「推理延迟 × 车速」那部分确定性误差。传 0 即回到固定容差。
    - hard_m 是**原始**极差（含离群点）的绝对上限，防止「真的在连续漂移」被
      当成稳定。None -> 沿用旧的 2 × 容差。
  """
  if len(stop_pt_win) < max(1, min_frames):
    return False
  # [第八轮 2026-09-20] present 也只在**判据自称的那个窗口**（最近 min_frames 帧）上统计。
  # 此前分母取的是整个 deque（STOP_PT_WINDOW_FRAMES = 60，3 s），而极差只在最近
  # min_frames（40，2 s）上算 —— 判据自称 2 s、收票却按 3 s，与极差口径又不一致。
  # 这是 get_decision_window 修过一次的同一个坑的**第二处**：只要判据与取样是两个
  # 表达式，它们就会漂开。现在两者都从 win 出发。
  win = list(stop_pt_win)[-max(1, int(min_frames)):]
  present = [float(v) for v in win if v is not None]
  if len(present) < max(1, int(round(present_ratio * len(win)))):
    return False
  tol_eff = tol_m + tol_extra_m
  raw_spread, trimmed_spread = get_stop_point_spreads(
    get_decision_window(stop_pt_win, min_frames))
  cap = (2.0 * tol_eff) if hard_m is None else float(hard_m)
  if raw_spread > cap:
    return False
  return trimmed_spread <= tol_eff



def check_model_stopping(stop_pt_win: deque, start_win: deque, cancel_win: deque, state: int,
                         v_cruise: float, model_v_traj, model_x_traj, v_ego: float,
                         a_ego: float, model_y_end: float, d_rel: float,
                         lead_present: bool, tuning: StopTuning,
                         ego_travel_m: float) -> tuple[int, dict]:
  """返回 (traffic_state, debug)。三个 deque 由本函数负责推进（1 s / 3 s 滚动窗口）。

  ego_travel_m：调用方维护的**累计里程**（绝对坐标系原点），用于把停止点换算到绝对系。

  ── ⚠️⚠️ [第五轮] 坐标系：本模块最隐蔽、也最致命的一个坑 ──────────────
  `model_v2.position.x` 是**自车坐标系**（原点在自车、向前为正）。
  于是「模型预测的停止点」这个读数里，混进了自车自己的位移：
  自车前进 v_ego·dt，同一个物理停止线在自车坐标里就后退 v_ego·dt。
  ⇒ 一个**纹丝不动的**停止点，在自车坐标系下也会以 v_ego 的速度匀速「漂移」。
    50 km/h 时 2 s 漂移 27.8 m，远大于 stop_pt_tol_m(3.0)。
  ⇒ 稳定性判据（用户第 2 步）在行驶中**数学上不可能成立**；只有当
    v_ego ≲ 3.0/2.0 = 1.5 m/s（≈5 km/h）时才可能成立。
  ⇒ 实测后果：本模块的「行驶中判据」是一条死路，唯一能成立的入口是
    「已停稳」分支（只判 stop_point < STOP_SIGN_HOLD_X_M）。
    实车 26 次介入里 23 次 vEgo=0.0、以及用户抱怨的「过停止线没有辅助刹车」
    （req=0.00 / outA=-0.00，一点刹车力都没给）由此而来。
  修法：进窗口前把停止点加上累计里程（换算到绝对系）。这样判的就只剩
  **模型自身的预测抖动**（亚米级），而不是自车的位移。见 stop_pt_abs。

  ── [第三轮] 判据：模型「打算停在哪儿」─────────────────────────────────
  四步合取，全部成立才介入：
    ① 模型在减速   : 末速 < 起点速度 × decel_ratio
    ② 停止点稳定   : 停止位置在 stop_pt_stable_s 内基本没变（用户第 2 步）
                     ↑ 必须在绝对坐标系里判，见上
    ③ 停止点够近   : 不超过 assist_max_dist_m（安全上限，防模型给出离谱远点）
    ④ 来不及了     : required = v²/(2d) > comfort_decel_ms2
                     ——「以舒适刹车已经要过线」，此刻介入才叫**辅助**
    ⑤ 动作         : 给出 required 本身（上限 max_brake_ms2；高速段用 high_speed_brake_ms2）
    ⑥ 跟车让位     : 前车**确实在停止线之前**才让位（[第九轮 方案 A]）
                     —— 判据 `d_rel < d_to_stop + lead_yield_margin_m`，两个分支都挡。
                     [第四轮] 原始要求是「30 m 内有前车就不触发」，[第九轮] 细化为
                     「前车比停止线更近才让位」，30 m 退为「模型没给停止点」时的兜底。
                     详见 is_lead_yielding_stop() 的说明。

  ⚠️ 用户提的「离停止点只有 8 米」在第 ④ 步里是**自动蕴含**的：
      8 m 正好是 20 km/h 的舒适刹车距离（5.56² / (2×2.0) = 7.7 m）。
      写成 required = v²/(2d) 就同时覆盖了所有车速：
      40 km/h 时门限是 31 m，80 km/h 时是 123 m。
      而固定的 8 m 在高速上**物理刹不住**（8 m 内停住 50 km/h 需要 12 m/s²）——
      这正是「高速容易红灯刹不住」的由来。

  ⚠️ 第 ② 步已经要求「连续 2 s 一致」，所以进入 STOPPING **不再叠加确认窗口** ——
      省下那 0.6 s，在 50 km/h 上就是 8 m 刹车距离。

  ── 为什么这样就不会在斑马线误触发 ──────────────────────────────────────
  斑马线（或弯道/限速变化）处模型只是减速、不打算停：
    - 视界末速不为 0、又不够「明显减速」-> 拿不到停止点 -> 第 ② 步不成立
    - 就算勉强外推出一个停止点，它也很远、required 远小于 comfort
      -> 第 ④ 步不成立（「现在的刹车刹得住，不用管」）
  两条各自独立地把它挡掉 —— 这是本判据和上游「减速即停」最本质的区别。

  ── 「释放」与「没证据」必须区分 ────────────────────────────────────────
  OFF（没有停止点，多半是抖动）-> 继续 hold（[P0-1] 的本意）。
  cancel_sign（模型**明确**不再打算停 / 明确还在走）-> 攒够帧数即释放。
  """
  v_ego_kph = v_ego * 3.6
  v_traj = np.asarray(model_v_traj, dtype=float)

  stop_point, sp = get_model_stop_point(v_traj, model_x_traj, tuning)
  # [第五轮] 换算到绝对坐标系再进窗口 —— 否则自车位移会被当成「停止点在漂移」。
  # stop_point 本身（自车系）继续用于 ③④ 的门限计算，两者用途不同，都要留。
  stop_pt_abs = None if stop_point is None else float(stop_point) + float(ego_travel_m)
  stop_pt_win.append(stop_pt_abs)
  stable_frames = max(1, int(round(tuning.stop_pt_stable_s / DT_MDL)))
  present = [float(v) for v in stop_pt_win if v is not None]
  # [第七轮] 容差随车速放大 —— 模型的停止点误差来源于推理延迟 × 车速，
  # 与车速成正比。固定 3.0 m 的后果是「只有 ≤8 km/h 才可能通过」，
  # 实车 50+ km/h 两个红灯全程 outA=0.00、越线约 2 m。详见 StopTuning ② 段。
  tol_eff = get_stop_point_tolerance(v_ego_kph, tuning)
  stable = is_stop_point_stable(stop_pt_win, tuning.stop_pt_tol_m,
                                tuning.stop_pt_present_ratio, stable_frames,
                                tol_extra_m=tol_eff - tuning.stop_pt_tol_m,
                                hard_m=tuning.stop_pt_tol_hard_m)
  spread_raw, spread_trim = get_stop_point_spreads(
    get_decision_window(stop_pt_win, stable_frames))

  v_end_raw = float(v_traj[-1]) if v_traj.size else 0.0
  v_start = float(v_traj[0]) if v_traj.size else 0.0
  decelerating = v_end_raw < v_start * tuning.decel_ratio

  # ④ 必需减速度：正好停在停止点前所需的（留 assist_margin_m 作落脚余量）
  d_to_stop = None if stop_point is None else float(stop_point)
  required = 0.0
  if d_to_stop is not None:
    usable = max(d_to_stop - tuning.assist_margin_m, 0.5)
    required = v_ego * v_ego / (2.0 * usable)

  intends_stop = (d_to_stop is not None) and stable and decelerating

  # ── ⑥ 跟车让位（**唯一来源**，入口闸门与 STOPPING/STOPPED 的释放共用它）────
  # [第九轮 方案 A] 判据从「30 m 内一律让位」改为「前车确实在停止线之前才让位」。
  # 必须在分支之前算出来 —— 下面 stop_sign 的合取项、以及函数末尾的让位块都要用它。
  lead_yield = is_lead_yielding_stop(lead_present, d_rel, d_to_stop, tuning)
  # 日志用：让位判据实际比较到的那个量（前车距离小于它才让位）。
  # 与判据共用 get_lead_yield_threshold —— 日志和判据必须是同一个表达式，
  # 否则标定时会去调一个根本不是判据用到的量（本项目踩过这个坑两次）。
  lead_yield_ref_m = get_lead_yield_threshold(d_to_stop, tuning)

  stop_sign = False
  stop_sign_weak = False      # [第六轮] 只在「已停稳」分支可能为真，见该分支说明
  start_sign = False
  cancel_sign = False

  if v_traj.size == 0:
    pass                                    # 无模型数据：不给任何信号（窗口已记 None）
  elif v_ego_kph < 1.0:
    # ── 已停稳 ──
    # 旧版维持判据只有 `model_x_end < 20m`（「模型想停在哪」），慢速跟车时恒成立
    # -> 一直 hold -> v_target 被压成 0 -> ACC 跟不上车流。
    # 现在必须「模型确实还想停」（有停止点）才 hold。
    #
    # [第六轮 2026-09-19] 但这仍然不够 —— 车停着时模型**必然**预测自己不动，
    # `get_model_stop_point` 的「视界内已静止」路径于是把停止点算成 x[0]≈0，
    # `d_to_stop < 20 m` 在任何静止时刻都成立，与前方有没有停止线无关。
    # 实车日志实证（engage 行）：stopPoint = 0.1 / -0.0 / 0.6 / 0.7 / 0.9 / 1.0 m。
    # 于是每次停车都会新开一次停等，而释放只看「模型还给不给停止点」——
    # 停止点在 0~3 m 之间一抖就 release，下一帧又 hold，形成
    # **一秒内 4 次 engage/release** 的抖动（2026-09-19 15:12:59 实车）。
    # 修法：把「停止点太近、没有信息量」单独标成 stop_sign_weak；
    # 控制器只拿它挡**新介入**，不挡已在进行中的停等的维持
    # （维持路径继续用 stop_sign，避免把安全兜底做没了）。
    stop_sign_weak = (d_to_stop is not None) and d_to_stop < tuning.stop_point_min_hold_m
    stop_sign = (d_to_stop is not None) and d_to_stop < STOP_SIGN_HOLD_X_M
    start_sign = ((d_to_stop is not None and d_to_stop > GREEN_START_X_M)
                  or v_end_raw > MOVING_START_V_MS)
    cancel_sign = d_to_stop is None
  elif v_ego_kph < STOP_SIGN_MAX_SPEED_KPH:
    # [第九轮 方案 A] `ahead_of_lead`（旧：d_to_stop < d_rel - LEAD_CLEARANCE_M）
    # 直接换成 `not lead_yield` —— 两者本来就在说同一件事，但余量一个是 3 m、
    # 一个（让位块里的 CANCEL_LEAD_MARGIN_M）是 4 m，边界上会打架。
    # 现在只剩一个表达式，入口闸门也用它。
    stop_sign = (
      intends_stop and
      d_to_stop <= tuning.assist_max_dist_m and
      required > tuning.comfort_decel_ms2 and
      (not lead_yield) and
      abs(model_y_end) < STOP_SIGN_LATERAL_TOLERANCE_M
    )
    # ACC 已经比配置阈值更狠地在刹 -> 不叠加（防重复硬刹）。
    # 这个门只在「尚未激活」时起作用，不影响已激活的停等。
    if v_cruise != 0 and state == CRUISE and a_ego < tuning.suppress_if_decel_ms2:
      stop_sign = False
    start_sign = v_end_raw > MOVING_START_V_MS or v_end_raw > (v_start + MOVING_START_DELTA_MS)
    cancel_sign = (d_to_stop is None) or (v_end_raw > tuning.release_terminal_v_ms)
  else:
    cancel_sign = True                      # 超过检测车速上限：不介入，同时视为「明确在走」

  # ── ⑥ 跟车让位（用户 2026-09-19 实车要求 → [第九轮] 方案 A 细化）──────
  # 判据：前车**确实在停止线之前**（d_rel < d_to_stop + lead_yield_margin_m）才让位。
  # 为什么两个分支都要挡：让位时的纵向本来就该由 ACC / 驾驶模型「跟前车」处理，
  # 红灯辅助再叠一层，等于把「跟着前车」换成「跟着一条凭空造出来的虚拟停止线」。
  # 实车依据：两次抱怨的红灯，接近段 LongDecel 都是 lead=1 / dRel=5~10 m 的紧跟前车。
  #
  # 但**反过来也一样错**：前车在停止线之后时让位，等于放弃辅助、一路开到前车跟前
  # —— 这正是 >30 kph 时「停过线」的现场工况。旧版固定 30 m 门限把两者混为一谈。
  #
  # 释放路径：把 cancel_sign 置真（而不是只清 stop_sign），这样
  # 「已经 hold 住了」的情况也能在 release_confirm_ratio(0.6 s) 内干净退出，
  # 不会出现「前车进来了但模块还抱着虚拟停止线不放」。
  if lead_yield:
    stop_sign = False
    start_sign = False
    cancel_sign = True

  cancel_win.append(False if stop_sign else bool(cancel_sign))
  start_win.append(bool(start_sign and not stop_sign))

  release_frames = max(1, round(tuning.release_confirm_ratio * EVIDENCE_WINDOW_FRAMES))
  cancel_frames = sum(cancel_win)
  start_frames = sum(start_win)
  release_ready = cancel_frames >= release_frames

  if stop_sign:
    traffic_state = RED
  elif release_ready or start_frames >= START_CONFIRM_FRAMES:
    traffic_state = GREEN
  else:
    traffic_state = OFF

  debug = {
    "v_end_raw": v_end_raw,
    "v_start_raw": v_start,
    "decelerating": decelerating,
    "stop_point": d_to_stop,
    "stop_pt_abs": stop_pt_abs,
    "ego_travel_m": float(ego_travel_m),
    "stop_pt_src": sp["stop_pt_src"],
    # 注意：spread 是**绝对坐标系**下的极差（决策量），不是自车系下的原始极差。
    # 用自车系的原始极差看日志会永远很大（每 2 s 多出 ~v_ego×2 米），误导排查。
    # [第七轮] delta/delta_trim 都在**判据实际使用的窗口**（最近 stop_pt_stable_s 秒）
    # 上算，与 stable 同源 —— 否则日志里的极差和 stable 的结论对不上号，
    # 标定时会去调一个根本不是判据用到量的参数（这正是固定容差 3.0 全军覆没
    # 却没被看出来的原因：日志打的是 3 s 窗口的极差，判据早期用的是别的窗口）。
    # win_n/win_len 仍然是**整个** deque 的计数（那两条是「占比」判据用的）。
    "stop_pt_delta": spread_raw,
    "stop_pt_delta_trim": spread_trim,
    "stop_pt_tol_eff": tol_eff,
    "stop_pt_win_n": len(present),
    "stop_pt_win_len": len(stop_pt_win),
    "stop_pt_stable_frames": stable_frames,
    "stop_pt_stable": stable,
    "a_tail": sp["a_tail"],
    "extrap_span_s": sp["extrap_span_s"],
    "standstill_span": sp["standstill_span"],
    "intends_stop": intends_stop,
    "d_to_stop": d_to_stop,
    "required_decel": required,
    "comfort_decel": tuning.comfort_decel_ms2,
    "assist_max_dist": tuning.assist_max_dist_m,
    "stop_sign": stop_sign,
    "stop_sign_weak": stop_sign_weak,
    "stop_point_min_hold": tuning.stop_point_min_hold_m,
    "start_sign": start_sign,
    "cancel_sign": cancel_sign,
    "cancel_frames": cancel_frames,
    "start_frames": start_frames,
    "release_frames": release_frames,
    "release_ready": release_ready,
    # [第九轮] 键名仍是 lead_block（swaglog 解析脚本与既有日志对照都依赖它），
    # 语义已变为「前车确实在停止线之前 -> 让位」。yRef 是它比较到的那个量。
    "lead_block": lead_yield,
    "lead_yield_ref_m": lead_yield_ref_m,
    "lead_yield_margin": tuning.lead_yield_margin_m,
    "lead_close_yield_m": LEAD_CLOSE_YIELD_M,
    "lead_suppress_dist": tuning.lead_suppress_dist_m,
  }
  return traffic_state, debug

class TrafficStopController:
  """有状态控制器。planner 每个周期（20 Hz）调用一次 update()。"""

  def __init__(self):
    self.is_enabled, self.distance_adjust_m = _load_config()
    self.tuning = _load_tuning()
    self._poll_frame = 0
    # [第五轮] 累计里程（绝对坐标系原点），用于把「模型的停止点」从自车系换算到绝对系。
    # 只在 _reset() 里归零 —— 归零时窗口也同时清空，两者始终同步。
    self._ego_travel_m = 0.0

    self.state = CRUISE
    self._prev_state = CRUISE

    # 三个证据窗口：[第三轮] stop_pt_win 记「模型给出的停止位置」，
    # 另外两个记「模型明确在走 / 想起步」。用滚动窗口而不是连续计数 ——
    # 模型输出实测会单帧抖动，连续计数一抖即归零、真红灯前反而失效。
    self.stop_pt_win: deque = deque(maxlen=STOP_PT_WINDOW_FRAMES)
    self.start_win: deque = deque(maxlen=EVIDENCE_WINDOW_FRAMES)
    self.cancel_win: deque = deque(maxlen=EVIDENCE_WINDOW_FRAMES)
    self.debug: dict = {}
    self.stop_sign_count = 0        # 窗口内「要停」帧数（日志用）
    self.start_sign_count = 0       # 窗口内「要走」帧数（日志用）
    self.cancel_sign_count = 0      # 窗口内「在走」帧数（日志/释放用）
    self.model_v_hist: deque = deque(maxlen=MODEL_V_WINDOW)   # 仅探针日志用

    # median(3) → moving-average(15)，跨停等事件不清空（沿用上游/cp 8.9 的设计）
    self._median_hist: deque = deque(maxlen=MEDIAN_WINDOW)
    self._avg_hist: deque = deque(maxlen=MOVING_AVG_WINDOW)
    self.stop_model_x_raw = 0.0
    self.stop_model_x_rl = 0.0

    self.reference_speed_kph = 0.0
    self.actual_stop_distance = 0.0
    self.gas_suppress_frames = 0
    self.stopped_grace_frames = 0
    self.stopped_frames = 0
    self._timeout_logged = False

    # [第六轮 2026-09-19] 抖动抑制（见 StopTuning ⑦ / check_model_stopping 的
    # 「已停稳」分支说明）。实车实证：一秒内出现过 4 次 engage/release。
    self._active_frames = 0            # 本次停等已持续的帧数
    self._reengage_block_frames = 0    # 短命停等后的「禁止再介入」剩余帧数
    # [第九轮 方案 A] 上一次退出的原因。'lead' 的退出**不计入**短命停等封锁 ——
    # 前车让位是「交还给常规跟车」的正常移交，不是抖动；把它算成抖动会让模块
    # 在前车驶离后被封锁 6 s，白白错过真正该介入的红灯。
    self._release_reason: str | None = None

    self.stop_dist_m: float | None = None
    self.output_v_target = V_CRUISE_MAX
    self.output_a_target = 0.0

    self._log_ts = -1e9
    self._probe_ts = -1e9

  # ── 属性 ──────────────────────────────────────────────────────────────
  @property
  def is_active(self) -> bool:
    """是否正在主动管理一次停等（外层 planner 用它决定要不要加候选）。"""
    return self.state != CRUISE

  # ── 内部 ──────────────────────────────────────────────────────────────
  def _poll_config(self):
    self._poll_frame += 1
    if self._poll_frame >= CONFIG_POLL_FRAMES:
      self._poll_frame = 0
      self.is_enabled, self.distance_adjust_m = _load_config()
      self.tuning = _load_tuning()      # 阈值同样 1 s 热生效，方便路测标定

  def _deactivate(self):
    """[P0-1] 唯一的释放路径。任何「不再主动停等」的场合都必须经此，
    否则 MPC 层会继续拿着陈旧的 stop_dist_m 当硬约束（幽灵障碍物）。"""
    self.stop_dist_m = None
    self.output_v_target = V_CRUISE_MAX
    self.output_a_target = 0.0

  def _reset(self):
    self.state = CRUISE
    self.stop_pt_win.clear()
    self.start_win.clear()
    self.cancel_win.clear()
    self._ego_travel_m = 0.0          # [第五轮] 与 stop_pt_win 同生共死
    self.actual_stop_distance = 0.0
    self.reference_speed_kph = 0.0
    self.stopped_grace_frames = 0
    self.stopped_frames = 0
    self._timeout_logged = False
    self._active_frames = 0
    self._reengage_block_frames = 0
    self._release_reason = None
    self._deactivate()

  def _update_stop_model_x(self, raw_x: float, v_ego: float) -> tuple[float, float]:
    """median(3) → moving-average(15) → 单向速率限制（只限制「拉近」，放行「后退」）。"""
    self._median_hist.append(raw_x)
    median_val = float(np.median(self._median_hist))
    self._avg_hist.append(median_val)
    stop_model_x_raw = float(np.mean(self._avg_hist))

    max_step = v_ego * DT_MDL + RATE_LIMIT_CLOSING_MARGIN_M
    if stop_model_x_raw < self.stop_model_x_rl:
      stop_model_x_rl = max(stop_model_x_raw, self.stop_model_x_rl - max_step)
    else:
      stop_model_x_rl = stop_model_x_raw        # 后退：不限速
    return stop_model_x_raw, stop_model_x_rl

  def _log(self, traffic_state: int, v_ego: float, v_cruise: float,
           d_rel: float, lead_present: bool):
    """[C] 量化日志：状态/信号/各段距离/输出。1 Hz 节流。"""
    now = time.monotonic()
    if now - self._log_ts < LOG_INTERVAL_S:
      return
    self._log_ts = now
    d = self.debug
    sp = d.get("stop_point")
    cloudlog.info(
      f"[TrafficStop] {_STATE_NAME[self.state]}/{_SIGNAL_NAME[traffic_state]} "
      f"stopDist={0.0 if self.stop_dist_m is None else self.stop_dist_m:.1f}m "
      f"raw={self.stop_model_x_raw:.1f} rl={self.stop_model_x_rl:.1f} "
      f"adv={self.actual_stop_distance:.1f} refV={self.reference_speed_kph:.0f}kph "
      f"| vEgo={v_ego * 3.6:.1f} vCruise={v_cruise * 3.6:.1f} "
      f"| stopPt={'--' if sp is None else f'{sp:.1f}'} "
      f"src={d.get('stop_pt_src', '-')} "
      f"stable={int(bool(d.get('stop_pt_stable')))} "
      f"({d.get('stop_pt_win_n', 0)}/{d.get('stop_pt_win_len', 0)}"
      f" spread={d.get('stop_pt_delta', 0.0):.1f}"
      f" trim={d.get('stop_pt_delta_trim', 0.0):.1f}"
      f" tol={d.get('stop_pt_tol_eff', 0.0):.1f}) "
      f"req={d.get('required_decel', 0.0):.2f}/{d.get('comfort_decel', 0.0):.1f} "
      f"| vEnd={d.get('v_end_raw', 0.0) * 3.6:.1f}kph "
      f"cancelEv={self.cancel_sign_count}/{d.get('release_frames', 0)} "
      f"| lead={int(lead_present)} dRel={d_rel:.1f} "
      f"blk={int(bool(d.get('lead_block')))} yRef={d.get('lead_yield_ref_m', 0.0):.1f} "
      f"reBlk={self._reengage_block_frames} rel={self._release_reason or '-'} "
      f"| outV={self.output_v_target:.2f} outA={self.output_a_target:.2f} "
      f"gasSup={self.gas_suppress_frames} grace={self.stopped_grace_frames} "
      f"weak={int(bool(d.get('stop_sign_weak')))}"
    )

  def _probe(self, traffic_state: int, v_ego: float, a_ego: float, v_cruise: float,
             model_v_traj, model_x_end: float, model_y_end: float,
             d_rel: float, lead_present: bool, steering_angle_deg: float):
    """[标定探针] 只在「疑似停意图」附近输出，用来给阈值标定提供真实数据。

    为什么需要：stop_pt_tol_m / comfort_decel_ms2 / min_decel_ms2 这些
    「模型预测要停」的量化定义，取决于具体模型的轨迹末端行为。本机跑的是
    TSFDOM (recompiled20)，它的停止点稳定性没有公开基准，
    **只能靠实车数据标定，不能靠猜**。

    触发条件（任一）：
      - 本帧 stop_sign 成立（真正的候选样本）
      - 本帧已经有停止点（判据边界附近的「边缘样本」，正是标定最需要的数据）
      - 模块处于激活状态
      - [第五轮] a_ego < -0.2：正在减速 —— 接近红灯的**减速段全程在 CRUISE**，
        如果只留前三条，这段就一条日志都没有（实车就是这么盲的）。
        巡航（基本匀速）时不打，避免把 swaglog 刷满。

    1 Hz 节流，避免刷爆 swaglog（swaglog 最低收 INFO，cloudlog.debug 不落盘）。
    """
    if not self.tuning.probe:
      return
    now = time.monotonic()
    if now - self._probe_ts < LOG_INTERVAL_S:
      return

    v_end_raw = float(np.asarray(model_v_traj, dtype=float)[-1]) if len(model_v_traj) else 0.0
    d = self.debug
    if not (d.get("stop_point") is not None or d.get("stop_sign")
            or self.state != CRUISE or a_ego < -0.2):
      return
    self._probe_ts = now

    sp = d.get("stop_point")
    v_smooth = float(np.mean(self.model_v_hist)) if len(self.model_v_hist) else v_end_raw
    cloudlog.info(
      f"[TrafficStopProbe] vEgo={v_ego * 3.6:.1f} aEgo={a_ego:.2f} vCruise={v_cruise * 3.6:.1f} "
      f"| vEnd={v_end_raw * 3.6:.1f} vEndSm={v_smooth * 3.6:.1f} "
      f"vStart={d.get('v_start_raw', 0.0) * 3.6:.1f} "
      f"stopPt={'--' if sp is None else f'{sp:.1f}'} src={d.get('stop_pt_src', '-')} "
      f"stable={int(bool(d.get('stop_pt_stable')))} "
      f"({d.get('stop_pt_win_n', 0)}/{d.get('stop_pt_win_len', 0)}"
      f" spread={d.get('stop_pt_delta', 0.0):.1f}"
      f" trim={d.get('stop_pt_delta_trim', 0.0):.1f}"
      f" tol={d.get('stop_pt_tol_eff', 0.0):.1f}) "
      f"req={d.get('required_decel', 0.0):.2f} aTail={d.get('a_tail', 0.0):.2f} "
      f"xEnd={model_x_end:.1f} yEnd={model_y_end:.2f} "
      f"| stop={int(bool(d.get('stop_sign')))} cancel={int(bool(d.get('cancel_sign')))} "
      f"cancelEv={self.cancel_sign_count}/{d.get('release_frames', 0)} "
      f"| lead={int(lead_present)} dRel={d_rel:.1f} steer={steering_angle_deg:.1f} "
      f"yRef={d.get('lead_yield_ref_m', 0.0):.1f} blk={int(bool(d.get('lead_block')))} "
      f"| state={_STATE_NAME[self.state]} sig={_SIGNAL_NAME[traffic_state]} "
      f"stopDist={0.0 if self.stop_dist_m is None else self.stop_dist_m:.1f} "
      f"outA={self.output_a_target:.2f}"
    )

  # ── 主循环 ────────────────────────────────────────────────────────────
  def update(self, model_v2, car_state, radar_state, v_ego: float, a_ego: float, v_cruise: float):
    self._poll_config()
    # [第五轮] 累计里程每帧都推 —— 即使本帧早退（禁用/模型数据不全）也要保持
    # 与 stop_pt_win 里的绝对坐标一致，否则窗口里的历史值与新值的坐标系会错开。
    self._ego_travel_m += v_ego * DT_MDL

    if not self.is_enabled:
      if self.state != CRUISE:
        cloudlog.info("[TrafficStop] disabled -> release")
      self._reset()
      return

    model_x_traj = model_v2.position.x
    model_y_traj = model_v2.position.y
    model_v_traj = model_v2.velocity.x
    if len(model_x_traj) < 2 or len(model_v_traj) == 0:
      # 模型数据不完整：只释放障碍物，不强改状态（上游这里也没清 stop_dist_m —— P0-1）
      self._deactivate()
      return

    model_x_end = model_x_traj[-1]

    lead = radar_state.leadOne
    lead_present = bool(lead.present)          # 新版字段是 present（上游是 status）
    d_rel = lead.dRel if lead_present else 1000.0

    steering_angle_deg = car_state.steeringAngleDeg
    gas_pressed = car_state.gasPressed
    blinker_on = bool(car_state.leftBlinker or car_state.rightBlinker)   # [P2-8] 左右对称

    traffic_state, self.debug = check_model_stopping(
      self.stop_pt_win, self.start_win, self.cancel_win, self.state,
      v_cruise, model_v_traj, model_x_traj, v_ego, a_ego, model_y_traj[-1], d_rel,
      lead_present, self.tuning, self._ego_travel_m)
    self.cancel_sign_count = self.debug["cancel_frames"]
    self.start_sign_count = self.debug["start_frames"]
    self.stop_sign_count = 1 if self.debug["stop_sign"] else 0

    # 障碍物的锚点 = 模型的**停止点**（不是轨迹末点）。
    # 高速时 10 s 视界的末点还在半路上，用末点当锚点会把停止线摆得太远/太近；
    # 停止点才是「模型打算停在哪」。模型这帧没给停止点时保持上一帧的平滑值，
    # 避免把 None 喂进 median/average 过滤器（释放由状态机负责，不靠这里）。
    stop_point = self.debug.get("stop_point")
    if stop_point is not None:
      self.stop_model_x_raw, self.stop_model_x_rl = self._update_stop_model_x(float(stop_point), v_ego)
    self.model_v_hist.append(float(model_v_traj[-1]))

    # ── 油门抑制 ──
    # [第十轮 2026-09-20] 原条件是 `state == STOPPING` —— 与 cp / dp 原型逐字相同，
    # 缺口出在 **STOPPED**：车已停稳时踩油门，状态机把它释放回 CRUISE（见下方
    # STOPPED 分支），但抑制窗口仍然是 0 ⇒ 松油门后立刻重新满足入口条件。
    # 离线复现（verify 的 S 段）：松油门后第 38 帧（1.9 s）就 re-engage，
    # 实车表现就是用户 2026-09-20 的体感「踩油门也立马刹车，要一直踩着油门才肯走」。
    # 加上 STOPPED 之后，「踩油门」的语义统一为「驾驶员接管 10 s」，
    # 不再区分当时是正在接近（STOPPING）还是已经停稳（STOPPED）。
    if gas_pressed and self.state in (STOPPING, STOPPED):
      self.gas_suppress_frames = GAS_SUPPRESS_FRAMES
    elif self.gas_suppress_frames > 0:
      self.gas_suppress_frames -= 1

    # [第九轮 方案 A] 前车让位判据 —— **来自判据函数本身**（debug["lead_block"]），
    # 不再在这里另写一套表达式。旧版这里是 `lead_present and (d_rel -
    # stop_model_x_raw) < CANCEL_LEAD_MARGIN_M`（4 m 门限），而判据里是 30 m 距离
    # 门限、stop_sign 里还有第三套 3 m 门限（ahead_of_lead）—— 三套并存必然分裂。
    lead_yield = bool(self.debug.get("lead_block", False))
    # 前车已经压进来（< LEAD_CLOSE_YIELD_M）-> 不等 0.6 s 窗口，立即交还。
    lead_close_in = is_lead_close_in(lead_present, d_rel)

    # ── [第六轮] 抖动抑制的帧计数 ────────────────────────────────────────
    # 放在状态机之前：本帧的进入判据要用到 _reengage_block_frames 的最新值。
    was_active = self.state != CRUISE
    if self._reengage_block_frames > 0:
      self._reengage_block_frames -= 1
    if was_active:
      self._active_frames += 1

    # ── 状态转移 ──
    if self.state == CRUISE:
      entry_allowed = is_traffic_stop_entry_allowed(steering_angle_deg)
      # traffic_state == RED 现在**已经蕴含**「停止点连续稳定 stop_pt_stable_s 秒」
      # （见 check_model_stopping 的稳定性判据），所以这里不需要再叠加确认窗口。
      #
      # [第五轮] 入口门限由写死的 `not lead_present` 改为与判据里 ⑥ 一致的距离门限。
      #   旧写法是「只要雷达看到**任何距离**的前车就永远进不来」—— 与用户要求的
      #   「30 m 内有前车才不触发」不一致：80 m 外的前车也会把红灯辅助整个挡掉，
      #   而红灯明明就在前方。两处必须用同一个量，否则会出现
      #   「判据说可以、状态机说不行」的分裂（本次离线虚拟行车 M5 复现）。
      #
      # [第六轮] 再叠两条「不要新介入」的理由：
      #   stop_sign_weak      —— 已停稳且停止点没有信息量（见该分支说明）
      #   _reengage_block_frames —— 刚结束一次短命停等，正在防抖封锁期
      #
      # [第九轮 方案 A] 入口闸门与判据**统一**：直接用判据算出来的 lead_yield，
      # 不再另写 `(not lead_present) or (d_rel > lead_suppress_dist_m)`。
      # 旧写法的问题：判据（`ahead_of_lead`，3 m 余量）与闸门（30 m 距离）
      # 是两个不同的表达式，必然出现「判据说 RED、闸门说不行」的分裂 ——
      # 而且旧闸门在「停止线 12 m、前车 25 m」时会让位，辅助整个消失（停过线）。
      # [第十轮 2026-09-20] 入口再加一条 `not gas_pressed`：
      # 驾驶员的脚还在油门上时，**任何情况下都不新介入** —— 不要等 10 s 抑制窗口
      # 那一层间接保护（那一层只在踩下去的那一帧武装，中间松一下就没保护了）。
      # 语义直白：油门是驾驶员更强的接管意图，模块没有资格在同一时刻踩刹车。
      if (not gas_pressed and not lead_yield and traffic_state == RED and entry_allowed
          and self.gas_suppress_frames == 0
          and not self.debug.get("stop_sign_weak", False)
          and self._reengage_block_frames == 0):
        self.state = STOPPING
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, None)
        self.actual_stop_distance = get_virtual_traffic_stop_distance(
          self.stop_model_x_rl, self.reference_speed_kph)
        self._timeout_logged = False
        d = self.debug
        cloudlog.info(f"[TrafficStop] engage: vEgo={v_ego * 3.6:.1f}kph "
                      f"stopPoint={self.stop_model_x_rl:.1f}m src={d.get('stop_pt_src')} "
                      f"spread={d.get('stop_pt_delta', 0.0):.1f}m "
                      f"required={d.get('required_decel', 0.0):.2f}m/s2 "
                      f"vEnd={model_v_traj[-1] * 3.6:.1f}kph")

    elif self.state == STOPPING:
      # [本次修正] 释放条件里加入「模型明确预测车还在走」。
      # 上游/首版只能靠 GREEN（要求末速 > 5 m/s）退出 —— 门槛过高，
      # 误触发后进得来出不去，于是「只需要减速」的场合被一路刹到停。
      #
      # [第九轮] 前车让位**默认不在这里立即放行**，而是照旧走 0.6 s 取消窗口
      # （让位块会把 cancel_sign 置真 -> release_ready）。理由：
      #   边界本来就有抖动 —— d_rel 与 d_to_stop 齐平时两种判定的差别在亚米级，
      #   窗口天然提供去抖；直接把让位做成立即释放会把边界工况变成 1 帧抖动释放。
      # 唯一例外是「前车已经压进 10 m」（lead_close_in）：那一刻跟随前车是唯一
      # 合理做法，等 0.6 s 没有意义。旧版靠另一套 lead_cancels 做到这一点，
      # 现在由与让位判据同源的 is_lead_close_in 承担。
      if (gas_pressed or lead_close_in or traffic_state == GREEN
          or self.debug["release_ready"]):
        self._release_reason = ("gas" if gas_pressed else "lead" if lead_yield
                                else "green" if traffic_state == GREEN else "model-moving")
        cloudlog.info(f"[TrafficStop] release: vEgo={v_ego * 3.6:.1f}kph "
                      f"reason={self._release_reason} "
                      f"cancelEvidence={self.cancel_sign_count}/{self.debug['release_frames']} "
                      f"leadYieldRef={self.debug.get('lead_yield_ref_m', 0.0):.1f}m")
        self.state = CRUISE
      else:
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, self.reference_speed_kph)
        candidate = get_virtual_traffic_stop_distance(self.stop_model_x_rl, self.reference_speed_kph)
        if candidate > RECALIBRATE_MIN_DISTANCE_M:
          self.actual_stop_distance = candidate
        if v_ego < STOPPED_SPEED_MS:
          self.state = STOPPED
          self.stopped_grace_frames = STOPPED_GRACE_FRAMES
          self.stopped_frames = 0

    elif self.state == STOPPED:
      self.stopped_frames += 1
      # [第九轮] 停稳后前车压到停止线附近（lead_yield）-> 立即交还跟车。
      # 这里保留「立即」而不走 0.6 s 窗口：车已经停着，前车在 4 m 内时
      # 应当马上把纵向交出去（安全侧），窗口只是延时。
      if gas_pressed or lead_yield:
        self._release_reason = "gas" if gas_pressed else "lead"
        self.state = CRUISE
      else:
        if self.stopped_grace_frames == 0 and not blinker_on and (
            traffic_state == GREEN or self.debug["release_ready"]):
          # 两条释放路径：GREEN（模型预测强加速，0.2 s）与 cancel（模型不再预测
          # 静止，0.6 s）。后者专治慢速跟车：见 check_model_stopping 的静止分支。
          self._release_reason = "green" if traffic_state == GREEN else "model-moving"
          self.state = CRUISE
        self.stopped_grace_frames = max(0, self.stopped_grace_frames - 1)
        # [P2-9] 超时只提示、不自动放行：把「等不到绿灯」的安全兜底留给驾驶员
        if (not self._timeout_logged and
            self.stopped_frames * DT_MDL > STOPPED_TIMEOUT_LOG_S and traffic_state != GREEN):
          self._timeout_logged = True
          cloudlog.info(
            f"[TrafficStop] 已停在停止线 {self.stopped_frames * DT_MDL:.0f}s，模型仍未给出起步意图"
            f"（stopDist={self.stop_dist_m if self.stop_dist_m is None else round(self.stop_dist_m, 1)}m，"
            f"踩油门即可退出）"
          )

    # ── [第六轮] 停等结束：短命停等 -> 封锁再介入 ────────────────────────
    # 「不停地自动刹车」的直接对策。实车证据（2026-09-19）：
    #   15:12:59 一秒内 4 次 engage/release，release 原因全是 model-moving，
    #   每次 engage 的 stopPoint 都是 -0.0 m —— 停止点在 0 附近抖动，
    #   而释放判据（cancel_sign）与进入判据（stop_sign）用的是同一个振荡源。
    # 只封锁「短命」停等：真红灯停够 stop_reengage_min_episode_s 秒后正常释放，
    # 不影响下一个路口的介入时机。
    #
    # [第九轮 方案 A] **前车让位（reason='lead'）不计入封锁**。
    # 理由：让位是「交还给常规跟车」的正常移交，不是抖动 —— 它由前车位置决定，
    # 前车驶离后这个理由立刻消失，不存在「同一振荡源来回触发」的问题。
    # 反过来把它算成抖动，会让模块在前车驶离后被封锁 6 s，直接错过真正该介入
    # 的红灯（高速段 6 s × 14 m/s ≈ 84 m 的盲区，正是「停过线」的放大器）。
    # 防抖在这里由**入口闸门自己**承担：只要前车还在停止线之前，
    # 闸门（not lead_yield）就持续挡住再介入，不需要额外的封锁计时。
    if was_active and self.state == CRUISE:
      episode_s = self._active_frames * DT_MDL
      if episode_s < self.tuning.stop_reengage_min_episode_s and self._release_reason != "lead":
        self._reengage_block_frames = max(1, int(round(
          self.tuning.stop_reengage_cooldown_s / DT_MDL)))
        cloudlog.info(
          f"[TrafficStop] 短命停等 {episode_s:.1f}s（reason={self._release_reason}）"
          f" -> 封锁再介入 {self.tuning.stop_reengage_cooldown_s:.0f}s（防抖，见 StopTuning ⑦）")
      self._active_frames = 0

    # ── 状态跳变日志 ──
    if self.state != self._prev_state:
      cloudlog.info(f"[TrafficStop] state {_STATE_NAME[self._prev_state]} -> {_STATE_NAME[self.state]} "
                    f"(vEgo={v_ego * 3.6:.1f}kph signal={_SIGNAL_NAME[traffic_state]} "
                    f"raw={self.stop_model_x_raw:.1f}m)")
      self._prev_state = self.state

    # ── 完全释放 ──
    if self.state == CRUISE:
      self.actual_stop_distance = 0.0
      self.reference_speed_kph = 0.0
      self.stopped_grace_frames = 0
      self.stopped_frames = 0
      self._timeout_logged = False
      self._deactivate()
      # [第五轮] 日志/探针必须**放在 return 之前**。
      # 旧版它们在 return 之后，而 CRUISE 覆盖了「默认状态 + 整个接近段」——
      # 于是最需要数据的那一段反而零日志：实车两次「过停止线没辅助刹车」，
      # swaglog 里 [TrafficStop] 与 [TrafficStopProbe] 一条都没有，
      # 导致排查时只能靠读代码猜（上一轮就把「零探针」误读成「没有停止点」，
      # 真正的机制其实是坐标系问题）。诊断代码本身也要被测到，见验证脚本 M6。
      if self.debug.get("stop_point") is not None or a_ego < -0.2:
        self._log(traffic_state, v_ego, v_cruise, d_rel, lead_present)
      self._probe(traffic_state, v_ego, a_ego, v_cruise, model_v_traj, model_x_end,
                  model_y_traj[-1], d_rel, lead_present, steering_angle_deg)
      return

    # ── 停等流程中：障碍物始终有效（P0-1 的核心修正）──
    # 无论信号是 RED / OFF / GREEN，只要还没转回 CRUISE，就继续用航位推算推进距离，
    # 避免「信号一帧读不出来就把障碍物丢掉」导致车在停止线前被 ACC 拽着走。
    self.actual_stop_distance = max(0.0, self.actual_stop_distance - v_ego * DT_MDL)

    # [P1-5] 这里**不再**做上游的 "force sync"（上游在主动刹车路径上每帧
    #   self.stop_model_x_rl = self.stop_model_x_raw
    # 把 _update_stop_model_x 里的速率限制整个绕过，模型距离瞬间跳近会直接穿透、
    # 造成额外重刹）。rate limit 的步长是 v*dt + 0.5，恒大于实际接近量 v*dt，
    # 所以正常接近完全不受限，它只挡抖动。

    contribution = 0.0 if self.actual_stop_distance > 0.0 else self.stop_model_x_rl
    stop_dist = max(0.0, contribution + self.actual_stop_distance)
    # [P1-3] 补偿 MPC 的领航裕度，让最终停车点回到「模型预测的停止线」
    stop_dist = get_traffic_stop_obstacle_distance(
      stop_dist + MPC_STOP_MARGIN_COMP_M, CAMERA_TO_FRONT_M + self.distance_adjust_m)
    self.stop_dist_m = stop_dist

    if self.state == STOPPED:
      self.output_v_target = 0.0
      self.output_a_target = 0.0
    else:
      # [本次修正] 由「恒定 COMFORT_BRAKE」改为「停在障碍物前所需的减速度」。
      #
      # 旧版无条件给 -2.16 m/s²。外层 planner 用 min(candidates, key=a_target)
      # 选候选，所以只要 is_active，这个恒定值就**必然**被选中 —— 哪怕模型只是
      # 轻抬油门，也会被按 2.16 m/s² 刹，误触发的代价直接是「刹停」。
      #
      # 现在按需反解：a = v² / 2d。60 m 外只需 ~0.6 m/s²，与 MPC 自己在做的事情
      # 量级一致，选不选中都不改变行为；只有真的该刹时才会逼近上限。
      # [第三轮] 上限按车速分两档：高速红灯的物理需求本来就更硬
      #   （50 km/h 停 48 m 需要 2.0 m/s²、停 30 m 需要 3.2 m/s²），
      #   常规 2.16 在高速上会把「正好停住」变成「刚好过线」。
      # MPC（带障碍物）自己做主时不受此限。
      margin = self.tuning.assist_margin_m
      brake_cap = self.tuning.max_brake_ms2
      if v_ego * 3.6 >= self.tuning.high_speed_kph:
        brake_cap = max(brake_cap, self.tuning.high_speed_brake_ms2)
      usable = max(stop_dist - margin, 1.0)
      required = (v_ego * v_ego) / (2.0 * usable)
      self.output_a_target = -min(brake_cap, required)

      # v_target 只承担「软限速」：给的是「还能舒适停下的最高车速」。
      # 若这个速度已经高于当前车速，说明还轮不到我们表态 —— 返回 V_CRUISE_MAX
      # （= 没有意见）。不能写成 min(v_limited, v_ego)：那样等于「不许加速」，
      # 在一次误触发里会白白压住本该进行的加速。
      v_limited = (2 * self.tuning.comfort_decel_ms2 * max(stop_dist - margin, 0.0)) ** 0.5
      self.output_v_target = v_limited if v_limited < v_ego else V_CRUISE_MAX

    self._log(traffic_state, v_ego, v_cruise, d_rel, lead_present)
    self._probe(traffic_state, v_ego, a_ego, v_cruise, model_v_traj, model_x_end,
                model_y_traj[-1], d_rel, lead_present, steering_angle_deg)
