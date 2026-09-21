#!/usr/bin/env python3
import math
import time
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.turn_decel import TurnDecelController
from openpilot.sunnypilot.selfdrive.controls.lib.coast_resume import CoastResumeController

#A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
#A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MAX_BP = [0., 2.77, 5.55, 8.33, 11.11, 13.89, 16.6, 19.4, 22.22, 25, 27.78, 33.33]
A_CRUISE_MAX_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
J_CRUISE_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
# [2026-09-20] allow_throttle 迟滞帧数（20 Hz，5 帧 = 0.25 s）。见 update() 里的说明。
# 设 0 = 无迟滞（原行为）。
ALLOW_THROTTLE_HYST_FRAMES = 5

# 实验模式(end-to-end)正加速度上限（m/s²）。
# 非 e2e 时纵向走 A_CRUISE_MAX_VALS 表（低速封顶 1.10 m/s²），而 e2e 时
# get_cruise_accel() 直接把 max_accel 放成 ACCEL_MAX(≈2.0)，模型链路的
# e2e / MPC 候选也能给到 ACCEL_MAX —— 换成大模型后纵向更激进，体感偏猛。
# 这里给 e2e 路径单独加一道正加速度上限：
#   调小 = 更保守；设为 None = 完全恢复原始行为（不限制）
# 实车调参：1.4 仍偏猛 -> 1.2 -> **1.6**（用户 2026-09-19 反馈「有点肉」，往回收一点）
# 还肉继续升到 1.8；猛了回 1.2
E2E_ACCEL_MAX_M_S2: float | None = 1.6

# ---- 超速滑行回落（仿 dragonpilot 的 ACM / Adaptive Coasting Module）----
# 现象：设定 60 km/h，驾驶员踩油门把车推到 70，松开油门后旧逻辑会立刻用
#       A_CRUISE_MIN(-1.2 m/s²) 恒定硬刹回 60，体感像是被点了一脚刹车。
# 原因：cruise 候选项恒为 np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)，
#       只要超速超过 4.3 km/h 就直接饱和到 -1.2。
# 改为：按超速幅度在「自然滑行」与「全力减速」之间线性过渡 —— 小超速让车顺着
#       阻力滑回设定速度（平路约 -0.3 m/s²，从 70 回到 60 约 9 秒），大超速再
#       逐步加大刹车力度，兼顾舒适与安全。
# 安全：只改 cruise 候选项的**下限**，候选集依然取 min()，所以 MPC / e2e / FCW
#       给出的更保守制动（跟车、前车急刹、视觉风险）完全不受影响。
#   调参：嫌回落慢 -> 调小 COAST_OVERSPEED_PURE_KPH / FULL_KPH；嫌回落猛 -> 调大；
#         COAST_OVERSPEED_ENABLE = False 一键回退成原行为（恒 -1.2）。
COAST_OVERSPEED_ENABLE = True
COAST_OVERSPEED_PURE_KPH = 10.0   # 超速 <= 10 km/h：纯滑行（accel_coast，平路约 -0.3 m/s²）
COAST_OVERSPEED_FULL_KPH = 30.0   # 超速 >= 30 km/h：回到全力 A_CRUISE_MIN(-1.2 m/s²)
# 日志迟滞（纯降噪，不参与控制）：overspeed 恰好 ≈ 0 时 `v_ego > v_cruise` 会 20 Hz 翻转，
# 实车 2026-09-20 12:34 一分钟打了 110 条 engaged/released，把真信号全淹了。
# 控制侧不受影响：减速度下限走的是 get_cruise_accel_min() 的连续插值。
# 设 0.0 = 无迟滞（原行为）。
COAST_OVERSPEED_LOG_HYST_KPH = 0.5

# ---- a_cruise 负值恢复速率（2026-09-20 新增，解决「加速意愿不足 / 踩油门刹车」）----
# 现象（实车 2026-09-20 12:43:01 抓到原文）：
#   [LongDecel] aTarget=-1.12 src=0 | aMpc=0.53 aCruise=-1.12 | vEgo=48 vCruiseUI=58 |
#               lead=0 prob=0.00 radar=0        ← 前方空无一物、车速比设定低 10km/h，
#                                                 MPC 想给 +0.53，实际却在 -1.12 刹车
# 原因：a_cruise 之前被压到过 ≈ -1.1（SLA/SCC 短暂把目标速度降到 ~20km/h，
#   实车 12:39:02 就抓到 vCruiseUI=59 / vCruiseInt=26 / spSrc=1），而
#   J_CRUISE_VALS 在 50km/h 只有 **0.40 m/s³** —— 从 -1.2 爬回 0 要**整整 3 秒**。
#   这 3 秒里 cruise 候选一直是负的，min(candidates) 选中它就压住了加速意图：
#   体感就是「加速意愿不足」；目标速度若反复被压/放开，就是「踩油门刹车」。
#   注：这档现象与红灯模块、雷达都无关 —— 解释了「关掉红灯辅助后减轻但没消失」。
# 改法：只在 a_cruise **当前为负**（陷在刹车残值里）时，把**向上**的 jerk 上限提到此值；
#   向下（更负）仍走原 J_CRUISE_VALS，所以任何真正的减速需求都不受影响。
# 安全性：cruise 候选最终仍与 MPC / e2e / traffic_stop / turn_decel 一起取 min()，
#   把它抬快只会让它更早「输给」更保守的一方，**不可能放松任何制动**。
#   -1.2 → 0 的恢复时间：3.0 s（原）-> 0.6 s（2.0）-> 0.4 s（3.0）
# 实车核对：grep CruiseStale /data/log/swaglog.*（burst 时长应从 ~3s 掉到 <1s）
# 设 None = 完全恢复原行为（上下都用 J_CRUISE_VALS）。
CRUISE_RECOVER_J_MS3: float | None = 2.0

# ---- 「巡航候选残留负值」探针（诊断用，2026-09-20 新增）----
# 判据：**该加速却给了负号** —— (v_cruise - v_ego) > CRUISE_STALE_MIN_GAP_MS（离设定
#       还有 7+ km/h）且 a_cruise < CRUISE_STALE_MAX_A。命中即说明 a_cruise 正被自身
#       的 jerk 恢复速率卡着（见 CRUISE_RECOVER_J_MS3）。这是上面那个 bug 的
#       **直接观测量**，也是验证本次修复的验收指标。节流 CRUISE_STALE_PROBE_INTERVAL。
CRUISE_STALE_PROBE_ENABLE = True
CRUISE_STALE_PROBE_INTERVAL = 1.0
CRUISE_STALE_MIN_GAP_MS = 2.0     # v_cruise - v_ego 至少要差这么多才算「该加速」
CRUISE_STALE_MAX_A = -0.2         # a_cruise 低于此值才算「带着刹车残值」

# ---- 大减速现场记录器（诊断用，2026-09-18 新增）----
# 目的：一趟车就能回答「这一脚到底为什么刹」。输出加速度低于阈值时，把当时的
# 各条候选值 / 胜出来源 / 设定速度 / 前车详情一次性打进 swaglog。
#
# 关键的两个对照量（这条日志的价值所在）：
#   vCruiseUI vs vCruiseInt —— UI 上的设定速度 vs SP 内部实际跟踪的目标速度。
#     两者不一致 = Smart Cruise Control(Vision/Map) 或 Speed Limit Assist 在悄悄
#     压速，与车主拧的设定速度无关。
#   lead=1 + vLeadK≈0 + dRel 小 —— 雷达把一个几乎静止的目标当成了前车。
#
# 实车核对：grep LongDecel /data/log/swaglog.*
#   DECEL_PROBE_ENABLE   一键开关
#   DECEL_PROBE_A_TARGET 输出加速度低于此值记一条 (m/s²)
#   DECEL_PROBE_INTERVAL 节流间隔 (s)
DECEL_PROBE_ENABLE = True
DECEL_PROBE_A_TARGET = -0.8
DECEL_PROBE_INTERVAL = 3.0

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def update_allow_throttle(allow_prev: bool, flip_prev: int, throttle_prob: float, v_ego: float) -> tuple[bool, int]:
  """allow_throttle 的状态推进（带迟滞，见 ALLOW_THROTTLE_HYST_FRAMES）。

  抽成纯函数是为了能被 verify_cruise_stale.py 直接用真实源码驱动测试
  （本仓库惯例：测试不抄逻辑副本）。
  返回 (allow_throttle, 连续反向帧计数)。
  """
  # 低速项是硬覆盖，必须瞬时生效（起步/排队场景不能等迟滞）
  if v_ego <= MIN_ALLOW_THROTTLE_SPEED:
    return True, 0

  want = bool(throttle_prob > ALLOW_THROTTLE_THRESHOLD)
  if want == allow_prev:
    return allow_prev, 0

  flip = flip_prev + 1
  if flip >= max(1, int(ALLOW_THROTTLE_HYST_FRAMES)):
    return want, 0
  return allow_prev, flip


def is_cruise_stale(v_cruise: float, v_ego: float, a_cruise: float) -> bool:
  """「巡航候选残留负值」判据：该加速却给了负号（见 CRUISE_STALE_PROBE_*）。"""
  return (v_cruise - v_ego) > CRUISE_STALE_MIN_GAP_MS and a_cruise < CRUISE_STALE_MAX_A


def enum_str(v) -> str:
  """capnp 枚举的安全字符串化（**日志专用**，绝不能被 int() 替换）。

  ⚠ 实测（2026-09-20，设备上的 pycapnp）:`capnp.lib.capnp._DynamicEnum` **没有**
  `__int__` —— `int(sm['controlsState'].longControlState)` 会抛
      TypeError: int() argument must be a string, a bytes-like object or a real
                 number, not 'capnp.lib.capnp._DynamicEnum'
  而它嵌在 f-string 里，异常会**直接从 plannerd 的 update() 抛出**：
  plannerd 当场崩、被 manager 反复拉起，症状是「纵向控制整个没了」——
  比「日志少一个字段」严重得多。可用的是 str()（得到 'off'）与 .raw（得到 0）。
  这类 bug 本地跑不出来（本机没有 capnp），只在设备上现形：
  verify_coast_resume.py 的 E 段（真实 planner 集成）跑一次就报。
  本脚本 G/H 段有静态守卫，禁止再写出 `int(sm[...])`。
  """
  return str(v)

def get_cruise_accel_min(v_cruise, v_ego, accel_coast):
  """cruise 速度跟踪的减速度下限（超速回落力度）。

  未超速时这个下限不起作用（v_cruise - v_ego > 0），只有超速时它才决定回落力度：
  超速幅度 <= COAST_OVERSPEED_PURE_KPH 用自然滑行 accel_coast（平路约 -0.3 m/s²），
  线性过渡到 COAST_OVERSPEED_FULL_KPH 时恢复原来的 A_CRUISE_MIN(-1.2 m/s²)。
  """
  if not COAST_OVERSPEED_ENABLE:
    return A_CRUISE_MIN

  overspeed_kph = max(0.0, v_ego - v_cruise) * CV.MS_TO_KPH
  # accel_coast = sin(pitch) * -5.65 - 0.3，是「零踏板自然加速度」：
  #   长下坡可能为正（滑行反而越滑越快）-> 夹到 0，保证至少不主动加速；
  #   陡上坡可能比 -1.2 还负（自然减速更快）-> 夹到 A_CRUISE_MIN，
  #   保证滑行回落永远不会比原来的硬刹更激进。
  coast_min = float(np.clip(accel_coast, A_CRUISE_MIN, 0.0))
  return float(np.interp(overspeed_kph,
                         [COAST_OVERSPEED_PURE_KPH, COAST_OVERSPEED_FULL_KPH],
                         [coast_min, A_CRUISE_MIN]))


def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  # 超速回落的下限：小超速滑行、大超速逐渐加大刹车（见 COAST_OVERSPEED_* 常量）
  accel_min = get_cruise_accel_min(v_cruise, v_ego, accel_coast)
  target_accel = np.clip(v_cruise - v_ego, accel_min, max_accel)
  j_cruise = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS))
  # [2026-09-20] 非对称 jerk：向下（越来越负）永远用原值；向上时若 a_cruise 还陷在
  # 负值里，用更快的恢复速率，避免「该加速却刹着」持续 3 秒（见 CRUISE_RECOVER_J_MS3）。
  j_up = j_cruise
  if a_cruise_prev < 0.0 and CRUISE_RECOVER_J_MS3 is not None:
    j_up = max(j_cruise, float(CRUISE_RECOVER_J_MS3))
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_up * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True
    # [2026-09-20] allow_throttle 迟滞计数（见 ALLOW_THROTTLE_HYST_FRAMES）
    self._allow_throttle_flip = 0

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    # 打灯减速 + 大角度限加速（sunnypilot 追加，见 turn_decel.py）
    self.turn_decel = TurnDecelController()

    # 松油门滑行回落：跟车时踩一脚油追上去，松开不该被一脚重刹"顶"回来
    # （机理与判据见 sunnypilot/.../lib/coast_resume.py 文件头）
    self.coast_resume = CoastResumeController()
    self._coast_resume_active = False

    # 超速滑行回落的状态（仅用于状态跳变时打一条日志，见 update）
    self._coast_overspeed_active = False

    # 大减速现场记录器的节流时间戳（见 update 与文件头 DECEL_PROBE_*）
    self._decel_probe_ts = -1e9
    # 「巡航候选残留负值」探针的节流时间戳（见文件头 CRUISE_STALE_PROBE_*）
    self._cruise_stale_ts = -1e9
    # 供 [LongDecel] 记录当前帧是否处于 reset_state（见文件头 CRUISE_RECOVER_J_MS3）
    self._dbg_reset_state = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    # [2026-09-20] 加迟滞（见 ALLOW_THROTTLE_HYST_FRAMES / update_allow_throttle）：
    # gasPressProbs 是模型的逐帧输出，在 0.4 门限附近抖动时 allow_throttle 会 20 Hz
    # 反复翻转；非 e2e 路径下它直接决定巡航的加速上限（False 时高速段上限被压到
    # accel_coast ≈ -0.3 m/s²），翻转即「踩油门 / 松油门」的抖动。
    # ⚠ 实测（2026-09-20 全部 15 条 LongDecel 样本）：本车跑实验模式，日志里 e2e=1，
    #   而 get_cruise_accel 的 allow_throttle 分支被 `if not e2e:` 挡住 —— 即
    #   **e2e 模式下这一项不参与控制**（只影响 longitudinalPlan.allowThrottle 的发布值）。
    #   所以它修的不是本次「踩油门刹车」；保留它是为了非 e2e 模式下的健壮性。
    self.allow_throttle, self._allow_throttle_flip = update_allow_throttle(
      self.allow_throttle, self._allow_throttle_flip, throttle_prob, v_ego)

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    self._dbg_reset_state = bool(reset_state)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      # [2026-09-20] a_cruise 是「巡航速度跟踪意图」，这里**不能**继承 aEgo 的负值。
      # reset_state 在两类情况下为真：① 未接管；② carState.vCruise == V_CRUISE_UNSET(255)
      # —— 后者在实车上会**逐帧**成立（实车 12:45:28 抓到 vCruiseUI=145=V_CRUISE_MAX，
      # 说明原值 255 被 145 截断，即 V_CRUISE_UNSET）。于是 a_cruise 每帧被写成 clip(aEgo)，
      # 车在减速时巡航候选就一直是负的，被 min(candidates) 选中后压住 MPC 的加速意图
      # -> 「无前车、车速远低于设定，却不加速」。
      # 夹到 >=0 只削掉有害的那一半：正加速度照旧继承（避免再接管时跳变），
      # 而巡航候选即使退一步也只会是 0（不加速），**不会主动刹车**。
      self.a_cruise = max(0.0, float(self.output_a_target))
      # 未接管/车速未就绪时清掉打灯减速的累计状态，避免再接管时
      # 立刻按「转向灯已开很久」的旧状态减速
      self.turn_decel.reset()
      # 同理清掉滑行窗口，避免带着上一段的 gas 下降沿再接管时白滑一段
      self.coast_resume.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    # 红灯/停止标志的虚拟停止线：作为第 3 条障碍物交给 MPC。
    # traffic_stop.stop_dist_m 为 None 时内部会退化成 1000 m 外的哨兵，行为与改动前一致。
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality,
                    traffic_stop_obstacle_m=self.traffic_stop.stop_dist_m)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    # 超速滑行回落只在状态跳变时打一条日志（20Hz 循环里不能每帧刷），
    # 实车核对：grep CoastOverspeed /data/log/swaglog.*
    # [2026-09-20] 迟滞 0.5kph（见 COAST_OVERSPEED_LOG_HYST_KPH）。这里只驱动日志，
    # 不参与控制 —— 控制侧走 get_cruise_accel_min() 的连续插值。
    if COAST_OVERSPEED_ENABLE:
      _os_kph = (v_ego - v_cruise) * CV.MS_TO_KPH
      coast_overspeed_active = _os_kph > (COAST_OVERSPEED_LOG_HYST_KPH if self._coast_overspeed_active else 0.0)
    else:
      coast_overspeed_active = False
    if coast_overspeed_active != self._coast_overspeed_active:
      cloudlog.info(f"CoastOverspeed {'engaged' if coast_overspeed_active else 'released'}: "
                    f"vEgo={v_ego * CV.MS_TO_KPH:.1f} vCruise={v_cruise * CV.MS_TO_KPH:.1f} "
                    f"overspeed={(v_ego - v_cruise) * CV.MS_TO_KPH:.1f}kph aCruise={self.a_cruise:.2f}")
      self._coast_overspeed_active = coast_overspeed_active

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    # 红灯/停止标志的保底层：主刹车曲线来自上面 MPC 的虚拟停止线障碍物，
    # 这里只在「正在主动停等」时加一个候选，保证即使 MPC 求解异常也不会放过停止线。
    # 取 min() 的语义天然安全：它只可能让结果更保守，绝不会削弱 e2e / 跟车制动。
    if self.traffic_stop.is_active:
      ts_a = self.traffic_stop.output_a_target
      candidates.append((ts_a, LongitudinalPlanSource.cruise, should_stop(v_ego, ts_a)))
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # 实验模式(e2e)正加速度上限：e2e 与 MPC 候选都能给到 ACCEL_MAX，
    # 大模型接管后纵向偏激进，这里对正加速度单独封顶（见文件顶部常量）
    if is_e2e and E2E_ACCEL_MAX_M_S2 is not None:
      output_a_target = min(output_a_target, E2E_ACCEL_MAX_M_S2)

    # 打灯减速 + 大角度限加速（sunnypilot 追加）。
    # 放在 candidates 的 min() 之后、np.clip 之前：
    #   - min() 保证不会盖过 candidates 里更保守的一方（FCW/前车/MPC 该刹还是刹）
    #   - 同时不会被 e2e 的正加速度覆盖掉"不许加速"的约束
    # 两种意图：打转向灯后弯道减速；以及方向角度过大时（与转向灯无关）不许加速
    blinker_on = bool(sm['carState'].leftBlinker or sm['carState'].rightBlinker)
    turn_decel_res = self.turn_decel.update(
      blinker_on=blinker_on,
      v_ego=v_ego,
      steering_angle_deg=steer_angle_without_offset,
      dt=self.dt,
    )
    if turn_decel_res.a_target_override is not None:
      output_a_target = min(output_a_target, turn_decel_res.a_target_override)
    if turn_decel_res.block_accel and output_a_target > 0.0:
      output_a_target = 0.0

    # 松油门滑行回落（见 sunnypilot/.../lib/coast_resume.py 文件头）。
    # 放在 candidates 的 min() 之后、np.clip 之前：
    #   - 只抬「负值」-> 不会覆盖任何正的加速意图（e2e 上限、turn_decel 的 block_accel 都在它前面）
    #   - 只抬到自然滑行水平，前车危险 / 要停车 / FCW / 踩刹车时立刻让位
    # a_target_raw 传抬升**之前**的值：判据要用它判断"模型是否给出了极激进的减速"。
    a_target_raw = output_a_target
    coast_res = self.coast_resume.update(
      gas_pressed=sm['carState'].gasPressed,
      brake_pressed=sm['carState'].brakePressed,
      v_ego=v_ego,
      accel_coast=accel_coast,
      lead_present=sm['radarState'].leadOne.present,
      d_rel=sm['radarState'].leadOne.dRel,
      v_lead=sm['radarState'].leadOne.vLeadK,
      a_lead=sm['radarState'].leadOne.aLeadK,
      should_stop=self.output_should_stop,
      traffic_stop_active=self.traffic_stop.is_active,
      fcw=self.fcw,
      a_target_raw=a_target_raw,
      dt=self.dt,
    )
    if coast_res.a_floor is not None:
      output_a_target = max(output_a_target, coast_res.a_floor)

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    # 状态跳变各打一条（20 Hz 里不能每帧刷）。
    # aRaw 是"没被滑行抬过"的原始值，aRaw - aFloor 就是这一脚被柔和掉的幅度。
    # 实车核对：grep CoastResume /data/log/swaglog.*
    if coast_res.active != self._coast_resume_active:
      a_floor_dbg = coast_res.a_floor if coast_res.a_floor is not None else 0.0
      lead = sm['radarState'].leadOne
      cloudlog.info(f"CoastResume {'engaged' if coast_res.active else 'released'}: "
                    f"vEgo={v_ego * CV.MS_TO_KPH:.1f} vCruiseInt={v_cruise * CV.MS_TO_KPH:.1f} "
                    f"| aRaw={a_target_raw:.2f} aFloor={a_floor_dbg:.2f} aFinal={self.output_a_target:.2f} "
                    f"| lead={int(lead.present)} dRel={lead.dRel:.1f} vLead={lead.vLeadK:.1f} "
                    f"aLead={lead.aLeadK:.2f} leadReq={coast_res.lead_required:.2f} "
                    f"reason={coast_res.reason}")
      self._coast_resume_active = coast_res.active

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

    # ---- 大减速现场记录器（见文件头 DECEL_PROBE_*）----
    # 只在输出明显减速时记一条，用来事后判断「这一脚是谁给的」：
    #   vCruiseUI 与 vCruiseInt 不一致 -> SP 的 SCC / SLA 在悄悄压速
    #   lead 存在且 vLeadK≈0、dRel 小 -> 雷达把静止物当成了前车
    # [2026-09-20] 追加 5 个字段，把上一轮只能靠推理的几件事变成事实：
    #   vCruiseRaw  —— carState.vCruise 原值。=255(V_CRUISE_UNSET) 时 vCruiseUI 会被
    #                  V_CRUISE_MAX(145) 截断成 145，并且**逐帧**触发 reset_state。
    #   aEgo        —— 实际加速度。用来看 a_cruise 到底继承了多少「刹车残值」。
    #   enabled / longCtrl / resetState —— 确认这一帧到底在不在接管、是不是 reset 态。
    #   coastRes    —— coast_resume 是否正在抬升（reset_state 逐帧为真时它会被反复 reset 而失效）。
    if DECEL_PROBE_ENABLE and self.output_a_target < DECEL_PROBE_A_TARGET:
      now = time.monotonic()
      if now - self._decel_probe_ts >= DECEL_PROBE_INTERVAL:
        self._decel_probe_ts = now
        lead = sm['radarState'].leadOne
        a_e2e_str = f"{output_a_target_e2e:.2f}" if is_e2e else "n/a"
        cloudlog.info(
          f"[LongDecel] aTarget={self.output_a_target:.2f} src={self.mpc.source} spSrc={self.source} "
          f"| aMpc={output_a_target_mpc:.2f} aCruise={self.a_cruise:.2f} aE2e={a_e2e_str} "
          f"| vEgo={v_ego * CV.MS_TO_KPH:.0f} vCruiseUI={v_cruise_kph:.0f} "
          f"vCruiseInt={v_cruise * CV.MS_TO_KPH:.0f} "
          f"| lead={int(lead.present)} dRel={lead.dRel:.1f} vLeadK={lead.vLeadK:.1f} "
          f"prob={lead.modelProb:.2f} radar={int(lead.radar)} "
          f"| allowThr={int(self.allow_throttle)} aCoast={accel_coast:.2f} "
          f"turnDecel={turn_decel_res.phase} e2e={int(is_e2e)}"
          f" | aEgo={sm['carState'].aEgo:.2f} vCruiseRaw={sm['carState'].vCruise:.0f} "
          f"enabled={1 if sm['selfdriveState'].enabled else 0} "
          f"longCtrl={enum_str(sm['controlsState'].longControlState)} "
          f"resetState={1 if self._dbg_reset_state else 0} coastRes={1 if self._coast_resume_active else 0}"
        )

    # ---- 「巡航候选残留负值」探针（见文件头 CRUISE_STALE_PROBE_*）----
    # 判据 = 该加速却给了负号。这是本次修复的**直接观测量**与验收指标：
    # 修复前 a_cruise 从 -1.2 恢复要 ~3 s，这段里每秒命中一次；
    # 修复后恢复只要 ~0.6 s，burst 应该短到 0~1 条。
    if CRUISE_STALE_PROBE_ENABLE and is_cruise_stale(v_cruise, v_ego, self.a_cruise):
      now2 = time.monotonic()
      if now2 - self._cruise_stale_ts >= CRUISE_STALE_PROBE_INTERVAL:
        self._cruise_stale_ts = now2
        cloudlog.info(
          f"[CruiseStale] aCruise={self.a_cruise:.2f} aTarget={self.output_a_target:.2f} "
          f"| aMpc={output_a_target_mpc:.2f} src={self.mpc.source} spSrc={self.source} "
          f"| vEgo={v_ego * CV.MS_TO_KPH:.0f} vCruiseInt={v_cruise * CV.MS_TO_KPH:.0f} "
          f"vCruiseUI={v_cruise_kph:.0f} vCruiseRaw={sm['carState'].vCruise:.0f} "
          f"| aEgo={sm['carState'].aEgo:.2f} resetState={1 if self._dbg_reset_state else 0} "
          f"lead={1 if sm['radarState'].leadOne.present else 0} e2e={1 if is_e2e else 0}"
        )

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
