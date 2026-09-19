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

#A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
#A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MAX_BP = [0., 2.77, 5.55, 8.33, 11.11, 13.89, 16.6, 19.4, 22.22, 25, 27.78, 33.33]
A_CRUISE_MAX_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
J_CRUISE_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# 实验模式(end-to-end)正加速度上限（m/s²）。
# 非 e2e 时纵向走 A_CRUISE_MAX_VALS 表（低速封顶 1.10 m/s²），而 e2e 时
# get_cruise_accel() 直接把 max_accel 放成 ACCEL_MAX(≈2.0)，模型链路的
# e2e / MPC 候选也能给到 ACCEL_MAX —— 换成大模型后纵向更激进，体感偏猛。
# 这里给 e2e 路径单独加一道正加速度上限：
#   调小 = 更保守；设为 None = 完全恢复原始行为（不限制）
# 实车调参：1.4 仍偏猛 -> 1.2（当前值）；还猛继续降到 1.0；肉了往 1.5/1.6 升
E2E_ACCEL_MAX_M_S2: float | None = 1.2

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
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    # 打灯减速 + 大角度限加速（sunnypilot 追加，见 turn_decel.py）
    self.turn_decel = TurnDecelController()

    # 超速滑行回落的状态（仅用于状态跳变时打一条日志，见 update）
    self._coast_overspeed_active = False

    # 大减速现场记录器的节流时间戳（见 update 与文件头 DECEL_PROBE_*）
    self._decel_probe_ts = -1e9

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
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      # 未接管/车速未就绪时清掉打灯减速的累计状态，避免再接管时
      # 立刻按「转向灯已开很久」的旧状态减速
      self.turn_decel.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

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
    coast_overspeed_active = bool(COAST_OVERSPEED_ENABLE and v_ego > v_cruise)
    if coast_overspeed_active != self._coast_overspeed_active:
      cloudlog.info(f"CoastOverspeed {'engaged' if coast_overspeed_active else 'released'}: "
                    f"vEgo={v_ego * CV.MS_TO_KPH:.1f} vCruise={v_cruise * CV.MS_TO_KPH:.1f} "
                    f"overspeed={(v_ego - v_cruise) * CV.MS_TO_KPH:.1f}kph aCruise={self.a_cruise:.2f}")
      self._coast_overspeed_active = coast_overspeed_active

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
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

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

    # ---- 大减速现场记录器（见文件头 DECEL_PROBE_*）----
    # 只在输出明显减速时记一条，用来事后判断「这一脚是谁给的」：
    #   vCruiseUI 与 vCruiseInt 不一致 -> SP 的 SCC / SLA 在悄悄压速
    #   lead 存在且 vLeadK≈0、dRel 小 -> 雷达把静止物当成了前车
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
