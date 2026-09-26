#!/usr/bin/env python3
import math
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
from openpilot.common.params import Params

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.turn_decel import TurnDecelController
from openpilot.sunnypilot.selfdrive.controls.lib.gas_override import GasOverrideController
from openpilot.sunnypilot.selfdrive.controls.lib.stop_soften import StopSoftenController

A_CRUISE_MAX_BP = [0., 2.77, 5.55, 8.33, 11.11, 13.89, 16.6, 19.4, 22.22, 25, 27.78, 33.33]
A_CRUISE_MAX_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
J_CRUISE_VALS = [1.10, 0.9, 0.80, 0.65, 0.50, 0.40, 0.35, 0.35, 0.35, 0.33, 0.31, 0.29]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# 实验模式（e2e）下的正加速上限。
#
# 原逻辑用 opendbc 的 ACCEL_MAX（= 2.0，那是**车辆物理上限**，用于最终
# np.clip 与 MPC 约束，不能动）：`max_accel = ACCEL_MAX if e2e else 查表`。
# 也就是说实验模式下 a_cruise 的加速上限直接顶到 2.0（≈0.20 g），
# 比非实验模式的查表值（低速 1.10 → 高速 0.29）猛得多，畅通路段模型
# 会把 desiredAcceleration 拉满，体感偏冲。
#
# 这里单独把 e2e 分支收窄到 1.6（≈0.16 g）。
# ★生效机制：a_cruise 永远在 candidates 里且 min() 取小 ⇒ 只要压住
#   a_cruise 的上限，最终 output_a_target 就不可能超过它，
#   无论 MPC 轨迹或 e2e 候选给多大都会被 min 掉。
#   （所以不需要去改 long_mpc.py 的求解约束，也就避开了动求解器的风险。）
E2E_MAX_ACCEL = 1.6

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  # 实验模式走 E2E_MAX_ACCEL(1.6)，非实验模式走速度查表（低速 1.10 → 高速 0.29）。
  # 见上方 E2E_MAX_ACCEL 的说明：这里压住 a_cruise 的上限，就压住了整条链路的正加速上限。
  max_accel = E2E_MAX_ACCEL if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
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

    # 驾驶员踩油门让位（需求三条 + 开关见 gas_override.py 文件头）。
    # 运行期开关是参数 GasPedalOverride（设置页 "Gas Pedal Override"，默认开），
    # 由控制器自己按 1 Hz 轮询，所以这里只把 Params 句柄注入进去。
    self.gas_override = GasOverrideController(Params())

    # 「低速/静止 + 前方静止目标」的减速柔化（见 stop_soften.py 文件头）。
    # 只在 v_ego 低 + 前车静止 + 距离 > 2 m 时抬下限，且地板含
    # "恰好停得住"的物理下界 ⇒ 不削弱任何真正的制动能力。
    self.stop_soften = StopSoftenController()

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
      # 同理清掉踩油门让位的相位机（否则再接管时可能带着旧的 coast/hold 相位）
      self.gas_override.reset()
      # 柔化模块无累计状态以外的相位，但保持与上面两个模块一致的复位语义
      self.stop_soften.reset()

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

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

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
      # [2026-09-21] 驾驶员踩油门 -> 本模块完全让位（含松油门后 2 s 宽限）。
      # 依据：实车探针 05:05:52 三条候选 aMpc=2.33/aCruise=2.00/aE2e=+1.11 全为正，
      # 最终 aTgt 却是 0.000（gas=1）—— min() 不可能产生 0，唯一来源就是这里的
      # block_accel。机理与安全论证见 turn_decel.py 文件头「驾驶员加速意图让位」。
      gas_pressed=sm['carState'].gasPressed,
    )
    if turn_decel_res.a_target_override is not None:
      output_a_target = min(output_a_target, turn_decel_res.a_target_override)
    if turn_decel_res.block_accel and output_a_target > 0.0:
      output_a_target = 0.0

    # 驾驶员踩油门让位（需求三条见 gas_override.py 文件头）。
    # 位置：candidates min() 和 turn_decel 之后、np.clip 之前。
    #   - 它只**抬升下限**（max），所以不可能抢走 turn_decel / FCW / 前车 /
    #     MPC 任何一方"更保守"的结论；反过来 turn_decel 压的是上限、本模块抬的
    #     是下限，两条约束互不覆盖。
    #   - 未接管（reset_state）时不参与，避免把"没接管"搅进相位机，也避免
    #     在没接管时武装松油门窗口。
    if not reset_state:
      lead = sm['radarState'].leadOne
      gas_res = self.gas_override.update(
        gas_pressed=bool(sm['carState'].gasPressed),
        v_ego=v_ego,
        # 用**原逻辑实际执行**的目标速度（SP 的 SCC/SLA 若在压低速度，这里就是
        # 压低后的值）：这样"滑行到设定速度"收回时，原逻辑刚好也在那里停止制动，
        # 交接处没有阶跃。
        v_cruise=v_cruise,
        accel_coast=accel_coast,
        a_target_in=output_a_target,
        dt=self.dt,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
      )
      output_a_target = gas_res.a_target_out
      # 需求 3「不要有空档期」：低速时 LongControl 会因为 should_stop 把状态机
      # 切到 stopping（不再跟随 a_target）。让位期间一并清掉该意图。
      if gas_res.suppress_should_stop:
        self.output_should_stop = False
      if gas_res.log is not None:
        cloudlog.info(gas_res.log)

      # 「低速/静止 + 前方静止目标」的减速柔化（需求与安全论证见
      # stop_soften.py 文件头）。位置：gas_override 之后、np.clip 之前。
      #   - 与 gas_override 同为"只抬下限"，两者互不覆盖（连续两次 max）
      #   - 触发条件含"前车静止"⇒ 常规跟车/前车在动完全不受影响
      #   - 地板含物理下界（恰好停得住）⇒ 不会因为柔化而追尾
      soften_res = self.stop_soften.update(
        v_ego=v_ego,
        lead_present=bool(lead.present),
        d_rel=float(lead.dRel),
        v_lead=float(lead.vLead),
        fcw=bool(self.fcw),
        force_decel=bool(sm['controlsState'].forceDecel),
        a_target_in=output_a_target,
        dt=self.dt,
      )
      output_a_target = soften_res.a_target_out
      if soften_res.log is not None:
        cloudlog.info(soften_res.log)

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

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
