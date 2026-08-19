import math

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner as UpstreamLongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.long_mpc import (
  LongitudinalMpc, LongitudinalPlanSource, T_IDXS as T_IDXS_MPC,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP


A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20.0, 40.0]


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3


def limit_accel_in_turns(v_ego, lateral_curvature, accel_limits):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * abs(lateral_curvature)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
  return [accel_limits[0], min(accel_limits[1], a_x_allowed)]


def legacy_should_stop(v_ego: float, a_target: float) -> bool:
  return bool(v_ego < 0.25 and a_target < 0.1)


class LongitudinalPlanner(UpstreamLongitudinalPlanner):
  """Experimental provider kept behind the longitudinal backend registry."""

  def __init__(self, CP, CP_SP, **kwargs):
    super().__init__(CP, CP_SP, mpc_factory=LongitudinalMpc, **kwargs)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    accel_coast = (get_coast_accel(sm['carControl'].orientationNED[1])
                   if len(sm['carControl'].orientationNED) == 3 else ACCEL_MAX)
    v_ego = sm['carState'].vEgo
    v_cruise = min(sm['carState'].vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS
    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or sm['carState'].vCruise == V_CRUISE_UNSET
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = limit_accel_in_turns(
      v_ego, sm['controlsState'].curvature, [ACCEL_MIN, get_max_accel(v_ego)],
    )
    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    if not self.allow_throttle:
      clipped_coast = max(accel_coast, accel_clip[0])
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
                              [accel_clip[1], clipped_coast])
      accel_clip[1] = min(accel_clip[1], coast_limit)

    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(
      self, sm, self.v_desired_filter.x, self.a_desired, v_cruise,
    )
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    # Preserve the final rs408 inactive/reset path byte-for-byte. The robust
    # condensing fallback is only allowed once longitudinal control is active.
    self.mpc.set_recovery_enabled(sm['carControl'].longActive)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)
    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x += self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(
      self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX, action_t=action_t,
    )
    output_should_stop_mpc = legacy_should_stop(float(self.v_desired_trajectory[0]), output_a_target_mpc)
    output_a_target = output_a_target_mpc
    self.output_should_stop = output_should_stop_mpc
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    if self.is_e2e(sm):
      self.output_should_stop = bool(self.output_should_stop or sm['modelV2'].action.shouldStop)
      if output_a_target_e2e < output_a_target_mpc:
        output_a_target = output_a_target_e2e
        self.mpc.source = LongitudinalPlanSource.e2e

    for index in range(2):
      accel_clip[index] = np.clip(
        accel_clip[index], self.prev_accel_clip[index] - 0.05, self.prev_accel_clip[index] + 0.05,
      )
    self.output_a_target = float(np.clip(output_a_target, accel_clip[0], accel_clip[1]))
    self.prev_accel_clip = accel_clip
