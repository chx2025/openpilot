from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP as UpstreamLongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.accel_controller.accel_controller import (
  AccelController, AccelControllerState,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.accel_controller.constants import BRAKING_ACCEL_THRESHOLD
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.long_mpc import (
  LongitudinalPlanSource as MpcLongitudinalPlanSource,
)


class LongitudinalPlannerSP(UpstreamLongitudinalPlannerSP):
  """TN behavior layered on the current upstream sunnypilot planner helpers."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc, dt: float = DT_MDL):
    super().__init__(CP, CP_SP, mpc, enable_dec=False)
    self.mpc = mpc
    self.accel_controller = AccelController(CP, dt=dt)
    self._radar_log_mono_time = None
    self._radar_fresh_this_cycle = True
    self._long_active_last_cycle = False
    self.previous_plan_accel = 0.0
    self.mpc_accel_seed = 0.0

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    # TN-NoDEC deliberately ignores Dynamic Experimental Control.
    return sm['selfdriveState'].experimentalMode

  def update_accel_controller(self, sm: messaging.SubMaster, v_cruise: float, prev_accel_constraint: bool,
                              stock_accel_max: float, reset_state: bool) -> tuple[bool, float]:
    is_e2e = self.is_e2e(sm)
    force_decel = sm['controlsState'].forceDecel
    previous_mpc_failed = self.mpc.last_solution_status != 0
    previous_plan_accel = self.previous_plan_accel if self._long_active_last_cycle and not previous_mpc_failed else float('inf')
    self.mpc_accel_seed = self.a_desired

    self.accel_controller.update(
      sm['radarState'], base_speed=self.output_v_target, v_ego=sm['carState'].vEgo, a_ego=sm['carState'].aEgo,
      follow_personality=sm['selfdriveState'].personality, acc_selected=not is_e2e,
      engaged=not reset_state and not force_decel, cruise_initialized=sm['carState'].vCruise != V_CRUISE_UNSET,
      stock_accel_max=stock_accel_max if self.allow_throttle else 0.0,
      radar_fresh=self._radar_fresh_this_cycle, previous_mpc_source=self.mpc.source, planner_speed=self.v_desired_filter.x,
      planner_accel=self.a_desired, previous_plan_accel=previous_plan_accel,
    )
    controller = self.accel_controller
    braking_handoff = (controller.is_active and not is_e2e and not previous_mpc_failed
                       and self.mpc.source == MpcLongitudinalPlanSource.e2e
                       and previous_plan_accel <= BRAKING_ACCEL_THRESHOLD)
    if braking_handoff:
      self.mpc_accel_seed = min(self.a_desired, previous_plan_accel)
    actuating = controller.is_active and not is_e2e and not force_decel and not previous_mpc_failed
    valid_lead_stop_hold = actuating and controller.state == AccelControllerState.stopHold and controller.selected_lead >= 0
    controller_v_cruise = v_cruise if valid_lead_stop_hold else min(v_cruise, controller.output_v_target) if actuating else v_cruise
    accel_max = controller.mpc_accel_max if actuating else None
    cruise_accel_max = controller.cruise_accel_max if actuating else None
    jerk_cost_multiplier = controller.get_jerk_cost_multiplier(
      actuating, prev_accel_constraint, v_cruise - controller_v_cruise, previous_mpc_failed,
    )
    self.mpc.set_accel_controller_params(accel_max, jerk_cost_multiplier, cruise_accel_max)
    self._long_active_last_cycle = not reset_state and not force_decel
    return is_e2e, controller_v_cruise

  def update_should_stop(self, should_stop: bool) -> bool:
    return self.accel_controller.update_should_stop(should_stop)

  def _update_radar_freshness(self, sm: messaging.SubMaster) -> bool:
    radar_log_mono_time = sm.logMonoTime['radarState']
    radar_healthy = sm.valid['radarState'] and sm.alive['radarState']
    radar_advanced = self._radar_log_mono_time is None or radar_log_mono_time > self._radar_log_mono_time
    if radar_advanced:
      self._radar_log_mono_time = radar_log_mono_time
    return radar_healthy and radar_advanced

  def _update_backend(self, sm: messaging.SubMaster) -> None:
    self.previous_plan_accel = self.output_a_target
    self._radar_fresh_this_cycle = self._update_radar_freshness(sm)
    self.accel_controller.update_params()

  def _publish_backend_state(self, longitudinal_plan_sp) -> None:
    accel_controller = longitudinal_plan_sp.accelController
    accel_controller.enabled = self.accel_controller.is_enabled
    accel_controller.active = self.accel_controller.is_active
    accel_controller.profile = self.accel_controller.profile
    accel_controller.state = self.accel_controller.state
