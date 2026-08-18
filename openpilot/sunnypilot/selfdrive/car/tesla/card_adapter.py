"""Thin card Adapter for Tesla state-machine context.

The vehicle state machine remains in opendbc. This Module supplies fresh
planner/control context and observes the original speed-wheel template without
adding Tesla branches throughout generic card.
"""

import time
from typing import Any


CONTEXT_STALE_S = 0.2
CONTEXT_SERVICES = ("selfdriveStateSP",)


def longitudinal_context(sm, now: float) -> tuple[int, bool, bool, float, bool, bool, bool, float, bool, float, bool]:
  plan = sm["longitudinalPlanSP"]
  plan_source = int(getattr(plan.longitudinalPlanSource, "raw", plan.longitudinalPlanSource))
  plan_recv_time = float(sm.recv_time["longitudinalPlanSP"])
  plan_valid = (sm.seen["longitudinalPlanSP"] and sm.valid["longitudinalPlanSP"] and
                now - plan_recv_time <= CONTEXT_STALE_S)

  car_control = sm["carControl"]
  car_control_valid = (sm.seen["carControl"] and sm.valid["carControl"] and
                       now - sm.recv_time["carControl"] <= CONTEXT_STALE_S)
  lane_change_active = bool(car_control.leftBlinker or car_control.rightBlinker)

  selfdrive_state_sp = sm["selfdriveStateSP"]
  mads_state_valid = (sm.seen["selfdriveStateSP"] and sm.valid["selfdriveStateSP"] and
                      now - sm.recv_time["selfdriveStateSP"] <= CONTEXT_STALE_S)
  lateral_control_ready = ((car_control_valid and bool(car_control.latActive)) or
                           (mads_state_valid and bool(selfdrive_state_sp.mads.active)))

  return (plan_source, sm.updated["longitudinalPlanSP"], plan_valid, plan_recv_time,
          lane_change_active, car_control_valid, lateral_control_ready, now,
          bool(car_control.longActive), float(car_control.actuators.accel), car_control_valid)


def speed_limit_context(sm, now: float) -> tuple[float, bool]:
  plan = sm["longitudinalPlanSP"]
  plan_recv_time = float(sm.recv_time["longitudinalPlanSP"])
  plan_valid = (sm.seen["longitudinalPlanSP"] and sm.valid["longitudinalPlanSP"] and
                now - plan_recv_time <= CONTEXT_STALE_S)
  resolver = plan.speedLimit.resolver
  limit_valid = bool(resolver.speedLimitValid or resolver.speedLimitLastValid)
  target = float(resolver.speedLimitFinalLast)
  valid = plan_valid and bool(plan.speedLimit.assist.enabled) and limit_valid and target > 0.0
  return (target if valid else 0.0, valid)


class TeslaCardAdapter:
  SPEED_BUTTON_ADDRESS = 0x3C2
  VEHICLE_BUS = 1

  def __init__(self, brand: str, car_interface: Any, submaster: Any):
    self.enabled = brand == "tesla"
    self.car_interface = car_interface
    self.sm = submaster

  def observe_can(self, can_list) -> None:
    state = getattr(self.car_interface, "CS", None)
    update_template = getattr(state, "update_speed_button_template", None)
    if not self.enabled or update_template is None:
      return

    for mono_time, frames in can_list:
      for address, data, source in frames:
        if source == self.VEHICLE_BUS and address == self.SPEED_BUTTON_ADDRESS:
          update_template(data, mono_time)

  def update_context(self, now: float | None = None) -> None:
    state = getattr(self.car_interface, "CS", None)
    update_longitudinal = getattr(state, "update_longitudinal_context", None)
    if not self.enabled or update_longitudinal is None:
      return

    timestamp = time.monotonic() if now is None else now
    update_longitudinal(*longitudinal_context(self.sm, timestamp))

    update_speed_limit = getattr(state, "update_speed_limit_target", None)
    if update_speed_limit is not None:
      update_speed_limit(*speed_limit_context(self.sm, timestamp))
