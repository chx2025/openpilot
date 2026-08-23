"""Thin card Adapter for Tesla state-machine context.

The vehicle state machine remains in opendbc. This Module supplies fresh
planner/control context and observes the original speed-wheel template without
adding Tesla branches throughout generic card.
"""

import time
from typing import Any

from opendbc.sunnypilot.car.tesla.values import TeslaSafetyFlagsSP
from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import TeslaTurnSignalRealtimeController
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import (
  TeslaTrafficControlObserver, publish_tesla_traffic_control,
)


CONTEXT_STALE_S = 0.2
CONTEXT_SERVICES = ("selfdriveStateSP", "modelV2")


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


def speed_limit_context(sm, now: float, assist_configured: bool | None = None) -> tuple[float, bool]:
  plan = sm["longitudinalPlanSP"]
  plan_recv_time = float(sm.recv_time["longitudinalPlanSP"])
  plan_valid = (sm.seen["longitudinalPlanSP"] and sm.valid["longitudinalPlanSP"] and
                now - plan_recv_time <= CONTEXT_STALE_S)
  resolver = plan.speedLimit.resolver
  limit_valid = bool(resolver.speedLimitValid or resolver.speedLimitLastValid)
  target = float(resolver.speedLimitFinalLast)
  configured = bool(plan.speedLimit.assist.enabled) if assist_configured is None else assist_configured
  valid = plan_valid and configured and limit_valid and target > 0.0
  return (target if valid else 0.0, valid)


class TeslaCardAdapter:
  SPEED_BUTTON_ADDRESS = 0x3C2
  VEHICLE_BUS = 1

  def __init__(self, brand: str, car_interface: Any, submaster: Any):
    self.enabled = brand == "tesla"
    self.car_interface = car_interface
    self.sm = submaster
    self.traffic_control_observer = TeslaTrafficControlObserver() if self.enabled else None
    self.road_context_parser = self._create_road_context_parser() if self.enabled else None
    self.speed_limit_assist_configured: bool | None = None
    configured = bool(getattr(car_interface, "CP_SP", None) and
                      car_interface.CP_SP.safetyParam & TeslaSafetyFlagsSP.TURN_SIGNAL_VALIDATION)
    self.validation = TeslaTurnSignalRealtimeController(configured) if self.enabled else None

  def _create_road_context_parser(self):
    try:
      from opendbc.can import CANParser
      from opendbc.car import Bus
      from opendbc.car.tesla.values import CANBUS, DBC

      fingerprint = self.car_interface.CP.carFingerprint
      return CANParser(DBC[fingerprint][Bus.party], [("DAS_road", float("nan"))], CANBUS.party)
    except (AttributeError, KeyError):
      return None

  def observe_can(self, can_list) -> list:
    if not self.enabled:
      return []

    if self.traffic_control_observer is not None:
      self.traffic_control_observer.update(can_list, time.monotonic_ns())
    if self.road_context_parser is not None:
      self.road_context_parser.update(can_list)

    state = getattr(self.car_interface, "CS", None)
    update_template = getattr(state, "update_speed_button_template", None)

    for mono_time, frames in can_list:
      for address, data, source in frames:
        if self.validation is not None:
          self.validation.observe_frame(mono_time, address, data, source)
        if update_template is not None and source == self.VEHICLE_BUS and address == self.SPEED_BUTTON_ADDRESS:
          update_template(data, mono_time)

    if self.validation is None:
      return []
    now_nanos = time.monotonic_ns()
    self.validation.advance_time(now_nanos)
    # Cancellation cannot depend on controlsd continuing to publish carControl.
    return self.validation.take_can_sends(now_nanos, cancel_only=True)

  def control_sends(self, car_state, car_control, now_nanos: int) -> list:
    if self.validation is None:
      return []
    now = time.monotonic()
    model_valid = (self.sm.seen["modelV2"] and self.sm.valid["modelV2"] and
                   now - self.sm.recv_time["modelV2"] <= CONTEXT_STALE_S)
    lane_change = self.sm["modelV2"].meta
    self.validation.update_lane_change_context(
      now_nanos,
      valid=model_valid,
      state=int(getattr(lane_change.laneChangeState, "raw", lane_change.laneChangeState)),
      direction=int(getattr(lane_change.laneChangeDirection, "raw", lane_change.laneChangeDirection)),
      lateral_active=bool(car_control.latActive),
      brake_pressed=bool(car_state.brakePressed),
    )
    return self.validation.take_can_sends(now_nanos)

  def service_params(self, params) -> None:
    self.speed_limit_assist_configured = params.get("SpeedLimitMode", return_default=True) == Mode.assist
    if self.validation is not None:
      self.validation.service_params(params)

  def update_state(self, state_sp, now_ns: int | None = None) -> None:
    if self.road_context_parser is None:
      return

    from opendbc.sunnypilot.car.tesla.carstate_ext import publish_tesla_road_context

    timestamp_ns = self.road_context_parser.ts_nanos["DAS_road"]["DAS_stopLineDist"]
    publish_tesla_road_context(
      state_sp,
      self.road_context_parser.vl["DAS_road"],
      timestamp_ns,
      time.monotonic_ns() if now_ns is None else now_ns,
    )

  def publish_state(self, state_sp, now_ns: int | None = None) -> None:
    if self.traffic_control_observer is None:
      return
    publish_tesla_traffic_control(
      state_sp,
      self.traffic_control_observer.snapshot(time.monotonic_ns() if now_ns is None else now_ns),
    )

  def update_context(self, now: float | None = None) -> None:
    state = getattr(self.car_interface, "CS", None)
    update_longitudinal = getattr(state, "update_longitudinal_context", None)
    if not self.enabled or update_longitudinal is None:
      return

    timestamp = time.monotonic() if now is None else now
    update_longitudinal(*longitudinal_context(self.sm, timestamp))

    update_speed_limit = getattr(state, "update_speed_limit_target", None)
    if update_speed_limit is not None:
      update_speed_limit(*speed_limit_context(self.sm, timestamp, self.speed_limit_assist_configured))
