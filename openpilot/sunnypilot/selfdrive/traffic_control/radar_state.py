"""Independent Traffic Radar producer state."""

from __future__ import annotations

from enum import IntEnum
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.traffic_control.controller import (
  TeslaTrafficControlController,
  TrafficControlConfig,
  TrafficControlMode,
  TrafficControlPhase,
)
from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import (
  TRAFFIC_CONTROL_STALE_NS,
  TeslaTrafficControlObservation,
)


class TrafficRadarGoPolicy(IntEnum):
  passive = 0
  active = 1


class TrafficControlStrategy(IntEnum):
  stopProfile = 0
  trafficRadar = 1


TRANSITION_REASON_CODES = {
  "": 0,
  "stop_confirmed": 1,
  "driver_bypass": 2,
  "radar_invalid": 3,
  "lead_present": 4,
  "observation_dropout": 5,
  "stationary_hold": 6,
  "green_release": 7,
  "candidate_started": 8,
  "candidate_replaced": 9,
  "candidate_cancelled": 10,
}


class TrafficRadarSource:
  """Publish a traffic-control target independently from physical radarState."""

  def __init__(self, config: TrafficControlConfig,
               go_policy: TrafficRadarGoPolicy = TrafficRadarGoPolicy.passive) -> None:
    self.controller = TeslaTrafficControlController(config)
    self.go_policy = go_policy
    self._lead_suppressed = False
    self._reconfirm_since_ns: int | None = None

  @staticmethod
  def _observation(msg, now_ns: int) -> TeslaTrafficControlObservation:
    observation = TeslaTrafficControlObservation.from_message(msg)
    age_ns = now_ns - observation.frame_mono_time
    return observation if 0 <= age_ns <= TRAFFIC_CONTROL_STALE_NS else TeslaTrafficControlObservation()

  @staticmethod
  def _model_stop(model, v_ego: float, lead_distance: float | None) -> tuple[float | None, bool]:
    x = list(model.position.x)
    velocity = list(model.velocity.x)
    if not x or not velocity:
      return None, False
    index = min(31, len(x) - 1, len(velocity) - 1)
    distance = float(x[index])
    terminal_speed = float(velocity[index])
    below_lead = lead_distance is None or distance < lead_distance - 3.0
    max_distance = float(np.interp(v_ego * 3.6, [60.0, 80.0], [120.0, 150.0]))
    stopped = (v_ego < 0.28 and distance < 20.0 and terminal_speed < 10.0) or (
      v_ego < 82.0 / 3.6 and distance < max_distance and terminal_speed < min(3.0, v_ego * 0.7)
    )
    return distance, bool(stopped and below_lead)

  def update(self, sm, now_ns: int):
    car_state_sp_valid = bool(sm.seen['carStateSP'] and sm.alive['carStateSP'] and sm.valid['carStateSP'])
    raw_traffic = sm['carStateSP'].teslaTrafficControl if car_state_sp_valid else None
    raw_frame_mono_time = int(raw_traffic.frameMonoTime) if raw_traffic is not None else 0
    raw_distance = float(raw_traffic.distance) if raw_traffic is not None else 255.0
    observation = (self._observation(raw_traffic, now_ns)
                   if raw_traffic is not None else TeslaTrafficControlObservation())
    car_state = sm['carState']
    car_control = sm['carControl']
    radar_valid = bool(sm.seen['radarState'] and sm.alive['radarState'] and sm.valid['radarState'])
    leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo) if radar_valid else ()
    present_leads = [lead for lead in leads if lead.present]
    lead_present = bool(present_leads)
    lead_distance = min(float(lead.dRel) for lead in present_leads) if present_leads else None
    model_valid = bool(sm.seen['modelV2'] and sm.alive['modelV2'] and sm.valid['modelV2'])
    if model_valid:
      model_distance, model_candidate = self._model_stop(sm['modelV2'], float(car_state.vEgo), lead_distance)
    else:
      model_distance, model_candidate = None, False

    decision = self.controller.update(
      observation, now_ns, v_ego=float(car_state.vEgo), a_ego=float(car_state.aEgo),
      model_stop_distance=model_distance, model_stop_candidate=model_candidate,
      lead_present=lead_present, radar_valid=radar_valid, enabled=bool(car_control.enabled),
      long_active=bool(car_control.longActive), gas_pressed=bool(car_state.gasPressed),
      brake_pressed=bool(car_state.brakePressed),
      turn_signal_active=bool(car_control.leftBlinker or car_control.rightBlinker),
    )

    active_stop = decision.phase in (
      TrafficControlPhase.approachRed, TrafficControlPhase.braking, TrafficControlPhase.hold,
    )
    if self.controller.config.retain_event_with_lead and lead_present and decision.active:
      self._lead_suppressed = True
      self._reconfirm_since_ns = None
    elif self._lead_suppressed:
      if not decision.active:
        self._lead_suppressed = False
        self._reconfirm_since_ns = None
      elif model_candidate:
        if self._reconfirm_since_ns is None:
          self._reconfirm_since_ns = now_ns
        if (now_ns - self._reconfirm_since_ns) / 1e9 >= self.controller.config.model_confirm_s:
          self._lead_suppressed = False
          self._reconfirm_since_ns = None
      else:
        self._reconfirm_since_ns = None
    valid_for_control = bool(
      radar_valid and not lead_present and not self._lead_suppressed and decision.apply_constraint
    )
    msg = messaging.new_message('trafficRadarState')
    target = msg.trafficRadarState
    target.targetPresent = bool(active_stop)
    target.oemTargetDistance = float(decision.remaining_distance + decision.stop_reference) if active_stop else 0.0
    target.targetRelativeVelocity = -float(car_state.vEgo) if active_stop else 0.0
    target.targetRelativeAcceleration = -float(car_state.aEgo) if active_stop else 0.0
    target.distanceToStopPoint = float(decision.remaining_distance)
    target.phase = int(decision.phase)
    target.lightState = int(decision.light_state)
    target.sourceBus = int(decision.source_bus)
    target.quality = int(decision.quality)
    target.confidence = 1.0 if active_stop else 0.0
    target.eventId = int(self.controller.event_id)
    target.publishMonoTime = int(now_ns)
    target.controlAllowed = valid_for_control
    target.suppressedByPhysicalLead = bool(lead_present or self._lead_suppressed)
    target.shouldStop = bool(decision.should_stop)
    target.plannerStartRequested = bool(
      self.go_policy == TrafficRadarGoPolicy.active and
      decision.mode == TrafficControlMode.stopGo and
      decision.phase == TrafficControlPhase.release and
      valid_for_control
    )
    target.mode = int(decision.mode)
    target.rawGreenSeen = bool(raw_traffic is not None and raw_traffic.available and raw_traffic.lightState == 2)
    target.releaseEligible = bool(
      target.rawGreenSeen and decision.phase == TrafficControlPhase.release and valid_for_control
    )
    target.eventContinuous = self.controller.event_continuous
    target.eventTransitionReason = TRANSITION_REASON_CODES.get(self.controller.transition_reason, 255)
    target.eventTransitionSeq = self.controller.transition_seq
    target.rawDistance = raw_distance
    target.observationAgeMs = float(
      max(0, now_ns - raw_frame_mono_time) / 1e6 if raw_frame_mono_time > 0 else 0.0
    )
    msg.valid = bool(model_valid and car_state_sp_valid and radar_valid)
    return msg
