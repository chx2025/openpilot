from __future__ import annotations

from enum import IntEnum
from types import SimpleNamespace

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


class TrafficObstacleGoPolicy(IntEnum):
  passive = 0
  active = 1


class TrafficControlStrategy(IntEnum):
  stopProfile = 0
  obstacleChannel = 1


class TrafficObstacleMpcAdapter:
  """Project the independent traffic target into an MPC-local lead channel."""

  def __init__(self, mpc) -> None:
    self._mpc = mpc
    self._obstacle = None
    self.last_applied = False

  def __getattr__(self, name):
    return getattr(self._mpc, name)

  def set_obstacle(self, obstacle, *, distance_correction: float = 0.0) -> None:
    self._obstacle = None if obstacle is None else SimpleNamespace(
      present=bool(obstacle.present),
      desiredStopDistance=max(0.0, float(obstacle.desiredStopDistance) - max(0.0, distance_correction)),
    )

  def update(self, radarstate, *args, **kwargs):
    obstacle = self._obstacle
    physical_lead = bool(radarstate.leadOne.present or radarstate.leadTwo.present)
    self.last_applied = bool(obstacle is not None and obstacle.present and not physical_lead)
    if self.last_applied:
      tuning = getattr(self._mpc, "runtime_tuning", None)
      stop_distance = float(getattr(tuning, "stop_distance", 6.0))
      virtual_lead = SimpleNamespace(
        present=True,
        dRel=max(0.0, float(obstacle.desiredStopDistance) + stop_distance),
        vLead=0.0,
        aLeadK=0.0,
        aLeadTau=1.5,
        modelProb=0.0,
        radar=False,
        radarTrackId=-1,
      )
      radarstate = SimpleNamespace(leadOne=radarstate.leadOne, leadTwo=virtual_lead)
    return self._mpc.update(radarstate, *args, **kwargs)


class TrafficObstacleSource:
  """Publish a traffic-control target independently from physical radarState."""

  def __init__(self, config: TrafficControlConfig,
               go_policy: TrafficObstacleGoPolicy = TrafficObstacleGoPolicy.passive) -> None:
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
    observation = (self._observation(sm['carStateSP'].teslaTrafficControl, now_ns)
                   if car_state_sp_valid else TeslaTrafficControlObservation())
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
    msg = messaging.new_message('trafficObstacleState')
    obstacle = msg.trafficObstacleState
    obstacle.present = bool(active_stop and valid_for_control)
    obstacle.dRel = float(decision.remaining_distance + decision.stop_reference) if active_stop else 0.0
    obstacle.vRel = -float(car_state.vEgo) if active_stop else 0.0
    obstacle.aRel = -float(car_state.aEgo) if active_stop else 0.0
    obstacle.desiredStopDistance = float(decision.remaining_distance)
    obstacle.phase = int(decision.phase)
    obstacle.lightState = int(decision.light_state)
    obstacle.sourceBus = int(decision.source_bus)
    obstacle.quality = int(decision.quality)
    obstacle.confidence = 1.0 if active_stop else 0.0
    obstacle.eventId = int(self.controller.event_id)
    obstacle.frameMonoTime = int(now_ns)
    obstacle.validForControl = valid_for_control
    obstacle.suppressedByLead = bool(lead_present or self._lead_suppressed)
    obstacle.shouldStop = bool(decision.should_stop)
    obstacle.startRequested = bool(
      self.go_policy == TrafficObstacleGoPolicy.active and
      decision.mode == TrafficControlMode.stopGo and
      decision.phase == TrafficControlPhase.release and
      valid_for_control
    )
    obstacle.mode = int(decision.mode)
    msg.valid = bool(model_valid and car_state_sp_valid and radar_valid)
    return msg
