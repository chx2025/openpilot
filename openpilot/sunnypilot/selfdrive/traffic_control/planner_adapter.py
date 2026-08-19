from __future__ import annotations

from dataclasses import dataclass
import time
from types import SimpleNamespace

import numpy as np

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.traffic_control.controller import (
  TeslaTrafficControlController,
  TrafficControlConfig,
  TrafficControlDecision,
  TrafficControlMode,
  TrafficControlPhase,
)
from openpilot.sunnypilot.selfdrive.traffic_control.stop_profile import StopProfileGenerator
from openpilot.sunnypilot.selfdrive.traffic_control.obstacle_state import (
  TrafficControlStrategy,
  TrafficObstacleMpcAdapter,
)
from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TRAFFIC_CONTROL_STALE_NS, TeslaTrafficControlObservation


MIN_STOP_REFERENCE = 2.0
MAX_STOP_REFERENCE = 12.0
# Match the producer state machine's observation dropout window. The producer
# keeps advancing its latched stop point during a short raw-CAN gap, so a single
# delayed IPC frame must not abruptly remove an already-active red constraint.
TRAFFIC_OBSTACLE_STALE_NS = 750_000_000


@dataclass
class TrafficControlDiagnostics:
  decision: TrafficControlDecision
  applied: bool = False
  constraint_accel: float = 0.0


@dataclass
class _PlannerOutputSnapshot:
  speeds: np.ndarray
  accels: np.ndarray
  jerks: np.ndarray
  a_target: float
  should_stop: bool
  allow_throttle: bool


class _PublishProxy:
  def __init__(self, pm, diagnostics: TrafficControlDiagnostics) -> None:
    self._pm = pm
    self._diagnostics = diagnostics

  def __getattr__(self, name):
    return getattr(self._pm, name)

  def send(self, service: str, msg) -> None:
    if service == "longitudinalPlanSP":
      decision = self._diagnostics.decision
      target = msg.longitudinalPlanSP.teslaTrafficControl
      target.mode = int(decision.mode)
      target.phase = int(decision.phase)
      target.active = decision.active
      target.shadow = decision.shadow
      target.applied = self._diagnostics.applied
      target.shouldStop = decision.should_stop
      target.remainingDistance = decision.remaining_distance
      target.stopReference = decision.stop_reference
      target.lightState = decision.light_state
      target.sourceBus = decision.source_bus
      target.quality = decision.quality
      target.constraintAccel = self._diagnostics.constraint_accel
    self._pm.send(service, msg)


class TrafficControlPlannerAdapter:
  """Optional Tesla stop/go constraint around any selected longitudinal backend."""

  def __init__(self, planner, CP, params: Params | None = None) -> None:
    self._planner = planner
    self._CP = CP
    self._params = params or Params()
    self._controller = TeslaTrafficControlController()
    self._profile = StopProfileGenerator(actuator_delay=float(CP.longitudinalActuatorDelay))
    self._diagnostics = TrafficControlDiagnostics(self._controller._decision())
    self._last_transition_seq = self._controller.transition_seq
    self._base_output: _PlannerOutputSnapshot | None = None
    self._read_config()
    self._obstacle_mpc = None
    self._latched_hold_obstacle = None
    if self._strategy == TrafficControlStrategy.obstacleChannel and hasattr(self._planner, "mpc"):
      self._obstacle_mpc = TrafficObstacleMpcAdapter(self._planner.mpc)
      self._planner.mpc = self._obstacle_mpc

  def __getattr__(self, name):
    return getattr(self._planner, name)

  @staticmethod
  def _mode(value) -> TrafficControlMode:
    try:
      return TrafficControlMode(int(value or 0))
    except (TypeError, ValueError):
      return TrafficControlMode.off

  def _read_config(self) -> None:
    reference_dm = self._params.get("TeslaTrafficStopReference", return_default=True)
    reference = float(np.clip(float(reference_dm or 60) / 10.0, MIN_STOP_REFERENCE, MAX_STOP_REFERENCE))
    config = TrafficControlConfig(
      mode=self._mode(self._params.get("TeslaTrafficControlMode", return_default=True)),
      default_stop_reference=reference,
      adaptive_reference=bool(self._params.get("TeslaTrafficAdaptiveReference", return_default=True)),
    )
    self._controller.set_config(config)
    self._profile.comfort_brake = config.comfort_brake
    try:
      self._strategy = TrafficControlStrategy(int(
        self._params.get("TeslaTrafficControlStrategy", return_default=True) or 0,
      ))
    except (TypeError, ValueError):
      self._strategy = TrafficControlStrategy.stopProfile

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

  @staticmethod
  def _traffic_obstacle(sm, now_ns: int):
    healthy = bool(
      sm.seen['trafficObstacleState'] and sm.alive['trafficObstacleState'] and sm.valid['trafficObstacleState']
    )
    if not healthy:
      return None
    obstacle = sm['trafficObstacleState']
    age_ns = now_ns - int(obstacle.frameMonoTime)
    return obstacle if 0 <= age_ns <= TRAFFIC_OBSTACLE_STALE_NS else None

  def _decision_from_obstacle(self, obstacle) -> TrafficControlDecision:
    if obstacle is None:
      return TrafficControlDecision(
        mode=self._controller.config.mode, phase=TrafficControlPhase.off,
        active=False, apply_constraint=False, shadow=False, should_stop=False,
        remaining_distance=0.0, stop_reference=self._controller.config.default_stop_reference,
        light_state=0, source_bus=0, quality=0,
      )
    try:
      mode = TrafficControlMode(int(obstacle.mode))
      phase = TrafficControlPhase(int(obstacle.phase))
    except (TypeError, ValueError):
      mode = TrafficControlMode.off
      phase = TrafficControlPhase.off
    active = phase in self._controller.ACTIVE_PHASES or phase == TrafficControlPhase.release
    return TrafficControlDecision(
      mode=mode, phase=phase, active=active,
      apply_constraint=mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and active,
      shadow=mode == TrafficControlMode.shadow and active,
      should_stop=bool(obstacle.shouldStop),
      remaining_distance=max(0.0, float(obstacle.desiredStopDistance)),
      stop_reference=max(0.0, float(obstacle.dRel) - float(obstacle.desiredStopDistance)),
      light_state=int(obstacle.lightState), source_bus=int(obstacle.sourceBus), quality=int(obstacle.quality),
    )

  def _update_obstacle_strategy(self, sm, now_ns: int) -> None:
    obstacle = self._traffic_obstacle(sm, now_ns)
    if obstacle is not None:
      try:
        phase = TrafficControlPhase(int(obstacle.phase))
      except (TypeError, ValueError):
        phase = TrafficControlPhase.off
      if bool(obstacle.validForControl) and bool(obstacle.shouldStop) and phase == TrafficControlPhase.hold:
        self._latched_hold_obstacle = SimpleNamespace(
          present=True, dRel=float(obstacle.dRel), desiredStopDistance=float(obstacle.desiredStopDistance),
          phase=int(obstacle.phase), lightState=int(obstacle.lightState), mode=int(obstacle.mode),
          shouldStop=True, sourceBus=int(obstacle.sourceBus), quality=int(obstacle.quality),
          eventId=int(obstacle.eventId), frameMonoTime=now_ns, validForControl=True,
          startRequested=False,
        )
      elif phase == TrafficControlPhase.release:
        self._latched_hold_obstacle = None

    car_state = sm['carState']
    car_control = sm['carControl']
    driver_override = bool(car_state.gasPressed or not car_control.enabled or not car_control.longActive)
    if driver_override:
      self._latched_hold_obstacle = None
    elif obstacle is None and self._latched_hold_obstacle is not None:
      obstacle = self._latched_hold_obstacle

    if self._obstacle_mpc is not None:
      age_s = (0.0 if obstacle is None else
               max(0.0, (now_ns - int(obstacle.frameMonoTime)) / 1e9))
      self._obstacle_mpc.set_obstacle(
        obstacle if obstacle is not None and bool(obstacle.validForControl) else None,
        distance_correction=max(0.0, float(sm['carState'].vEgo)) * age_s,
      )
    self._planner.update(sm)

    decision = self._decision_from_obstacle(obstacle)
    applied = bool(self._obstacle_mpc is not None and self._obstacle_mpc.last_applied)
    constraint_accel = float(self._planner.output_a_target) if applied else 0.0
    if applied and decision.should_stop:
      self._capture_base_output()
      self._planner.output_should_stop = True
      self._planner.allow_throttle = False
    if obstacle is not None and bool(obstacle.validForControl) and bool(obstacle.startRequested):
      release_applied, constraint_accel = self._apply_constraint(decision, sm)
      applied = applied or release_applied
    self._diagnostics = TrafficControlDiagnostics(decision, applied, constraint_accel)

  def _apply_constraint(self, decision: TrafficControlDecision, sm) -> tuple[bool, float]:
    if not decision.apply_constraint:
      if not decision.active:
        self._profile.reset()
      return False, 0.0

    planner = self._planner
    base_speeds = np.asarray(planner.v_desired_trajectory, dtype=float)
    times = np.asarray(ModelConstants.T_IDXS[:len(base_speeds)], dtype=float)
    v_ego = float(sm['carState'].vEgo)
    a_ego = float(sm['carState'].aEgo)

    # CP moves directly from e2eStopped to cruise on a confirmed green. Keep
    # that behavior narrowly scoped to the same Tesla event, low speed, valid
    # no-lead radar, and no driver brake/turn intent. The departure acceleration
    # comes from the official planner's cruise candidate and is capped here.
    if decision.phase == TrafficControlPhase.release:
      self._profile.reset()
      car_state = sm['carState']
      car_control = sm['carControl']
      radar_valid = bool(sm.seen['radarState'] and sm.alive['radarState'] and sm.valid['radarState'])
      leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo) if radar_valid else ()
      no_lead = radar_valid and not any(lead.present for lead in leads)
      low_speed = float(car_state.vEgo) <= 1.0
      driver_allows = (not car_state.brakePressed and not car_state.gasPressed and
                       not car_control.leftBlinker and not car_control.rightBlinker)
      cruise_accel = float(np.clip(getattr(planner, "a_cruise", 0.0), 0.0, 0.4))
      departure_allowed = (decision.mode == TrafficControlMode.stopGo and decision.light_state == 2 and
                           low_speed and no_lead and driver_allows and cruise_accel > 0.0)
      if not departure_allowed:
        return False, 0.0

      self._capture_base_output()
      previous_accel = float(planner.output_a_target)
      previous_should_stop = bool(planner.output_should_stop)
      previous_allow_throttle = bool(planner.allow_throttle)
      planner.output_a_target = max(previous_accel, cruise_accel)
      planner.output_should_stop = False
      planner.allow_throttle = True
      applied = (planner.output_a_target > previous_accel + 1e-3 or previous_should_stop or
                 not previous_allow_throttle)
      return applied, float(planner.output_a_target)

    stop_speeds, stop_accels, _ = self._profile.build_stop(
      v_ego=v_ego, a_ego=a_ego, remaining_distance=decision.remaining_distance,
      times=times, hold=decision.phase == TrafficControlPhase.hold,
    )

    base_accels = np.asarray(planner.a_desired_trajectory, dtype=float)
    merged_speeds = np.minimum(base_speeds, stop_speeds)
    merged_accels = np.minimum(base_accels, stop_accels)
    merged_jerks = np.diff(merged_accels) / np.maximum(np.diff(times), 1e-3)

    delay = float(self._CP.longitudinalActuatorDelay) + 0.05
    stop_constraint_accel = float(np.interp(delay, times, stop_accels))
    constraint_accel = min(float(planner.output_a_target), stop_constraint_accel)
    applied = (bool(np.any(merged_speeds < base_speeds - 1e-3)) or
               bool(np.any(merged_accels < base_accels - 1e-3)) or
               constraint_accel < float(planner.output_a_target) - 1e-3)
    if not applied:
      return False, constraint_accel

    self._capture_base_output()
    planner.v_desired_trajectory = merged_speeds
    planner.a_desired_trajectory = merged_accels
    planner.j_desired_trajectory = np.pad(
      merged_jerks, (0, max(0, len(merged_speeds) - len(merged_jerks))), mode="edge",
    )[:len(merged_speeds)]
    planner.output_a_target = constraint_accel
    planner.output_should_stop = bool(planner.output_should_stop or decision.should_stop)
    planner.allow_throttle = bool(planner.allow_throttle and not decision.should_stop)
    return applied, constraint_accel

  def _capture_base_output(self) -> None:
    if self._base_output is not None:
      return
    planner = self._planner
    self._base_output = _PlannerOutputSnapshot(
      speeds=np.asarray(planner.v_desired_trajectory, dtype=float).copy(),
      accels=np.asarray(planner.a_desired_trajectory, dtype=float).copy(),
      jerks=np.asarray(planner.j_desired_trajectory, dtype=float).copy(),
      a_target=float(planner.output_a_target),
      should_stop=bool(planner.output_should_stop),
      allow_throttle=bool(planner.allow_throttle),
    )

  def _restore_base_output(self) -> None:
    if self._base_output is None:
      return
    planner = self._planner
    planner.v_desired_trajectory = self._base_output.speeds
    planner.a_desired_trajectory = self._base_output.accels
    planner.j_desired_trajectory = self._base_output.jerks
    planner.output_a_target = self._base_output.a_target
    planner.output_should_stop = self._base_output.should_stop
    planner.allow_throttle = self._base_output.allow_throttle
    self._base_output = None

  def _log_transition(self, decision: TrafficControlDecision, observation: TeslaTrafficControlObservation,
                      *, v_ego: float, lead_present: bool, radar_valid: bool,
                      applied: bool, constraint_accel: float) -> None:
    if self._controller.transition_seq == self._last_transition_seq:
      return
    self._last_transition_seq = self._controller.transition_seq
    cloudlog.event(
      "tesla_traffic_control_transition",
      transition=self._controller.transition_reason,
      mode=int(decision.mode), phase=int(decision.phase), applied=applied,
      sourceBus=observation.source_bus, featureState=observation.feature_state,
      stateMachine=observation.state_machine, controlSource=observation.control_source,
      lightState=observation.light_state, observationDistance=round(observation.distance, 2),
      candidateDistance=round(self._controller.candidate_distance, 2),
      distanceInnovation=round(self._controller.last_distance_innovation, 2),
      confirmationRequiredS=round(self._controller.candidate_confirm_s, 2),
      remainingDistance=round(decision.remaining_distance, 2), vEgo=round(v_ego, 3),
      leadPresent=lead_present, radarValid=radar_valid,
      constraintAccel=round(constraint_accel, 3),
    )

  def update(self, sm) -> None:
    # A previous constrained publish must never seed the next backend cycle.
    # The normal plannerd loop always publishes, but restoring here as well
    # keeps the boundary safe if a caller skips publish after an exception.
    self._restore_base_output()
    now_ns = time.monotonic_ns()
    if self._strategy == TrafficControlStrategy.obstacleChannel:
      self._update_obstacle_strategy(sm, now_ns)
      return
    self._planner.update(sm)

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
    decision = self._controller.update(
      observation, now_ns, v_ego=float(car_state.vEgo), a_ego=float(car_state.aEgo),
      model_stop_distance=model_distance, model_stop_candidate=model_candidate,
      lead_present=lead_present, radar_valid=radar_valid, enabled=bool(car_control.enabled),
      long_active=bool(car_control.longActive), gas_pressed=bool(car_state.gasPressed),
      brake_pressed=bool(car_state.brakePressed),
      turn_signal_active=bool(car_control.leftBlinker or car_control.rightBlinker),
    )
    applied, constraint_accel = self._apply_constraint(decision, sm)
    self._log_transition(
      decision, observation, v_ego=float(car_state.vEgo), lead_present=lead_present,
      radar_valid=radar_valid, applied=applied, constraint_accel=constraint_accel,
    )
    self._diagnostics = TrafficControlDiagnostics(decision, applied, constraint_accel)

  def publish(self, sm, pm) -> None:
    try:
      self._planner.publish(sm, _PublishProxy(pm, self._diagnostics))
    finally:
      self._restore_base_output()
