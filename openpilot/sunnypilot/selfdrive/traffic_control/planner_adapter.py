from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.traffic_control.controller import (
  TrafficControlDecision,
  TrafficControlMode,
  TrafficControlPhase,
)
from openpilot.sunnypilot.selfdrive.traffic_control.radar_state import TrafficControlStrategy
from openpilot.sunnypilot.selfdrive.traffic_control.stop_profile import StopProfileGenerator
from openpilot.sunnypilot.selfdrive.traffic_control.target import TrafficMpcTarget


TRAFFIC_RADAR_STALE_NS = 250_000_000
ACTIVE_PHASES = {
  TrafficControlPhase.approachRed,
  TrafficControlPhase.braking,
  TrafficControlPhase.hold,
  TrafficControlPhase.release,
}


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


@dataclass(frozen=True)
class _TrafficRadarSnapshot:
  decision: TrafficControlDecision
  event_id: int
  publish_mono_time: int
  target_present: bool
  control_allowed: bool
  planner_start_requested: bool


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
  """Traffic-only constraint around a planner whose base behavior stays intact."""

  def __init__(self, planner, CP, params: Params | None = None) -> None:
    self._planner = planner
    self._CP = CP
    self._params = params or Params()
    self._profile = StopProfileGenerator(actuator_delay=float(CP.longitudinalActuatorDelay))
    self._base_output: _PlannerOutputSnapshot | None = None
    self._latched_hold: _TrafficRadarSnapshot | None = None
    self._last_start_event_id = -1
    try:
      self._strategy = TrafficControlStrategy(int(
        self._params.get("TeslaTrafficControlStrategy", return_default=True) or 0,
      ))
    except (TypeError, ValueError):
      self._strategy = TrafficControlStrategy.stopProfile
    self._diagnostics = TrafficControlDiagnostics(self._empty_decision(self._configured_mode()))

  def __getattr__(self, name):
    return getattr(self._planner, name)

  def _configured_mode(self) -> TrafficControlMode:
    try:
      return TrafficControlMode(int(self._params.get("TeslaTrafficControlMode", return_default=True) or 0))
    except (TypeError, ValueError):
      return TrafficControlMode.off

  @staticmethod
  def _empty_decision(mode: TrafficControlMode) -> TrafficControlDecision:
    return TrafficControlDecision(
      mode=mode,
      phase=TrafficControlPhase.off,
      active=False,
      apply_constraint=False,
      shadow=False,
      should_stop=False,
      remaining_distance=0.0,
      stop_reference=0.0,
      light_state=0,
      source_bus=0,
      quality=0,
    )

  @staticmethod
  def _physical_radar_clear(sm) -> bool:
    healthy = bool(sm.seen["radarState"] and sm.alive["radarState"] and sm.valid["radarState"])
    return healthy and not (sm["radarState"].leadOne.present or sm["radarState"].leadTwo.present)

  def _read_traffic_radar(self, sm, now_ns: int) -> _TrafficRadarSnapshot | None:
    healthy = bool(
      sm.seen["trafficRadarState"] and sm.alive["trafficRadarState"] and sm.valid["trafficRadarState"]
    )
    if not healthy:
      return None
    message = sm["trafficRadarState"]
    age_ns = now_ns - int(message.publishMonoTime)
    if not 0 <= age_ns <= TRAFFIC_RADAR_STALE_NS:
      return None
    try:
      mode = TrafficControlMode(int(message.mode))
      phase = TrafficControlPhase(int(message.phase))
    except (TypeError, ValueError):
      return None
    active = phase in ACTIVE_PHASES
    control_allowed = bool(message.controlAllowed and self._physical_radar_clear(sm))
    decision = TrafficControlDecision(
      mode=mode,
      phase=phase,
      active=active,
      apply_constraint=bool(
        control_allowed and mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and active
      ),
      shadow=bool(mode == TrafficControlMode.shadow and active),
      should_stop=bool(message.shouldStop),
      remaining_distance=max(0.0, float(message.distanceToStopPoint)),
      stop_reference=max(0.0, float(message.oemTargetDistance) - float(message.distanceToStopPoint)),
      light_state=int(message.lightState),
      source_bus=int(message.sourceBus),
      quality=int(message.quality),
    )
    return _TrafficRadarSnapshot(
      decision=decision,
      event_id=int(message.eventId),
      publish_mono_time=int(message.publishMonoTime),
      target_present=bool(message.targetPresent),
      control_allowed=control_allowed,
      planner_start_requested=bool(message.plannerStartRequested),
    )

  def _traffic_snapshot(self, sm, now_ns: int) -> _TrafficRadarSnapshot | None:
    snapshot = self._read_traffic_radar(sm, now_ns)
    car_state = sm["carState"]
    car_control = sm["carControl"]
    driver_override = bool(
      car_state.gasPressed or car_state.brakePressed or not car_control.enabled or not car_control.longActive
    )
    if driver_override:
      self._latched_hold = None
      return snapshot

    if snapshot is not None:
      if snapshot.decision.phase == TrafficControlPhase.release:
        self._latched_hold = None
      elif (snapshot.control_allowed and snapshot.decision.phase == TrafficControlPhase.hold
            and snapshot.decision.should_stop):
        self._latched_hold = snapshot
      return snapshot

    if (self._latched_hold is not None and float(car_state.vEgo) <= 1.0
        and self._physical_radar_clear(sm)):
      return self._latched_hold
    return None

  def _set_mpc_target(self, snapshot: _TrafficRadarSnapshot | None) -> bool:
    mpc = getattr(self._planner, "mpc", None)
    setter = getattr(mpc, "set_traffic_target", None)
    active = bool(
      snapshot is not None and snapshot.target_present and snapshot.decision.apply_constraint
      and snapshot.control_allowed
    )
    if setter is not None:
      setter(TrafficMpcTarget(
        event_id=snapshot.event_id,
        distance_to_stop_point=snapshot.decision.remaining_distance,
        should_stop=snapshot.decision.should_stop,
      ) if active else None)
    return active

  def _run_traffic_radar(self, sm, snapshot: _TrafficRadarSnapshot | None) -> tuple[bool, float]:
    target_set = self._set_mpc_target(snapshot)
    try:
      self._planner.update(sm)
    finally:
      self._set_mpc_target(None)

    applied = bool(
      target_set and self._planner.mpc.source == log.LongitudinalPlan.LongitudinalPlanSource.lead2
    )
    constraint_accel = float(self._planner.output_a_target) if applied else 0.0
    if target_set and snapshot is not None and snapshot.decision.should_stop:
      self._capture_base_output()
      self._planner.output_should_stop = True
      self._planner.allow_throttle = False
      applied = True

    if snapshot is not None and snapshot.planner_start_requested:
      start_applied, start_accel = self._apply_planner_start(snapshot, sm)
      applied = applied or start_applied
      if start_applied:
        constraint_accel = start_accel
    return applied, constraint_accel

  def _apply_planner_start(self, snapshot: _TrafficRadarSnapshot, sm) -> tuple[bool, float]:
    if snapshot.event_id == self._last_start_event_id:
      return False, 0.0
    decision = snapshot.decision
    car_state = sm["carState"]
    car_control = sm["carControl"]
    driver_allows = not (
      car_state.brakePressed or car_state.gasPressed or car_control.leftBlinker or car_control.rightBlinker
    )
    v_cruise = float(car_state.vCruise)
    cruise_initialized = 0.0 < v_cruise < V_CRUISE_UNSET
    departure_allowed = bool(
      self._strategy == TrafficControlStrategy.trafficRadar
      and decision.mode == TrafficControlMode.stopGo
      and decision.phase == TrafficControlPhase.release
      and decision.light_state == 2
      and float(car_state.vEgo) <= 1.0
      and car_control.enabled and car_control.longActive
      and self._physical_radar_clear(sm)
      and driver_allows and cruise_initialized
    )
    if not departure_allowed:
      return False, 0.0

    self._last_start_event_id = snapshot.event_id
    start_accel = float(np.clip(v_cruise / 3.6 - float(car_state.vEgo), 0.0, 0.4))
    if start_accel <= 0.0:
      return False, 0.0
    self._capture_base_output()
    previous = float(self._planner.output_a_target)
    self._planner.output_a_target = max(previous, start_accel)
    self._planner.output_should_stop = False
    self._planner.allow_throttle = True
    return bool(self._planner.output_a_target > previous + 1e-3), float(self._planner.output_a_target)

  def _apply_stop_profile(self, decision: TrafficControlDecision, sm) -> tuple[bool, float]:
    if not decision.apply_constraint or decision.phase == TrafficControlPhase.release:
      if not decision.active:
        self._profile.reset()
      return False, 0.0

    planner = self._planner
    base_speeds = np.asarray(planner.v_desired_trajectory, dtype=float)
    times = np.asarray(ModelConstants.T_IDXS[:len(base_speeds)], dtype=float)
    stop_speeds, stop_accels, _ = self._profile.build_stop(
      v_ego=float(sm["carState"].vEgo),
      a_ego=float(sm["carState"].aEgo),
      remaining_distance=decision.remaining_distance,
      times=times,
      hold=decision.phase == TrafficControlPhase.hold,
    )
    base_accels = np.asarray(planner.a_desired_trajectory, dtype=float)
    merged_speeds = np.minimum(base_speeds, stop_speeds)
    merged_accels = np.minimum(base_accels, stop_accels)
    merged_jerks = np.diff(merged_accels) / np.maximum(np.diff(times), 1e-3)
    delay = float(self._CP.longitudinalActuatorDelay) + 0.05
    stop_accel = float(np.interp(delay, times, stop_accels))
    constraint_accel = min(float(planner.output_a_target), stop_accel)
    applied = bool(
      np.any(merged_speeds < base_speeds - 1e-3)
      or np.any(merged_accels < base_accels - 1e-3)
      or constraint_accel < float(planner.output_a_target) - 1e-3
    )
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
    return True, constraint_accel

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

  def update(self, sm) -> None:
    self._restore_base_output()
    snapshot = self._traffic_snapshot(sm, time.monotonic_ns())
    decision = snapshot.decision if snapshot is not None else self._empty_decision(self._configured_mode())
    if self._strategy == TrafficControlStrategy.trafficRadar:
      applied, constraint_accel = self._run_traffic_radar(sm, snapshot)
    else:
      self._planner.update(sm)
      applied, constraint_accel = self._apply_stop_profile(decision, sm)
    self._diagnostics = TrafficControlDiagnostics(decision, applied, constraint_accel)

  def publish(self, sm, pm) -> None:
    try:
      self._planner.publish(sm, _PublishProxy(pm, self._diagnostics))
    finally:
      self._restore_base_output()
