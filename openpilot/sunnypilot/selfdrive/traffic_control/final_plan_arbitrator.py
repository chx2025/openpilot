from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import time

import numpy as np

from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.traffic_control import TRAFFIC_SIGNAL_CONTROL_PARAM
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.stop_profile import StopProfileGenerator


# CP's low-speed cruise acceleration envelope. This applies only to a
# same-event CAN-authoritative green start and remains time/speed bounded.
START_MAX_ACCEL = 1.6
START_MAX_SPEED = 2.5
START_MAX_DURATION_NS = 3_000_000_000
START_JERK_LIMIT = 1.0
TERMINAL_MAX_SPEED = 1.5
TERMINAL_LOOKAHEAD_S = 0.05
PLANNER_TRAFFIC_STALE_NS = 350_000_000


class TrafficPlanAction(IntEnum):
  none = 0
  stop = 1
  hold = 2
  start = 3
  release = 4


class TrafficStartBlockReason(IntEnum):
  none = 0
  noPreviousHold = 1
  eventMismatch = 2
  unsafeBasePlan = 3
  modelStop = 4
  driverOverride = 5
  physicalLead = 6
  invalidCruise = 7
  alreadyStarted = 8


@dataclass
class TrafficPlanDiagnostics:
  action: TrafficPlanAction = TrafficPlanAction.none
  applied: bool = False
  start_requested: bool = False
  start_applied: bool = False
  start_block_reason: TrafficStartBlockReason = TrafficStartBlockReason.none
  event_id: int = 0
  phase: int = int(TrafficControlPhase.off)
  light_state: int = 0
  remaining_distance: float = 0.0
  stop_reference: float = 0.0
  source_bus: int = 0
  quality: int = 0
  base_a_target: float = 0.0
  traffic_a_target: float = 0.0
  final_a_target: float = 0.0
  should_stop: bool = False
  terminal_catch_active: bool = False


class _TrafficPlanPublishSink:
  def __init__(self, pm, arbitrator: FinalPlanArbitrator, sm, now_ns: int) -> None:
    self._pm = pm
    self._arbitrator = arbitrator
    self._sm = sm
    self._now_ns = now_ns

  def __getattr__(self, name):
    return getattr(self._pm, name)

  def send(self, service: str, message) -> None:
    if service == "longitudinalPlan":
      self._arbitrator.apply(message.longitudinalPlan, self._sm, self._now_ns)
    elif service == "longitudinalPlanSP":
      self._arbitrator.annotate_plan_sp(message.longitudinalPlanSP)
    self._pm.send(service, message)


class FinalPlanArbitrator:
  """Explicit post-planner Traffic constraint; never wraps or mutates a planner backend."""

  def __init__(self, CP) -> None:
    self._actuator_delay = float(CP.longitudinalActuatorDelay)
    self._profile = StopProfileGenerator(
      actuator_delay=self._actuator_delay,
      release_jerk_limit=START_JERK_LIMIT,
    )
    self._held_event_id = 0
    self._active_start_event_id = 0
    self._completed_start_event_id = 0
    self._start_started_ns = 0
    self._was_stopping = False
    self._hold_latched = False
    self.diagnostics = TrafficPlanDiagnostics()

  def publisher(self, pm, sm, now_ns: int | None = None):
    return _TrafficPlanPublishSink(pm, self, sm, time.monotonic_ns() if now_ns is None else now_ns)

  @staticmethod
  def _healthy(sm, service: str) -> bool:
    return bool(sm.seen[service] and sm.alive[service] and sm.valid[service])

  def _traffic(self, sm, now_ns: int):
    if not self._healthy(sm, "trafficRadarState"):
      return None
    traffic = sm["trafficRadarState"]
    age_ns = now_ns - int(traffic.publishMonoTime)
    return traffic if 0 <= age_ns <= PLANNER_TRAFFIC_STALE_NS else None

  def _physical_radar_clear(self, sm) -> bool:
    if not self._healthy(sm, "radarState"):
      return False
    radar = sm["radarState"]
    return not (radar.leadOne.present or radar.leadTwo.present)

  @staticmethod
  def _driver_allows(sm) -> bool:
    car_state = sm["carState"]
    car_control = sm["carControl"]
    return bool(
      car_control.enabled and car_control.longActive
      and not car_state.gasPressed and not car_state.brakePressed
      and not car_control.leftBlinker and not car_control.rightBlinker
    )

  @staticmethod
  def _times(length: int) -> np.ndarray:
    return np.asarray(ModelConstants.T_IDXS[:length], dtype=float)

  @staticmethod
  def _padded_jerks(accels: np.ndarray, times: np.ndarray, output_length: int) -> np.ndarray:
    if len(accels) < 2:
      return np.zeros(output_length, dtype=float)
    jerks = np.diff(accels) / np.maximum(np.diff(times), 1e-3)
    return np.pad(jerks, (0, max(0, output_length - len(jerks))), mode="edge")[:output_length]

  def _set_diagnostics_from_traffic(self, traffic) -> None:
    if traffic is None:
      return
    self.diagnostics.event_id = int(traffic.eventId)
    self.diagnostics.phase = int(traffic.phase)
    self.diagnostics.light_state = int(traffic.lightState)
    self.diagnostics.remaining_distance = max(0.0, float(traffic.distanceToStopPoint))
    self.diagnostics.stop_reference = max(
      0.0, float(traffic.oemTargetDistance) - float(traffic.distanceToStopPoint),
    )
    self.diagnostics.source_bus = int(traffic.sourceBus)
    self.diagnostics.quality = int(traffic.quality)

  def _stop_target_accel(self, stop_accels: np.ndarray, times: np.ndarray, *, terminal: bool) -> float:
    actuator_time = self._actuator_delay + TERMINAL_LOOKAHEAD_S
    target = float(np.interp(actuator_time, times, stop_accels))
    if terminal:
      actuator_window = stop_accels[times <= actuator_time]
      if len(actuator_window):
        target = min(target, float(np.min(actuator_window)))
    return target

  def _apply_stop_constraint(self, plan, sm, *, remaining_distance: float,
                             hold: bool, terminal: bool) -> float:
    base_speeds = np.asarray(plan.speeds, dtype=float)
    base_accels = np.asarray(plan.accels, dtype=float)
    times = self._times(len(base_speeds))
    v_ego = float(sm["carState"].vEgo)
    stop_speeds, stop_accels, _ = self._profile.build_stop(
      v_ego=v_ego,
      a_ego=float(sm["carState"].aEgo),
      remaining_distance=remaining_distance,
      times=times,
      hold=hold,
    )
    final_speeds = np.minimum(base_speeds, stop_speeds)
    final_accels = np.minimum(base_accels, stop_accels)
    traffic_a_target = self._stop_target_accel(stop_accels, times, terminal=terminal)
    final_a_target = min(float(plan.aTarget), traffic_a_target)

    plan.speeds = final_speeds.tolist()
    plan.accels = final_accels.tolist()
    plan.jerks = self._padded_jerks(final_accels, times, len(plan.jerks)).tolist()
    plan.aTarget = final_a_target
    plan.shouldStop = bool(plan.shouldStop or hold or terminal)
    plan.allowThrottle = bool(plan.allowThrottle and not (hold or terminal))
    return traffic_a_target

  def _apply_stop(self, plan, sm, traffic) -> None:
    phase = TrafficControlPhase(int(traffic.phase))
    hold = bool(phase == TrafficControlPhase.hold or traffic.shouldStop)
    v_ego = float(sm["carState"].vEgo)
    remaining_distance = max(0.0, float(traffic.distanceToStopPoint))
    terminal_distance = (
      v_ego * (self._actuator_delay + TERMINAL_LOOKAHEAD_S)
      + v_ego ** 2 / (2.0 * self._profile.comfort_brake)
    )
    terminal_stop = bool(
      v_ego > 0.01
      and (
        remaining_distance <= 0.01
        or (v_ego <= TERMINAL_MAX_SPEED and remaining_distance <= terminal_distance)
      )
    )
    traffic_a_target = self._apply_stop_constraint(
      plan, sm,
      remaining_distance=remaining_distance,
      hold=hold,
      terminal=hold or terminal_stop,
    )

    if hold or terminal_stop:
      self._held_event_id = int(traffic.eventId)
      self._active_start_event_id = 0
      self._completed_start_event_id = 0
      self._start_started_ns = 0
      self._hold_latched = True
    self._was_stopping = True
    self.diagnostics.action = TrafficPlanAction.hold if hold else TrafficPlanAction.stop
    self.diagnostics.applied = True
    self.diagnostics.traffic_a_target = traffic_a_target
    self.diagnostics.terminal_catch_active = bool(terminal_stop or (hold and v_ego > 0.01))

  def _start_block_reason(self, sm, traffic) -> TrafficStartBlockReason:
    event_id = int(traffic.eventId)
    if self._held_event_id == 0:
      return TrafficStartBlockReason.noPreviousHold
    if event_id != self._held_event_id:
      return TrafficStartBlockReason.eventMismatch
    if self._completed_start_event_id == event_id:
      return TrafficStartBlockReason.alreadyStarted
    if not self._driver_allows(sm):
      return TrafficStartBlockReason.driverOverride
    if not self._physical_radar_clear(sm):
      return TrafficStartBlockReason.physicalLead
    # A same-event OEM CAN green is authoritative over a model/base-plan
    # traffic-stop residue, matching CP's e2eStopped -> e2eCruise transition.
    # Physical context and driver gates above remain absolute vetoes.
    v_cruise = float(sm["carState"].vCruise)
    if not 0.0 < v_cruise < V_CRUISE_UNSET:
      return TrafficStartBlockReason.invalidCruise
    return TrafficStartBlockReason.none

  def _finish_start(self, event_id: int) -> None:
    self._completed_start_event_id = event_id
    self._active_start_event_id = 0
    self._start_started_ns = 0

  def _apply_start(self, plan, sm, traffic, now_ns: int) -> bool:
    self.diagnostics.start_requested = True
    event_id = int(traffic.eventId)
    block_reason = self._start_block_reason(sm, traffic)
    self.diagnostics.start_block_reason = block_reason
    if block_reason != TrafficStartBlockReason.none:
      if self._active_start_event_id == event_id:
        self._finish_start(event_id)
      return False

    self._hold_latched = False

    v_ego = float(sm["carState"].vEgo)
    if v_ego > START_MAX_SPEED:
      self._finish_start(event_id)
      return False
    if self._active_start_event_id == 0:
      self._active_start_event_id = event_id
      self._start_started_ns = now_ns
    elif self._active_start_event_id != event_id:
      return False
    if now_ns - self._start_started_ns > START_MAX_DURATION_NS:
      self._finish_start(event_id)
      return False
    base_a_target = float(plan.aTarget)
    requested_accel = float(np.clip(float(sm["carState"].vCruise) / 3.6 - v_ego, 0.0, START_MAX_ACCEL))
    if requested_accel <= 0.0:
      return False

    times = self._times(len(plan.speeds))
    start_speeds, start_accels, _ = self._profile.build_release(
      v_ego=v_ego, base_accel=requested_accel, times=times,
      preserve_positive_accel=True,
    )
    start_a_target = float(np.interp(self._actuator_delay + 0.05, times, start_accels))
    final_a_target = float(np.clip(max(base_a_target, start_a_target), 0.0, START_MAX_ACCEL))

    plan.speeds = start_speeds.tolist()
    plan.accels = start_accels.tolist()
    plan.jerks = self._padded_jerks(start_accels, times, len(plan.jerks)).tolist()
    plan.aTarget = final_a_target
    plan.shouldStop = False
    plan.allowThrottle = bool(plan.allowThrottle)

    self._was_stopping = False
    self.diagnostics.action = TrafficPlanAction.start
    self.diagnostics.applied = True
    self.diagnostics.start_applied = True
    self.diagnostics.traffic_a_target = start_a_target
    return True

  def _apply_latched_hold(self, plan, sm) -> None:
    traffic_a_target = self._apply_stop_constraint(
      plan, sm,
      remaining_distance=0.0,
      hold=True,
      terminal=True,
    )
    self.diagnostics.action = TrafficPlanAction.hold
    self.diagnostics.applied = True
    self.diagnostics.event_id = self._held_event_id
    self.diagnostics.traffic_a_target = traffic_a_target
    self.diagnostics.terminal_catch_active = float(sm["carState"].vEgo) > 0.01

  def _apply_release(self, plan, sm) -> None:
    if not self._was_stopping:
      self._profile.reset()
      return
    base_speeds = np.asarray(plan.speeds, dtype=float)
    base_accels = np.asarray(plan.accels, dtype=float)
    times = self._times(len(base_speeds))
    release_speeds, release_accels, _ = self._profile.build_release(
      v_ego=float(sm["carState"].vEgo), base_accel=float(plan.aTarget), times=times,
    )
    final_speeds = np.minimum(base_speeds, release_speeds)
    final_accels = np.minimum(base_accels, release_accels)
    release_a_target = float(np.interp(self._actuator_delay + 0.05, times, release_accels))
    final_a_target = min(float(plan.aTarget), release_a_target)
    constrained = bool(
      final_a_target < float(plan.aTarget) - 1e-3
      or np.any(final_speeds < base_speeds - 1e-3)
      or np.any(final_accels < base_accels - 1e-3)
    )
    if not constrained:
      self._was_stopping = False
      self._profile.reset()
      return
    plan.speeds = final_speeds.tolist()
    plan.accels = final_accels.tolist()
    plan.jerks = self._padded_jerks(final_accels, times, len(plan.jerks)).tolist()
    plan.aTarget = final_a_target
    self.diagnostics.action = TrafficPlanAction.release
    self.diagnostics.applied = True
    self.diagnostics.traffic_a_target = release_a_target

  def apply(self, plan, sm, now_ns: int | None = None) -> None:
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    base_a_target = float(plan.aTarget)
    self.diagnostics = TrafficPlanDiagnostics(base_a_target=base_a_target, final_a_target=base_a_target)
    traffic = self._traffic(sm, now_ns)
    self._set_diagnostics_from_traffic(traffic)

    physical_clear = self._physical_radar_clear(sm)
    driver_allows = self._driver_allows(sm)
    confirmed_release = bool(
      traffic is not None and int(traffic.eventId) == self._held_event_id
      and int(traffic.phase) == int(TrafficControlPhase.release) and int(traffic.lightState) == 2
    )
    if confirmed_release:
      self._hold_latched = False
    elif not driver_allows:
      self._hold_latched = False
      if self._active_start_event_id != 0:
        self._finish_start(self._active_start_event_id)
      self._was_stopping = False
      self._profile.reset()
      self.diagnostics.final_a_target = float(plan.aTarget)
      self.diagnostics.should_stop = bool(plan.shouldStop)
      return
    active_stop = bool(
      traffic is not None and traffic.targetPresent and traffic.controlAllowed
      and int(traffic.lightState) == 1 and float(traffic.confidence) >= 0.9
      and int(traffic.phase) in (
        int(TrafficControlPhase.approachRed), int(TrafficControlPhase.braking), int(TrafficControlPhase.hold),
      )
      and physical_clear and driver_allows
    )
    if active_stop:
      self._apply_stop(plan, sm, traffic)
    elif traffic is not None and bool(traffic.plannerStartRequested) and int(traffic.lightState) == 2:
      if not self._apply_start(plan, sm, traffic, now_ns):
        self._apply_release(plan, sm)
    elif self._hold_latched and float(sm["carState"].vEgo) <= TERMINAL_MAX_SPEED and physical_clear and driver_allows:
      self._apply_latched_hold(plan, sm)
    else:
      if self._active_start_event_id != 0:
        self._finish_start(self._active_start_event_id)
      self._apply_release(plan, sm)

    self.diagnostics.final_a_target = float(plan.aTarget)
    self.diagnostics.should_stop = bool(plan.shouldStop)

  def annotate_plan_sp(self, plan_sp) -> None:
    diagnostics = self.diagnostics
    plan_sp.aTarget = diagnostics.final_a_target
    target = plan_sp.teslaTrafficControl
    target.mode = 4
    target.phase = diagnostics.phase
    target.active = diagnostics.action != TrafficPlanAction.none
    target.shadow = False
    target.applied = diagnostics.applied
    target.shouldStop = diagnostics.should_stop
    target.remainingDistance = diagnostics.remaining_distance
    target.stopReference = diagnostics.stop_reference
    target.lightState = diagnostics.light_state
    target.sourceBus = diagnostics.source_bus
    target.quality = diagnostics.quality
    target.constraintAccel = diagnostics.traffic_a_target
    target.action = int(diagnostics.action)
    target.baseATarget = diagnostics.base_a_target
    target.finalATarget = diagnostics.final_a_target
    target.startRequested = diagnostics.start_requested
    target.startApplied = diagnostics.start_applied
    target.startBlockReason = int(diagnostics.start_block_reason)
    target.eventId = diagnostics.event_id
    target.terminalCatchActive = diagnostics.terminal_catch_active


def create_final_plan_arbitrator(CP, params) -> FinalPlanArbitrator | None:
  if CP.brand != "tesla" or not params.get_bool(TRAFFIC_SIGNAL_CONTROL_PARAM):
    return None
  return FinalPlanArbitrator(CP)
