from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import time

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.traffic_control import TRAFFIC_SIGNAL_CONTROL_PARAM
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlMode, TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.stop_profile import StopProfileGenerator


# CP's low-speed cruise acceleration envelope. This applies only to a
# same-event CAN-authoritative green start and remains time/speed bounded.
START_MAX_ACCEL = 1.6
START_MAX_SPEED = 2.5
START_MAX_DURATION_NS = 3_000_000_000
START_JERK_LIMIT = 1.0
START_NEAR_LEAD_DISTANCE = 8.0
START_LEAD_CONFIRM_NS = 500_000_000
START_LEAD_CLEAR_NS = 400_000_000
START_LEAD_MAX_GAP_NS = 150_000_000
MOVING_GREEN_SPEED = 0.3
TERMINAL_MAX_SPEED = 1.5
TERMINAL_LOOKAHEAD_S = 0.05
PLANNER_TRAFFIC_STALE_NS = 350_000_000
MAX_TRAFFIC_STOP_BRAKE = 3.0


@dataclass(frozen=True)
class _TrafficStopStyle:
  comfort_brake: float
  jerk_limit: float
  decel_margin: float


class TrafficPlanAction(IntEnum):
  none = 0
  stop = 1
  hold = 2
  start = 3
  release = 4
  rollingRelease = 5


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
  mode: int = 0
  action: TrafficPlanAction = TrafficPlanAction.none
  applied: bool = False
  start_requested: bool = False
  start_applied: bool = False
  start_block_reason: TrafficStartBlockReason = TrafficStartBlockReason.none
  event_id: int = 0
  stop_session_id: int = 0
  direction_unknown: bool = False
  driver_override_active: bool = False
  can_remaining: float = 0.0
  station_innovation: float = 0.0
  stop_control_allowed: bool = False
  stop_safety_allowed: bool = False
  raw_observation_fresh: bool = False
  raw_observation_age_ms: float = 0.0
  stop_direction_unknown: bool = False
  phase: int = int(TrafficControlPhase.off)
  light_state: int = 0
  remaining_distance: float = 0.0
  raw_distance: float = 255.0
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
    self._held_session_id = 0
    self._seen_stop_session_id = 0
    self._owned_stop_session_id = 0
    self._active_start_session_id = 0
    self._completed_start_session_id = 0
    self._start_started_ns = 0
    self._lead_delegated_session_id = 0
    self._lead_candidate_session_id = 0
    self._lead_candidate_since_ns = 0
    self._lead_candidate_last_ns = 0
    self._near_lead_blocked_session_id = 0
    self._lead_clear_since_ns = 0
    self._lead_clear_last_ns = 0
    self._was_stopping = False
    self._hold_latched = False
    self._hold_latched_should_stop = False
    self._armed_stop_session_id = 0
    self._rejected_stop_session_id = 0
    self.diagnostics = TrafficPlanDiagnostics()

  def _reset_control_state(self) -> None:
    self._held_event_id = 0
    self._held_session_id = 0
    self._seen_stop_session_id = 0
    self._owned_stop_session_id = 0
    self._active_start_session_id = 0
    self._completed_start_session_id = 0
    self._start_started_ns = 0
    self._lead_delegated_session_id = 0
    self._lead_candidate_session_id = 0
    self._lead_candidate_since_ns = 0
    self._lead_candidate_last_ns = 0
    self._near_lead_blocked_session_id = 0
    self._lead_clear_since_ns = 0
    self._lead_clear_last_ns = 0
    self._was_stopping = False
    self._hold_latched = False
    self._hold_latched_should_stop = False
    self._armed_stop_session_id = 0
    self._rejected_stop_session_id = 0
    self._profile.reset()

  @staticmethod
  def _plan_snapshot(plan):
    return (
      tuple(float(value) for value in plan.speeds),
      tuple(float(value) for value in plan.accels),
      tuple(float(value) for value in plan.jerks),
      float(plan.aTarget), bool(plan.shouldStop), bool(plan.allowThrottle),
    )

  @staticmethod
  def _plan_changed(before, plan) -> bool:
    after = FinalPlanArbitrator._plan_snapshot(plan)
    for previous, current in zip(before[:3], after[:3], strict=True):
      if len(previous) != len(current) or any(abs(a - b) > 1e-6 for a, b in zip(previous, current, strict=True)):
        return True
    return bool(
      abs(before[3] - after[3]) > 1e-6
      or before[4] != after[4]
      or before[5] != after[5]
    )

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

  @staticmethod
  def _driver_allows_stop(sm) -> bool:
    car_state = sm["carState"]
    car_control = sm["carControl"]
    return bool(
      car_control.enabled and car_control.longActive
      and not car_state.gasPressed and not car_state.brakePressed
    )

  @classmethod
  def _driver_allows_start(cls, sm) -> bool:
    return cls._driver_allows_stop(sm)

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
    self.diagnostics.stop_session_id = int(traffic.stopSessionId)
    self.diagnostics.direction_unknown = bool(traffic.directionUnknown)
    self.diagnostics.driver_override_active = bool(traffic.driverOverrideActive)
    self.diagnostics.can_remaining = float(traffic.canRemaining)
    self.diagnostics.station_innovation = float(traffic.stationInnovation)
    self.diagnostics.stop_control_allowed = bool(traffic.stopControlAllowed)
    self.diagnostics.stop_safety_allowed = bool(traffic.stopSafetyAllowed)
    self.diagnostics.raw_observation_fresh = bool(traffic.rawObservationFresh)
    self.diagnostics.raw_observation_age_ms = float(traffic.observationAgeMs)
    self.diagnostics.stop_direction_unknown = bool(traffic.stopDirectionUnknown)
    self.diagnostics.mode = int(traffic.mode)
    self.diagnostics.phase = int(traffic.phase)
    self.diagnostics.light_state = int(traffic.lightState)
    self.diagnostics.remaining_distance = max(0.0, float(traffic.distanceToStopPoint))
    self.diagnostics.raw_distance = float(traffic.rawDistance)
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

  def _traffic_stop_style(self, sm, *, remaining_distance: float, terminal: bool) -> _TrafficStopStyle:
    personality = sm["selfdriveState"].personality
    if personality == log.LongitudinalPersonality.relaxed:
      base_brake, base_jerk, decel_margin = 2.2, 0.55, 1.15
    elif personality == log.LongitudinalPersonality.aggressive:
      base_brake, base_jerk, decel_margin = 2.8, 1.10, 1.02
    else:
      base_brake, base_jerk, decel_margin = 2.5, 0.80, 1.08

    v_ego = max(0.0, float(sm["carState"].vEgo))
    effective_distance = max(
      remaining_distance - v_ego * (self._actuator_delay + TERMINAL_LOOKAHEAD_S),
      0.5,
    )
    required_brake = v_ego ** 2 / (2.0 * effective_distance)
    comfort_brake = float(np.clip(
      max(base_brake, required_brake * decel_margin),
      base_brake,
      MAX_TRAFFIC_STOP_BRAKE,
    ))
    # At higher approach speeds, begin changing acceleration more decisively.
    # Personality still shapes the ramp: relaxed is gentler, aggressive reacts
    # faster. Terminal catch keeps a safety floor independent of preference.
    speed_jerk_scale = float(np.interp(v_ego * 3.6, [0.0, 30.0, 60.0, 90.0], [0.70, 0.85, 1.10, 1.25]))
    jerk_limit = base_jerk * speed_jerk_scale
    if terminal:
      comfort_brake = MAX_TRAFFIC_STOP_BRAKE
      jerk_limit = max(jerk_limit, 1.0)
      decel_margin = max(decel_margin, 1.10)
    return _TrafficStopStyle(comfort_brake, jerk_limit, decel_margin)

  def _traffic_activation_distance(self, sm) -> float:
    """Use the same delayed, jerk-limited dynamics that generate the stop."""
    personality = sm["selfdriveState"].personality
    if personality == log.LongitudinalPersonality.relaxed:
      activation_brake, activation_jerk = 2.2, 0.55
    elif personality == log.LongitudinalPersonality.aggressive:
      activation_brake, activation_jerk = 2.8, 1.10
    else:
      activation_brake, activation_jerk = 2.5, 0.80
    v_ego = max(0.0, float(sm["carState"].vEgo))
    speed_jerk_scale = float(np.interp(v_ego * 3.6, [0.0, 30.0, 60.0, 90.0], [0.70, 0.85, 1.10, 1.25]))
    braking_distance = StopProfileGenerator.required_stop_distance(
      v_ego=v_ego, a_ego=float(sm["carState"].aEgo),
      actuator_delay=self._actuator_delay + 0.2,
      max_brake=activation_brake, jerk_limit=activation_jerk * speed_jerk_scale,
    )
    # Reserve enough distance for 2 Hz signal cadence, one confirmation frame,
    # and style-preserving control convergence before the physical stop model
    # reaches its nominal boundary.
    return float(np.clip(braking_distance + 26.0, 20.0, 200.0))

  def _traffic_stop_feasible(self, sm, remaining_distance: float) -> bool:
    v_ego = max(0.0, float(sm["carState"].vEgo))
    required_distance = StopProfileGenerator.required_stop_distance(
      v_ego=v_ego, a_ego=float(sm["carState"].aEgo),
      actuator_delay=self._actuator_delay + TERMINAL_LOOKAHEAD_S,
      max_brake=MAX_TRAFFIC_STOP_BRAKE, jerk_limit=1.1,
    )
    return remaining_distance >= required_distance

  def _apply_stop_constraint(self, plan, sm, *, remaining_distance: float,
                             hold: bool, terminal: bool) -> float:
    base_speeds = np.asarray(plan.speeds, dtype=float)
    base_accels = np.asarray(plan.accels, dtype=float)
    times = self._times(len(base_speeds))
    v_ego = float(sm["carState"].vEgo)
    style = self._traffic_stop_style(sm, remaining_distance=remaining_distance, terminal=terminal)
    stop_speeds, stop_accels, _ = self._profile.build_stop(
      v_ego=v_ego,
      a_ego=float(sm["carState"].aEgo),
      remaining_distance=remaining_distance,
      times=times,
      hold=hold,
      comfort_brake=style.comfort_brake,
      jerk_limit=style.jerk_limit,
      decel_margin=style.decel_margin,
    )
    final_speeds = np.minimum(base_speeds, stop_speeds)
    final_accels = np.minimum(base_accels, stop_accels)
    traffic_a_target = self._stop_target_accel(stop_accels, times, terminal=terminal)
    final_a_target = min(float(plan.aTarget), traffic_a_target)

    plan.speeds = final_speeds.tolist()
    plan.accels = final_accels.tolist()
    plan.jerks = self._padded_jerks(final_accels, times, len(plan.jerks)).tolist()
    plan.aTarget = final_a_target
    plan.shouldStop = bool(plan.shouldStop or hold)
    plan.allowThrottle = bool(plan.allowThrottle and not (hold or terminal))
    return traffic_a_target

  def _apply_stop(self, plan, sm, traffic, now_ns: int) -> None:
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

    session_id = int(traffic.stopSessionId)
    self._update_go_lead_gate(plan, sm, session_id, now_ns)

    if hold or terminal_stop:
      self._held_event_id = int(traffic.eventId)
      self._held_session_id = int(traffic.stopSessionId)
      self._active_start_session_id = 0
      self._completed_start_session_id = 0
      self._start_started_ns = 0
      self._hold_latched = True
      self._hold_latched_should_stop = self._hold_latched_should_stop or hold
    self._was_stopping = True
    self._owned_stop_session_id = int(traffic.stopSessionId)
    self.diagnostics.action = TrafficPlanAction.hold if hold else TrafficPlanAction.stop
    self.diagnostics.applied = True
    self.diagnostics.traffic_a_target = traffic_a_target
    self.diagnostics.terminal_catch_active = bool(terminal_stop or (hold and v_ego > 0.01))

  @staticmethod
  def _base_plan_lead_source(plan) -> int | None:
    source = getattr(
      plan, "longitudinalPlanSource", log.LongitudinalPlan.LongitudinalPlanSource.cruise,
    )
    source = int(getattr(source, "raw", source))
    return source if source in (
      int(log.LongitudinalPlan.LongitudinalPlanSource.lead0),
      int(log.LongitudinalPlan.LongitudinalPlanSource.lead1),
      int(log.LongitudinalPlan.LongitudinalPlanSource.lead2),
    ) else None

  @classmethod
  def _lead_gate_state(cls, plan, sm) -> tuple[bool, bool, bool]:
    lead_source = cls._base_plan_lead_source(plan)
    if "radarState" not in sm.seen or not cls._healthy(sm, "radarState"):
      return False, False, False
    radar_state = sm["radarState"]
    near = []
    for lead in (radar_state.leadOne, radar_state.leadTwo):
      if not bool(getattr(lead, "present", False)):
        near.append(False)
        continue
      d_rel = float(getattr(lead, "dRel", float("nan")))
      # A malformed current lead is uncertain and therefore blocks this GO
      # cycle, but it can never permanently own a stop session.
      near.append(not np.isfinite(d_rel) or d_rel <= 0.0 or d_rel <= START_NEAR_LEAD_DISTANCE)
    any_near = any(near)
    selected_near = bool(
      (lead_source == int(log.LongitudinalPlan.LongitudinalPlanSource.lead0) and near[0])
      or (lead_source == int(log.LongitudinalPlan.LongitudinalPlanSource.lead1) and near[1])
      or (lead_source == int(log.LongitudinalPlan.LongitudinalPlanSource.lead2) and any_near)
    )
    return True, any_near, selected_near

  def _update_go_lead_gate(self, plan, sm, session_id: int, now_ns: int) -> bool:
    tracked_ids = (
      self._lead_delegated_session_id, self._lead_candidate_session_id,
      self._near_lead_blocked_session_id,
    )
    if any(tracked not in (0, session_id) for tracked in tracked_ids):
      self._lead_delegated_session_id = 0
      self._lead_candidate_session_id = 0
      self._lead_candidate_since_ns = 0
      self._lead_candidate_last_ns = 0
      self._near_lead_blocked_session_id = 0
      self._lead_clear_since_ns = 0
      self._lead_clear_last_ns = 0

    healthy, any_near, selected_near = self._lead_gate_state(plan, sm)
    if not healthy:
      self._lead_clear_since_ns = 0
      self._lead_clear_last_ns = 0
      self._lead_candidate_since_ns = 0
      self._lead_candidate_last_ns = 0
      return False

    if any_near:
      self._near_lead_blocked_session_id = session_id
      self._lead_clear_since_ns = 0
      self._lead_clear_last_ns = 0
    elif self._near_lead_blocked_session_id == session_id:
      if (self._lead_clear_since_ns == 0 or self._lead_clear_last_ns == 0
          or now_ns - self._lead_clear_last_ns > START_LEAD_MAX_GAP_NS):
        self._lead_clear_since_ns = now_ns
      self._lead_clear_last_ns = now_ns
      if now_ns - self._lead_clear_since_ns >= START_LEAD_CLEAR_NS:
        self._near_lead_blocked_session_id = 0
        self._lead_clear_since_ns = 0
        self._lead_clear_last_ns = 0

    if selected_near:
      if (self._lead_candidate_session_id != session_id or self._lead_candidate_since_ns == 0
          or self._lead_candidate_last_ns == 0
          or now_ns - self._lead_candidate_last_ns > START_LEAD_MAX_GAP_NS):
        self._lead_candidate_session_id = session_id
        self._lead_candidate_since_ns = now_ns
      self._lead_candidate_last_ns = now_ns
      if now_ns - self._lead_candidate_since_ns >= START_LEAD_CONFIRM_NS:
        self._lead_delegated_session_id = session_id
    else:
      self._lead_candidate_session_id = 0
      self._lead_candidate_since_ns = 0
      self._lead_candidate_last_ns = 0
    return True

  def _go_lead_blocked(self, plan, sm, session_id: int, now_ns: int) -> bool:
    healthy = self._update_go_lead_gate(plan, sm, session_id, now_ns)
    return bool(
      not healthy
      or self._lead_delegated_session_id == session_id
      or self._near_lead_blocked_session_id == session_id
    )

  def _start_block_reason(self, plan, sm, traffic, now_ns: int) -> TrafficStartBlockReason:
    session_id = int(traffic.stopSessionId)
    if session_id == 0:
      return TrafficStartBlockReason.noPreviousHold
    if session_id not in (self._held_session_id, self._owned_stop_session_id, self._seen_stop_session_id):
      return TrafficStartBlockReason.eventMismatch
    if self._completed_start_session_id == session_id:
      return TrafficStartBlockReason.alreadyStarted
    if not self._driver_allows_start(sm):
      return TrafficStartBlockReason.driverOverride
    if self._go_lead_blocked(plan, sm, session_id, now_ns):
      return TrafficStartBlockReason.physicalLead
    # A same-event OEM CAN green is authoritative over a model/base-plan
    # traffic-stop residue, matching CP's e2eStopped -> e2eCruise transition.
    # Session, driver, direction, cruise, speed, and duration gates remain.
    v_cruise = float(sm["carState"].vCruise)
    if not 0.0 < v_cruise < V_CRUISE_UNSET:
      return TrafficStartBlockReason.invalidCruise
    return TrafficStartBlockReason.none

  def _finish_start(self, session_id: int) -> None:
    self._completed_start_session_id = session_id
    self._active_start_session_id = 0
    self._start_started_ns = 0

  def _apply_start(self, plan, sm, traffic, now_ns: int) -> bool:
    self.diagnostics.start_requested = True
    session_id = int(traffic.stopSessionId)
    block_reason = self._start_block_reason(plan, sm, traffic, now_ns)
    self.diagnostics.start_block_reason = block_reason
    if block_reason != TrafficStartBlockReason.none:
      if (block_reason != TrafficStartBlockReason.physicalLead
          and self._active_start_session_id == session_id):
        self._finish_start(session_id)
      return False

    self._hold_latched = False
    self._hold_latched_should_stop = False

    v_ego = float(sm["carState"].vEgo)
    if v_ego > START_MAX_SPEED:
      self._finish_start(session_id)
      return False
    if self._active_start_session_id == 0:
      self._active_start_session_id = session_id
      self._start_started_ns = now_ns
    elif self._active_start_session_id != session_id:
      return False
    if now_ns - self._start_started_ns > START_MAX_DURATION_NS:
      self._finish_start(session_id)
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
    v_ego = float(sm["carState"].vEgo)
    should_stop = self._hold_latched_should_stop or v_ego <= 0.3
    traffic_a_target = self._apply_stop_constraint(
      plan, sm,
      remaining_distance=0.0,
      hold=should_stop,
      terminal=True,
    )
    self._hold_latched_should_stop = should_stop
    self.diagnostics.action = TrafficPlanAction.hold
    self.diagnostics.applied = True
    self.diagnostics.event_id = self._held_event_id
    self.diagnostics.traffic_a_target = traffic_a_target
    self.diagnostics.terminal_catch_active = v_ego > 0.01

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
    base_snapshot = self._plan_snapshot(plan)
    base_a_target = float(plan.aTarget)
    self.diagnostics = TrafficPlanDiagnostics(base_a_target=base_a_target, final_a_target=base_a_target)
    traffic = self._traffic(sm, now_ns)
    self._set_diagnostics_from_traffic(traffic)

    if traffic is not None and int(traffic.mode) not in (
      int(TrafficControlMode.stopOnly), int(TrafficControlMode.stopGo),
    ):
      self._reset_control_state()
      self.diagnostics.final_a_target = float(plan.aTarget)
      self.diagnostics.should_stop = bool(plan.shouldStop)
      return

    event_passed = bool(
      traffic is not None and int(traffic.phase) == int(TrafficControlPhase.passed)
    )
    if event_passed:
      self._reset_control_state()
      self.diagnostics.final_a_target = float(plan.aTarget)
      self.diagnostics.should_stop = bool(plan.shouldStop)
      return

    driver_allows_stop = self._driver_allows_stop(sm)
    signal_release = bool(
      traffic is not None and int(traffic.stopSessionId) > 0
      and int(traffic.phase) == int(TrafficControlPhase.release) and int(traffic.lightState) == 2
    )
    confirmed_release = bool(signal_release and int(traffic.stopSessionId) == self._held_session_id)
    if confirmed_release:
      self._hold_latched = False
      self._hold_latched_should_stop = False
    elif not driver_allows_stop:
      self._hold_latched = False
      self._hold_latched_should_stop = False
      if self._active_start_session_id != 0:
        self._finish_start(self._active_start_session_id)
      self._was_stopping = False
      self._armed_stop_session_id = 0
      self._profile.reset()
      self.diagnostics.final_a_target = float(plan.aTarget)
      self.diagnostics.should_stop = bool(plan.shouldStop)
      return
    stop_phases = (
      int(TrafficControlPhase.approachRed), int(TrafficControlPhase.braking), int(TrafficControlPhase.hold),
      int(TrafficControlPhase.flashingGreenStop), int(TrafficControlPhase.yellowStop),
    )
    trackable_stop = bool(
      traffic is not None and traffic.targetPresent and float(traffic.confidence) >= 0.9
      and int(traffic.phase) in stop_phases
    )
    if trackable_stop:
      session_id = int(traffic.stopSessionId)
      self._seen_stop_session_id = int(traffic.stopSessionId)
      if self._rejected_stop_session_id not in (0, session_id):
        self._rejected_stop_session_id = 0
      inside_horizon = bool(
        int(traffic.phase) == int(TrafficControlPhase.hold)
        or float(traffic.distanceToStopPoint) <= self._traffic_activation_distance(sm)
      )
      feasible = self._traffic_stop_feasible(sm, float(traffic.distanceToStopPoint))
      if session_id not in (self._armed_stop_session_id, self._rejected_stop_session_id) and inside_horizon:
        if feasible and traffic.stopControlAllowed:
          self._armed_stop_session_id = session_id
        elif not feasible:
          # Make the ownership decision once. An initially impossible event
          # cannot become a surprise terminal catch only because the base
          # planner happened to slow the vehicle later.
          self._rejected_stop_session_id = session_id
    else:
      self._armed_stop_session_id = 0
    stale_armed_grace = bool(
      trackable_stop and int(traffic.stopSessionId) == self._armed_stop_session_id
      and traffic.stopSafetyAllowed and not traffic.rawObservationFresh
      and 0.0 <= float(traffic.observationAgeMs) <= 2000.0
      and float(traffic.rawDistance) < 255.0
    )
    active_stop = bool(
      trackable_stop and int(traffic.stopSessionId) == self._armed_stop_session_id
      and (traffic.stopControlAllowed or stale_armed_grace) and driver_allows_stop
    )
    if active_stop:
      self._apply_stop(plan, sm, traffic, now_ns)
    elif traffic is not None and bool(traffic.plannerStartRequested) and int(traffic.lightState) == 2:
      if float(sm["carState"].vEgo) > MOVING_GREEN_SPEED:
        self.diagnostics.start_requested = True
        if self._go_lead_blocked(plan, sm, int(traffic.stopSessionId), now_ns):
          self.diagnostics.start_block_reason = TrafficStartBlockReason.physicalLead
        self._finish_start(int(traffic.stopSessionId))
        self._hold_latched = False
        self._hold_latched_should_stop = False
        self._was_stopping = False
        self._armed_stop_session_id = 0
        self._profile.reset()
      elif not self._apply_start(plan, sm, traffic, now_ns):
        self._profile.reset()
        self._was_stopping = False
    elif signal_release:
      self._hold_latched = False
      self._hold_latched_should_stop = False
      self._was_stopping = False
      self._armed_stop_session_id = 0
      self._profile.reset()
    elif (self._hold_latched and float(sm["carState"].vEgo) <= TERMINAL_MAX_SPEED
          and driver_allows_stop):
      self._apply_latched_hold(plan, sm)
    else:
      same_release_start = bool(
        signal_release and traffic is not None
        and int(traffic.stopSessionId) == self._active_start_session_id
      )
      if self._active_start_session_id != 0 and not same_release_start:
        self._finish_start(self._active_start_session_id)
      self._apply_release(plan, sm)

    self.diagnostics.final_a_target = float(plan.aTarget)
    self.diagnostics.should_stop = bool(plan.shouldStop)
    if self.diagnostics.applied and not self._plan_changed(base_snapshot, plan):
      self.diagnostics.applied = False
      self.diagnostics.action = TrafficPlanAction.none
      self.diagnostics.start_applied = False

  def annotate_plan_sp(self, plan_sp) -> None:
    diagnostics = self.diagnostics
    plan_sp.aTarget = diagnostics.final_a_target
    target = plan_sp.teslaTrafficControl
    target.mode = diagnostics.mode
    target.phase = diagnostics.phase
    target.active = diagnostics.action != TrafficPlanAction.none
    target.shadow = False
    target.applied = diagnostics.applied
    target.shouldStop = diagnostics.should_stop
    target.remainingDistance = diagnostics.remaining_distance
    target.rawDistance = diagnostics.raw_distance
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
    target.stopSessionId = diagnostics.stop_session_id
    target.directionUnknown = diagnostics.direction_unknown
    target.driverOverrideActive = diagnostics.driver_override_active
    target.canRemaining = diagnostics.can_remaining
    target.stationInnovation = diagnostics.station_innovation
    target.stopControlAllowed = diagnostics.stop_control_allowed
    target.stopSafetyAllowed = diagnostics.stop_safety_allowed
    target.rawObservationFresh = diagnostics.raw_observation_fresh
    target.rawObservationAgeMs = diagnostics.raw_observation_age_ms
    target.stopDirectionUnknown = diagnostics.stop_direction_unknown


def create_final_plan_arbitrator(CP, params) -> FinalPlanArbitrator | None:
  if CP.brand != "tesla" or not params.get_bool(TRAFFIC_SIGNAL_CONTROL_PARAM):
    return None
  return FinalPlanArbitrator(CP)
