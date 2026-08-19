from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TeslaTrafficControlObservation


class TrafficControlMode(IntEnum):
  off = 0
  observe = 1
  shadow = 2
  stopOnly = 3
  stopGo = 4


class TrafficControlPhase(IntEnum):
  off = 0
  redCandidate = 1
  approachRed = 2
  braking = 3
  hold = 4
  goCandidate = 5
  release = 6
  bypass = 7


@dataclass
class TrafficControlConfig:
  mode: TrafficControlMode = TrafficControlMode.off
  default_stop_reference: float = 6.0
  adaptive_reference: bool = False
  comfort_brake: float = 2.4
  red_confirm_s: float = 0.5
  weak_red_confirm_s: float = 0.7
  replacement_confirm_s: float = 1.0
  model_confirm_s: float = 0.4
  green_confirm_s: float = 0.6
  release_s: float = 1.0
  bypass_s: float = 10.0
  observation_dropout_s: float = 0.75
  event_distance_tolerance: float = 12.0
  max_control_distance: float = 100.0
  candidate_distance_tolerance: float = 6.0
  candidate_distance_tolerance_ratio: float = 0.08
  model_alignment_min_m: float = 8.0
  model_alignment_max_m: float = 25.0
  model_alignment_ratio: float = 0.20


@dataclass(frozen=True)
class TrafficControlDecision:
  mode: TrafficControlMode
  phase: TrafficControlPhase
  active: bool
  apply_constraint: bool
  shadow: bool
  should_stop: bool
  remaining_distance: float
  stop_reference: float
  light_state: int
  source_bus: int
  quality: int


class TeslaTrafficControlController:
  """OEM-primary stop/go state machine with CP-inspired distance latching."""

  ACTIVE_PHASES = (TrafficControlPhase.approachRed, TrafficControlPhase.braking, TrafficControlPhase.hold)

  def __init__(self, config: TrafficControlConfig | None = None) -> None:
    self.config = config or TrafficControlConfig()
    self.transition_seq = 0
    self.transition_reason = ""
    self.last_distance_innovation = 0.0
    self.phase = TrafficControlPhase.off
    self.red_since_ns: int | None = None
    self.green_since_ns: int | None = None
    self.release_since_ns: int | None = None
    self.bypass_until_ns = 0
    self.last_update_ns: int | None = None
    self.remaining_distance = 0.0
    self.stop_reference = self.config.default_stop_reference
    self.light_state = 0
    self.source_bus = 0
    self.event_source_bus = 0
    self.event_control_source = 0
    self.candidate_source_bus = 0
    self.candidate_control_source = 0
    self.candidate_distance = 0.0
    self.candidate_last_ns: int | None = None
    self.candidate_anchor_distance = 0.0
    self.candidate_travel_distance = 0.0
    self.candidate_confirm_s = self.config.red_confirm_s
    self.pending_candidate_since_ns: int | None = None
    self.pending_candidate_distance = 0.0
    self.pending_candidate_last_ns: int | None = None
    self.pending_candidate_travel_distance = 0.0
    self.model_confirm_since_ns: int | None = None
    self.last_event_observation_ns: int | None = None
    self.quality = 0

  def _mark_transition(self, reason: str) -> None:
    self.transition_seq += 1
    self.transition_reason = reason

  def set_config(self, config: TrafficControlConfig) -> None:
    if config.mode == TrafficControlMode.off and self.config.mode != TrafficControlMode.off:
      self.reset()
    self.config = config

  def reset(self) -> None:
    self.phase = TrafficControlPhase.off
    self.red_since_ns = None
    self.green_since_ns = None
    self.release_since_ns = None
    self.remaining_distance = 0.0
    self.stop_reference = self.config.default_stop_reference
    self.light_state = 0
    self.source_bus = 0
    self.event_source_bus = 0
    self.event_control_source = 0
    self.candidate_source_bus = 0
    self.candidate_control_source = 0
    self.candidate_distance = 0.0
    self.candidate_last_ns = None
    self.candidate_anchor_distance = 0.0
    self.candidate_travel_distance = 0.0
    self.candidate_confirm_s = self.config.red_confirm_s
    self.pending_candidate_since_ns = None
    self.pending_candidate_distance = 0.0
    self.pending_candidate_last_ns = None
    self.pending_candidate_travel_distance = 0.0
    self.model_confirm_since_ns = None
    self.last_distance_innovation = 0.0
    self.last_event_observation_ns = None
    self.quality = 0
    self.last_update_ns = None

  def _decision(self) -> TrafficControlDecision:
    active = self.phase in self.ACTIVE_PHASES or self.phase == TrafficControlPhase.release
    # shouldStop switches longcontrol out of its normal PID into the fixed
    # standstill policy. Keep approach/braking on the planned acceleration
    # profile and request the standstill policy only for the final hold.
    stopping = self.phase == TrafficControlPhase.hold
    return TrafficControlDecision(
      mode=self.config.mode,
      phase=self.phase,
      active=active,
      apply_constraint=self.config.mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and active,
      shadow=self.config.mode == TrafficControlMode.shadow and active,
      should_stop=stopping,
      remaining_distance=max(0.0, self.remaining_distance),
      stop_reference=self.stop_reference,
      light_state=self.light_state,
      source_bus=self.source_bus,
      quality=self.quality,
    )

  def _update_reference(self, observation: TeslaTrafficControlObservation,
                        model_stop_distance: float | None, model_stop_candidate: bool) -> None:
    if not self.config.adaptive_reference or not model_stop_candidate or model_stop_distance is None:
      return
    estimate = observation.distance - model_stop_distance
    if 2.0 <= estimate <= 12.0:
      self.stop_reference = 0.75 * self.stop_reference + 0.25 * estimate

  def _model_confirms_stop(self, observation: TeslaTrafficControlObservation,
                           model_stop_distance: float | None, model_stop_candidate: bool) -> bool:
    if not model_stop_candidate or model_stop_distance is None:
      return False
    expected_stop_distance = max(0.0, observation.distance - self.stop_reference)
    tolerance = float(np.clip(
      observation.distance * self.config.model_alignment_ratio,
      self.config.model_alignment_min_m,
      self.config.model_alignment_max_m,
    ))
    return abs(model_stop_distance - expected_stop_distance) <= tolerance

  def _start_stop(self, observation: TeslaTrafficControlObservation, v_ego: float,
                  model_stop_distance: float | None) -> None:
    # Distant visual estimates are useful for building confidence but are too
    # unstable and lack lane identity. Never let them constrain the planner.
    if observation.distance > self.config.max_control_distance:
      return
    self.event_source_bus = observation.source_bus
    self.event_control_source = observation.control_source
    self.last_event_observation_ns = self.last_update_ns
    # CP's e2e stop state commits the model stop distance, while the traffic
    # light detector decides whether that stop is a traffic-control event. Use
    # the same split: Tesla CAN confirms red and event identity; the aligned CP
    # model target supplies the primary stopping point.
    tesla_remaining = max(0.0, observation.distance - self.stop_reference)
    self.remaining_distance = (tesla_remaining if model_stop_distance is None else
                               max(0.0, float(model_stop_distance)))
    # A stable target may begin constraining as it crosses 100 m. The stop
    # profile still derives its acceleration from current speed and remaining
    # distance; this boundary only prevents distant visual estimates from
    # touching the planner.
    activation_control_distance = self.config.max_control_distance
    if observation.distance <= activation_control_distance:
      required = v_ego ** 2 / (2.0 * max(self.remaining_distance, 0.5))
      self.phase = TrafficControlPhase.braking if required >= 0.5 else TrafficControlPhase.approachRed
      self._mark_transition("stop_confirmed")

  def _advance_latched_distance(self, dt: float, v_ego: float,
                                observation: TeslaTrafficControlObservation) -> None:
    self.remaining_distance = max(0.0, self.remaining_distance - max(v_ego, 0.0) * dt)
    same_event = (observation.source_bus == self.event_source_bus and
                  observation.control_source == self.event_control_source)
    expected_distance = self.remaining_distance + self.stop_reference
    distance_consistent = abs(observation.distance - expected_distance) <= self.config.event_distance_tolerance
    if observation.valid_for_control and observation.light_state in (1, 3) and same_event and distance_consistent:
      self.last_event_observation_ns = self.last_update_ns
      oem_remaining = max(0.0, observation.distance - self.stop_reference)
      # OEM remains primary, but never allow a noisy target switch to move the
      # committed stop point materially farther away in one planner cycle.
      correction = float(np.clip(oem_remaining - self.remaining_distance, -2.0, 0.25 * dt))
      self.remaining_distance = max(0.0, self.remaining_distance + correction)

  def update(self, observation: TeslaTrafficControlObservation, now_ns: int, *, v_ego: float, a_ego: float,
             model_stop_distance: float | None, model_stop_candidate: bool, lead_present: bool,
             radar_valid: bool, enabled: bool, long_active: bool, gas_pressed: bool,
             brake_pressed: bool, turn_signal_active: bool) -> TrafficControlDecision:
    del a_ego
    dt = 0.0 if self.last_update_ns is None else max(0.0, min((now_ns - self.last_update_ns) / 1e9, 0.5))
    self.last_update_ns = now_ns

    if self.config.mode == TrafficControlMode.off:
      self.reset()
      return self._decision()

    if gas_pressed:
      entering_bypass = self.phase != TrafficControlPhase.bypass
      self.phase = TrafficControlPhase.bypass
      self.bypass_until_ns = now_ns + int(self.config.bypass_s * 1e9)
      self.red_since_ns = None
      self.green_since_ns = None
      if entering_bypass:
        self._mark_transition("driver_bypass")
      return self._decision()

    if self.phase == TrafficControlPhase.bypass:
      if now_ns < self.bypass_until_ns:
        return self._decision()
      self.reset()
      self.last_update_ns = now_ns

    if not enabled or (self.config.mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and not long_active):
      self.reset()
      return self._decision()

    # "No lead" must be positively established before traffic-control data is
    # allowed to constrain longitudinal planning. If a lead appears, release
    # this independent constraint and leave following entirely to the base
    # planner. Invalid radar is treated conservatively as unknown, not no lead.
    if self.config.mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and (not radar_valid or lead_present):
      was_tracking = self.phase != TrafficControlPhase.off
      self.reset()
      self.last_update_ns = now_ns
      if was_tracking:
        self._mark_transition("lead_present" if lead_present else "radar_invalid")
      return self._decision()

    valid_red = observation.valid_for_control and observation.light_state == 1
    valid_green = observation.valid_for_control and observation.light_state == 2
    valid_yellow = observation.valid_for_control and observation.light_state == 3
    if observation.available:
      self.light_state = observation.light_state
      self.source_bus = observation.source_bus
      self.quality = observation.quality

    if self.phase in self.ACTIVE_PHASES:
      self._advance_latched_distance(dt, v_ego, observation)

      observation_dropout_s = (float("inf") if self.last_event_observation_ns is None else
                               (now_ns - self.last_event_observation_ns) / 1e9)
      # A noisy distance jump must not become irrevocable merely because it
      # landed close to the nominal stop point. Only an actual stationary hold
      # survives loss of the traffic-control observation.
      committed = self.phase == TrafficControlPhase.hold
      if observation_dropout_s >= self.config.observation_dropout_s and not committed:
        self.reset()
        self.last_update_ns = now_ns
        self._mark_transition("observation_dropout")
        return self._decision()

      if v_ego < 0.3 and self.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking):
        self.phase = TrafficControlPhase.hold
        self._mark_transition("stationary_hold")

      expected_distance = self.remaining_distance + self.stop_reference
      same_event_green = (valid_green and observation.source_bus == self.event_source_bus and
                          observation.control_source == self.event_control_source and
                          abs(observation.distance - expected_distance) <= self.config.event_distance_tolerance)
      if same_event_green:
        self.last_event_observation_ns = now_ns
        if self.green_since_ns is None:
          self.green_since_ns = now_ns
        green_stable = (now_ns - self.green_since_ns) / 1e9 >= self.config.green_confirm_s
        moving_release = v_ego >= 0.3 and not brake_pressed
        stopped_release = (self.config.mode == TrafficControlMode.stopGo and radar_valid and not lead_present and
                           not turn_signal_active and not brake_pressed)
        if green_stable and (moving_release or stopped_release):
          self.phase = TrafficControlPhase.release
          self.release_since_ns = now_ns
          self.remaining_distance = 0.0
          self._mark_transition("green_release")
      else:
        self.green_since_ns = None

      if self.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking) and self.remaining_distance > 0.0:
        required = v_ego ** 2 / (2.0 * max(self.remaining_distance, 0.5))
        self.phase = TrafficControlPhase.braking if required >= 0.5 else TrafficControlPhase.approachRed
      return self._decision()

    if self.phase == TrafficControlPhase.release:
      if valid_red:
        self.phase = TrafficControlPhase.redCandidate
        self.red_since_ns = now_ns
        self.release_since_ns = None
      elif self.release_since_ns is not None and (now_ns - self.release_since_ns) / 1e9 >= self.config.release_s:
        self.reset()
        self.last_update_ns = now_ns
      return self._decision()

    if valid_red or valid_yellow:
      candidate_dt = (0.0 if self.candidate_last_ns is None else
                      max(0.0, (now_ns - self.candidate_last_ns) / 1e9))
      expected_distance = max(0.0, self.candidate_distance - max(v_ego, 0.0) * candidate_dt)
      projected_travel = self.candidate_travel_distance + max(v_ego, 0.0) * candidate_dt
      anchor_expected_distance = max(0.0, self.candidate_anchor_distance - projected_travel)
      distance_tolerance = max(
        self.config.candidate_distance_tolerance,
        self.config.candidate_distance_tolerance_ratio * max(self.candidate_distance, observation.distance),
      )
      anchor_tolerance = max(
        self.config.candidate_distance_tolerance,
        self.config.candidate_distance_tolerance_ratio * self.candidate_anchor_distance,
      )
      same_candidate = (observation.source_bus == self.candidate_source_bus and
                        observation.control_source == self.candidate_control_source and
                        abs(observation.distance - expected_distance) <= distance_tolerance and
                        abs(observation.distance - anchor_expected_distance) <= anchor_tolerance)
      self.last_distance_innovation = 0.0 if self.red_since_ns is None else observation.distance - expected_distance
      same_source = (observation.source_bus == self.candidate_source_bus and
                     observation.control_source == self.candidate_control_source)

      # Route 00000003--5d11656108 segments 5/6 shows the same stationary
      # AP-PARTY red target quantizing by 7-8 m. A single distance innovation
      # is not a new event identity: require the alternative trajectory to
      # remain motion-consistent for the already-defined replacement window.
      if self.red_since_ns is not None and same_source and not same_candidate:
        pending_dt = (0.0 if self.pending_candidate_last_ns is None else
                      max(0.0, (now_ns - self.pending_candidate_last_ns) / 1e9))
        pending_expected = max(0.0, self.pending_candidate_distance - max(v_ego, 0.0) * pending_dt)
        pending_tolerance = max(
          self.config.candidate_distance_tolerance,
          self.config.candidate_distance_tolerance_ratio * max(self.pending_candidate_distance, observation.distance),
        )
        pending_consistent = (self.pending_candidate_since_ns is not None and
                              abs(observation.distance - pending_expected) <= pending_tolerance)
        if not pending_consistent:
          self.pending_candidate_since_ns = now_ns
          self.pending_candidate_travel_distance = 0.0
        else:
          self.pending_candidate_travel_distance += max(v_ego, 0.0) * pending_dt
        self.pending_candidate_distance = observation.distance
        self.pending_candidate_last_ns = now_ns
        self.candidate_last_ns = now_ns
        replacement_stable = ((now_ns - self.pending_candidate_since_ns) / 1e9 >=
                              self.config.replacement_confirm_s)
        if not replacement_stable:
          self.phase = TrafficControlPhase.redCandidate
          return self._decision()
        # Commit only a sustained alternative trajectory, then run the normal
        # candidate/model confirmation from a clean event anchor below.
        same_candidate = False
      elif same_candidate:
        self.pending_candidate_since_ns = None
        self.pending_candidate_distance = 0.0
        self.pending_candidate_last_ns = None
        self.pending_candidate_travel_distance = 0.0

      if self.red_since_ns is None or not same_candidate:
        replacement = self.red_since_ns is not None
        self.red_since_ns = now_ns
        self.stop_reference = self.config.default_stop_reference
        self.source_bus = observation.source_bus
        self.candidate_source_bus = observation.source_bus
        self.candidate_control_source = observation.control_source
        self.candidate_anchor_distance = observation.distance
        self.candidate_travel_distance = 0.0
        base_confirm_s = self.config.red_confirm_s if observation.quality >= 2 else self.config.weak_red_confirm_s
        self.candidate_confirm_s = max(base_confirm_s, self.config.replacement_confirm_s if replacement else 0.0)
        self.model_confirm_since_ns = None
        self.pending_candidate_since_ns = None
        self.pending_candidate_distance = 0.0
        self.pending_candidate_last_ns = None
        self.pending_candidate_travel_distance = 0.0
        self._mark_transition("candidate_replaced" if replacement else "candidate_started")
      else:
        self.candidate_travel_distance = projected_travel
      self.candidate_distance = observation.distance
      self.candidate_last_ns = now_ns
      self.phase = TrafficControlPhase.redCandidate
      self._update_reference(observation, model_stop_distance, model_stop_candidate)
      model_confirms = self._model_confirms_stop(observation, model_stop_distance, model_stop_candidate)
      if model_confirms:
        # plannerd calls this adapter only when modelV2 is updated. The adapter
        # separately requires the fresh message to be valid before it reaches
        # this state machine.
        if self.model_confirm_since_ns is None:
          self.model_confirm_since_ns = now_ns
      else:
        self.model_confirm_since_ns = None
      oem_confirmed = (now_ns - self.red_since_ns) / 1e9 >= self.candidate_confirm_s
      model_confirmed = (self.model_confirm_since_ns is not None and
                         (now_ns - self.model_confirm_since_ns) / 1e9 >= self.config.model_confirm_s)
      confirmed = oem_confirmed and model_confirmed
      if valid_yellow:
        remaining = max(0.5, observation.distance - self.stop_reference)
        required = v_ego ** 2 / (2.0 * remaining)
        confirmed = confirmed and required <= self.config.comfort_brake
      if confirmed:
        self._start_stop(observation, v_ego, model_stop_distance)
      return self._decision()

    if self.phase == TrafficControlPhase.redCandidate and self.candidate_last_ns is not None:
      candidate_dropout_s = (now_ns - self.candidate_last_ns) / 1e9
      if candidate_dropout_s < self.config.observation_dropout_s:
        return self._decision()

    candidate_cancelled = self.phase == TrafficControlPhase.redCandidate
    self.red_since_ns = None
    self.candidate_source_bus = 0
    self.candidate_control_source = 0
    self.candidate_distance = 0.0
    self.candidate_last_ns = None
    self.candidate_anchor_distance = 0.0
    self.candidate_travel_distance = 0.0
    self.candidate_confirm_s = self.config.red_confirm_s
    self.pending_candidate_since_ns = None
    self.pending_candidate_distance = 0.0
    self.pending_candidate_last_ns = None
    self.pending_candidate_travel_distance = 0.0
    self.model_confirm_since_ns = None
    self.phase = TrafficControlPhase.off
    if candidate_cancelled:
      self._mark_transition("candidate_cancelled")
    return self._decision()
