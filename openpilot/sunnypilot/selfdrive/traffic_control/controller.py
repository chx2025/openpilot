from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import (
  TRAFFIC_CONTROL_MAX_DISTANCE,
  TeslaTrafficControlObservation,
)


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
  greenFlashCandidate = 8
  flashingGreenStop = 9
  yellowStop = 10
  yellowPass = 11
  lateRed = 12
  passed = 13


@dataclass
class TrafficControlConfig:
  mode: TrafficControlMode = TrafficControlMode.off
  default_stop_reference: float = 5.0
  adaptive_reference: bool = False
  comfort_brake: float = 2.4
  red_confirm_s: float = 0.5
  weak_red_confirm_s: float = 0.7
  replacement_confirm_s: float = 1.0
  model_confirm_s: float = 0.4
  green_confirm_s: float = 0.0
  yellow_green_confirm_s: float = 0.6
  release_s: float = 3.0
  bypass_s: float = 10.0
  observation_dropout_s: float = 0.75
  candidate_dropout_s: float = 2.5
  event_distance_tolerance: float = 12.0
  max_control_distance: float = TRAFFIC_CONTROL_MAX_DISTANCE
  candidate_distance_tolerance: float = 6.0
  candidate_distance_tolerance_ratio: float = 0.08
  model_alignment_min_m: float = 8.0
  model_alignment_max_m: float = 25.0
  model_alignment_ratio: float = 0.20
  model_only_min_speed: float = 1.0
  retain_event_with_lead: bool = False
  max_control_speed: float = float("inf")
  final_distance_freeze_m: float = 10.0
  yellow_stop_decel: float = 2.2
  yellow_pass_decel: float = 3.0
  flash_interval_min_s: float = 0.5
  flash_interval_max_s: float = 1.5


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
  """CAN-color traffic-light state machine with a CP-style latched stop point."""

  ACTIVE_PHASES = (
    TrafficControlPhase.approachRed,
    TrafficControlPhase.braking,
    TrafficControlPhase.hold,
    TrafficControlPhase.flashingGreenStop,
    TrafficControlPhase.yellowStop,
  )

  def __init__(self, config: TrafficControlConfig | None = None) -> None:
    self.config = config or TrafficControlConfig()
    self.transition_seq = 0
    self.transition_reason = ""
    self.event_seq = 0
    self.event_id = 0
    self.phase = TrafficControlPhase.off
    self.last_update_ns: int | None = None
    self.last_real_frame_ns = 0
    self.last_real_color = 0
    self.last_raw_distance: float | None = None
    self.last_distance_ego_station = 0.0
    self.ego_station = 0.0
    self.stop_station: float | None = None
    self.station_samples: deque[float] = deque(maxlen=3)
    self.remaining_distance = 0.0
    self.stop_reference = self.config.default_stop_reference
    self.light_state = 0
    self.source_bus = 0
    self.quality = 0
    self.event_source_bus = 0
    self.event_control_source = 0
    self.event_continuous = False
    self.candidate_color = 0
    self.candidate_count = 0
    self.candidate_last_ns = 0
    self.candidate_distance = 0.0
    self.pending_distance = 0.0
    self.pending_ego_station = 0.0
    self.pending_color = 0
    self.pending_count = 0
    self.green_count = 0
    self.first_off_ns = 0
    self.green_between_off = False
    self.flash_latched = False
    self.yellow_latched: bool | None = None
    self.release_since_ns: int | None = None
    self.bypass_until_ns = 0
    self.last_distance_innovation = 0.0

  def _mark_transition(self, reason: str) -> None:
    self.transition_seq += 1
    self.transition_reason = reason

  def set_config(self, config: TrafficControlConfig) -> None:
    previous_mode = self.config.mode
    self.config = config
    if config.mode == TrafficControlMode.off and previous_mode != TrafficControlMode.off:
      self.reset()
    elif self.phase == TrafficControlPhase.off:
      self.stop_reference = config.default_stop_reference

  def reset(self) -> None:
    self.phase = TrafficControlPhase.off
    self.event_id = 0
    self.last_real_frame_ns = 0
    self.last_real_color = 0
    self.last_raw_distance = None
    self.last_distance_ego_station = self.ego_station
    self.stop_station = None
    self.station_samples.clear()
    self.remaining_distance = 0.0
    self.stop_reference = self.config.default_stop_reference
    self.light_state = 0
    self.source_bus = 0
    self.quality = 0
    self.event_source_bus = 0
    self.event_control_source = 0
    self.event_continuous = False
    self.candidate_color = 0
    self.candidate_count = 0
    self.candidate_last_ns = 0
    self.candidate_distance = 0.0
    self.pending_distance = 0.0
    self.pending_ego_station = self.ego_station
    self.pending_color = 0
    self.pending_count = 0
    self.green_count = 0
    self.first_off_ns = 0
    self.green_between_off = False
    self.flash_latched = False
    self.yellow_latched = None
    self.release_since_ns = None
    self.last_distance_innovation = 0.0
    self.last_update_ns = None

  def _decision(self) -> TrafficControlDecision:
    active = self.phase in self.ACTIVE_PHASES or self.phase == TrafficControlPhase.release
    return TrafficControlDecision(
      mode=self.config.mode,
      phase=self.phase,
      active=active,
      apply_constraint=self.config.mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and active,
      shadow=self.config.mode == TrafficControlMode.shadow and active,
      should_stop=self.phase == TrafficControlPhase.hold,
      remaining_distance=max(0.0, self.remaining_distance),
      stop_reference=self.stop_reference,
      light_state=self.light_state,
      source_bus=self.source_bus,
      quality=self.quality,
    )

  def _distance_tolerance(self, a: float, b: float) -> float:
    return max(self.config.candidate_distance_tolerance,
               self.config.candidate_distance_tolerance_ratio * max(a, b))

  def _same_track(self, observation: TeslaTrafficControlObservation) -> bool:
    if self.last_raw_distance is None:
      return True
    expected = max(0.0, self.last_raw_distance - (self.ego_station - self.last_distance_ego_station))
    innovation = observation.distance - expected
    self.last_distance_innovation = innovation
    return bool(observation.source_bus == self.source_bus and
                abs(innovation) <= self._distance_tolerance(observation.distance, expected))

  def _update_stop_station(self, observation: TeslaTrafficControlObservation) -> None:
    sample = self.ego_station + observation.distance - self.stop_reference
    self.station_samples.append(sample)
    filtered = float(np.median(np.asarray(self.station_samples, dtype=float)))
    if self.stop_station is None:
      self.stop_station = filtered
    elif self.remaining_distance > self.config.final_distance_freeze_m:
      # Distance quantization must not move a committed stop target away from
      # the car by meters at a time. Closing corrections are accepted; outward
      # corrections are bounded to five centimetres per real CAN sample.
      self.stop_station = min(filtered, self.stop_station + 0.05)
    self.remaining_distance = max(0.0, (self.stop_station or self.ego_station) - self.ego_station)

  def _start_stop(self, observation: TeslaTrafficControlObservation, v_ego: float,
                  phase: TrafficControlPhase, reason: str) -> None:
    self.event_seq += 1
    self.event_id = self.event_seq
    self.event_source_bus = observation.source_bus
    self.event_control_source = observation.control_source
    self.stop_reference = self.config.default_stop_reference
    # Every confirmed event owns fresh world geometry. In particular, a red
    # after release must never inherit the previous stop station, which is now
    # at or behind the ego position.
    self.station_samples.clear()
    self.stop_station = None
    self.remaining_distance = 0.0
    self._update_stop_station(observation)
    required = v_ego ** 2 / (2.0 * max(self.remaining_distance, 0.5))
    self.phase = (TrafficControlPhase.braking if required >= 0.5 else TrafficControlPhase.approachRed) \
      if phase == TrafficControlPhase.approachRed else phase
    self._mark_transition(reason)

  def _set_release(self, now_ns: int) -> None:
    self.phase = TrafficControlPhase.release
    self.release_since_ns = now_ns
    self.remaining_distance = 0.0
    self._mark_transition("green_release")

  def _handle_flash(self, observation: TeslaTrafficControlObservation, now_ns: int, v_ego: float) -> bool:
    color = observation.light_state
    if color == 0 and self.last_real_color == 2:
      if self.first_off_ns and self.green_between_off:
        interval_s = (now_ns - self.first_off_ns) / 1e9
        if self.config.flash_interval_min_s <= interval_s <= self.config.flash_interval_max_s:
          self.flash_latched = True
          if self.phase not in self.ACTIVE_PHASES:
            self._start_stop(observation, v_ego, TrafficControlPhase.flashingGreenStop, "flashing_green_stop")
          else:
            self.phase = TrafficControlPhase.flashingGreenStop
          return True
      self.first_off_ns = now_ns
      self.green_between_off = False
      if self.phase not in self.ACTIVE_PHASES:
        self.phase = TrafficControlPhase.greenFlashCandidate
        self._mark_transition("green_flash_candidate")
    elif color == 2 and self.first_off_ns:
      self.green_between_off = True
    elif color in (1, 3):
      self.first_off_ns = 0
      self.green_between_off = False
    return False

  def _update_candidate(self, observation: TeslaTrafficControlObservation) -> bool:
    same = bool(
      self.candidate_count > 0
      and self.candidate_color == observation.light_state
      and abs(observation.distance - self.candidate_distance) <=
      self._distance_tolerance(observation.distance, self.candidate_distance)
    )
    if same:
      self.candidate_count += 1
    else:
      self.candidate_color = observation.light_state
      self.candidate_count = 1
    self.candidate_distance = observation.distance
    self.candidate_last_ns = observation.frame_mono_time
    return self.candidate_count >= 2

  def _pending_replacement_confirmed(self, observation: TeslaTrafficControlObservation) -> bool:
    expected = max(0.0, self.pending_distance - (self.ego_station - self.pending_ego_station))
    same = bool(
      self.pending_count > 0
      and self.pending_color == observation.light_state
      and abs(observation.distance - expected) <= self._distance_tolerance(observation.distance, expected)
    )
    self.pending_count = self.pending_count + 1 if same else 1
    self.pending_distance = observation.distance
    self.pending_ego_station = self.ego_station
    self.pending_color = observation.light_state
    return self.pending_count >= 2

  def update(self, observation: TeslaTrafficControlObservation, now_ns: int, *, v_ego: float, a_ego: float,
             model_stop_distance: float | None, model_stop_candidate: bool, lead_present: bool,
             radar_valid: bool, enabled: bool, long_active: bool, gas_pressed: bool,
             brake_pressed: bool, turn_signal_active: bool) -> TrafficControlDecision:
    del a_ego, model_stop_distance, model_stop_candidate, lead_present, turn_signal_active
    dt = 0.0 if self.last_update_ns is None else max(0.0, min((now_ns - self.last_update_ns) / 1e9, 0.5))
    self.last_update_ns = now_ns
    self.ego_station += max(0.0, v_ego) * dt
    if self.stop_station is not None:
      self.remaining_distance = max(0.0, self.stop_station - self.ego_station)
    self.event_continuous = False

    if self.config.mode == TrafficControlMode.off:
      self.reset()
      return self._decision()
    if gas_pressed:
      entering = self.phase != TrafficControlPhase.bypass
      self.phase = TrafficControlPhase.bypass
      self.bypass_until_ns = now_ns + int(self.config.bypass_s * 1e9)
      if entering:
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
    if self.config.mode in (TrafficControlMode.stopOnly, TrafficControlMode.stopGo) and not radar_valid:
      return self._decision()
    if (self.phase not in (*self.ACTIVE_PHASES, TrafficControlPhase.release)
        and v_ego > self.config.max_control_speed):
      self.candidate_count = 0
      self.phase = TrafficControlPhase.off
      return self._decision()

    new_frame = bool(observation.available and observation.frame_mono_time > 0 and
                     observation.frame_mono_time != self.last_real_frame_ns and
                     observation.source_bus == 2 and observation.dlc >= 6 and
                     observation.control_type == 3 and observation.light_state in (0, 1, 2, 3))
    if not new_frame:
      if self.phase in (TrafficControlPhase.redCandidate, TrafficControlPhase.greenFlashCandidate):
        if self.candidate_last_ns and now_ns - self.candidate_last_ns > int(self.config.candidate_dropout_s * 1e9):
          self.phase = TrafficControlPhase.off
          self.candidate_count = 0
      if v_ego < 0.3 and self.phase in self.ACTIVE_PHASES and self.remaining_distance <= 1.5:
        self.phase = TrafficControlPhase.hold
      return self._decision()

    previous_color = self.last_real_color
    self.last_real_frame_ns = observation.frame_mono_time
    self.light_state = observation.light_state
    self.source_bus = observation.source_bus
    self.quality = observation.quality

    if observation.distance > self.config.max_control_distance:
      wrapped = bool(self.last_raw_distance is not None and self.last_raw_distance <= 2.0 and observation.distance >= 250.0)
      if wrapped:
        self.phase = TrafficControlPhase.passed
        self.remaining_distance = 0.0
        self._mark_transition("distance_wrap_passed")
      self.last_real_color = observation.light_state
      return self._decision()

    same_track = self._same_track(observation)
    self.event_continuous = same_track
    if not same_track and self.phase in self.ACTIVE_PHASES:
      # Adjacent signals frequently share the same CAN stream. A single
      # discontinuous tuple cannot replace or release the current event.
      # Only a motion-consistent two-frame red/yellow trajectory is promoted.
      replace = observation.light_state in (1, 3) and self._pending_replacement_confirmed(observation)
      if not replace:
        self.last_real_color = observation.light_state
        return self._decision()
      self.station_samples.clear()
      self.stop_station = None
      self.last_raw_distance = observation.distance
      self.last_distance_ego_station = self.ego_station
      replacement_phase = (TrafficControlPhase.yellowStop if observation.light_state == 3
                           else TrafficControlPhase.approachRed)
      self._start_stop(observation, v_ego, replacement_phase, "candidate_replaced")
      self.pending_count = 0
      self.last_real_color = observation.light_state
      return self._decision()
    if same_track:
      self.pending_count = 0

    self.last_raw_distance = observation.distance
    self.last_distance_ego_station = self.ego_station
    self._update_stop_station(observation)

    self.last_real_color = previous_color
    if self._handle_flash(observation, now_ns, v_ego):
      self.last_real_color = observation.light_state
      return self._decision()

    color = observation.light_state
    confirmed = self._update_candidate(observation)

    if self.phase in self.ACTIVE_PHASES:
      if self.flash_latched:
        self.phase = (TrafficControlPhase.hold if v_ego < 0.3 and self.remaining_distance <= 1.5
                      else TrafficControlPhase.flashingGreenStop)
      elif color == 2:
        self.green_count = self.green_count + 1 if previous_color == 2 else 1
        if self.config.mode == TrafficControlMode.stopGo and self.green_count >= 2 and not brake_pressed:
          self._set_release(now_ns)
      else:
        self.green_count = 0
        if color == 3:
          self.phase = TrafficControlPhase.yellowStop
        elif color == 1 and self.phase != TrafficControlPhase.hold:
          required = v_ego ** 2 / (2.0 * max(self.remaining_distance, 0.5))
          self.phase = TrafficControlPhase.braking if required >= 0.5 else TrafficControlPhase.approachRed
      if v_ego < 0.3 and self.remaining_distance <= 1.5 and self.phase != TrafficControlPhase.release:
        self.phase = TrafficControlPhase.hold
        self._mark_transition("stationary_hold")
      self.last_real_color = observation.light_state
      return self._decision()

    if self.phase == TrafficControlPhase.release:
      if color == 1 and confirmed:
        self._start_stop(observation, v_ego, TrafficControlPhase.approachRed, "red_after_release")
      elif self.release_since_ns is not None and now_ns - self.release_since_ns >= int(self.config.release_s * 1e9):
        self.reset()
        self.last_update_ns = now_ns
      self.last_real_color = observation.light_state
      return self._decision()

    if color == 1:
      self.phase = TrafficControlPhase.redCandidate
      if confirmed:
        self._start_stop(observation, v_ego, TrafficControlPhase.approachRed, "stop_confirmed")
    elif color == 3:
      required = v_ego ** 2 / (2.0 * max(observation.distance - self.stop_reference, 0.5))
      if self.yellow_latched is None and confirmed:
        if required <= self.config.yellow_stop_decel:
          self.yellow_latched = True
        elif required >= self.config.yellow_pass_decel:
          self.yellow_latched = False
        else:
          self.yellow_latched = required <= self.config.comfort_brake
      if self.yellow_latched is True:
        self._start_stop(observation, v_ego, TrafficControlPhase.yellowStop, "yellow_stop")
      elif self.yellow_latched is False:
        self.phase = TrafficControlPhase.yellowPass
        self._mark_transition("yellow_pass")
      else:
        self.phase = TrafficControlPhase.redCandidate
    elif color == 2:
      self.green_count = self.green_count + 1 if previous_color == 2 else 1
      if self.phase != TrafficControlPhase.greenFlashCandidate and self.green_count >= 2:
        self.phase = TrafficControlPhase.off
    elif color == 0 and self.phase != TrafficControlPhase.greenFlashCandidate:
      self.phase = TrafficControlPhase.off

    self.last_real_color = observation.light_state
    return self._decision()
