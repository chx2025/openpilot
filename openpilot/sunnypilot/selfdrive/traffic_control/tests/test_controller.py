import math
import json
from pathlib import Path

import pytest

from openpilot.sunnypilot.selfdrive.traffic_control.controller import (
  TeslaTrafficControlController,
  TrafficControlConfig,
  TrafficControlMode,
  TrafficControlPhase,
)
from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TeslaTrafficControlObservation


def observation(distance=80.0, light=1, quality=2, valid=True, now_ns=0, source_bus=2, control_source=3):
  return TeslaTrafficControlObservation(
    available=valid,
    valid_for_control=valid and distance < 255 and light in (1, 2, 3),
    source_bus=source_bus,
    dlc=6,
    feature_state=3,
    state_machine=4,
    control_source=control_source,
    control_type=3,
    distance=distance,
    light_state=light,
    frame_mono_time=now_ns,
    quality=quality,
  )


def raw_ineligible_event_observation(distance, *, light=1, state_machine=3, now_ns=0,
                                     source_bus=2, control_source=3):
  return TeslaTrafficControlObservation(
    available=True,
    valid_for_control=False,
    source_bus=source_bus,
    dlc=6,
    feature_state=0,
    state_machine=state_machine,
    control_source=control_source,
    control_type=3,
    distance=distance,
    light_state=light,
    unavailable_reason=1,
    vision_light=True,
    frame_mono_time=now_ns,
    quality=1,
  )


def feature_zero_yellow_observation(distance, *, now_ns=0):
  return TeslaTrafficControlObservation(
    available=True,
    valid_for_control=True,
    source_bus=2,
    dlc=6,
    feature_state=0,
    state_machine=6,
    control_source=3,
    control_type=3,
    distance=distance,
    light_state=3,
    continuation_reason=5,
    unavailable_reason=1,
    vision_light=True,
    frame_mono_time=now_ns,
    quality=2,
  )


def update(controller, now_s, obs, *, v_ego=15.0, a_ego=0.0, model_stop_distance=None,
           model_stop_candidate=False, lead_present=False, radar_valid=True, gas=False,
           brake=False, blinker=False, enabled=True, long_active=True):
  return controller.update(
    obs, int(now_s * 1e9), v_ego=v_ego, a_ego=a_ego,
    model_stop_distance=model_stop_distance, model_stop_candidate=model_stop_candidate,
    lead_present=lead_present, radar_valid=radar_valid, enabled=enabled,
    long_active=long_active, gas_pressed=gas, brake_pressed=brake,
    turn_signal_active=blinker,
  )


AUTO_MODEL_STOP = object()


def confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=AUTO_MODEL_STOP):
  if model_stop_distance is AUTO_MODEL_STOP:
    model_stop_distance = max(0.0, distance - controller.config.default_stop_reference)
  update(controller, 0.00, observation(distance, now_ns=0), v_ego=v_ego,
         model_stop_distance=model_stop_distance, model_stop_candidate=model_stop_distance is not None)
  update(controller, 0.30, observation(distance - 4, now_ns=int(0.30e9)), v_ego=v_ego,
         model_stop_distance=None if model_stop_distance is None else model_stop_distance - 4,
         model_stop_candidate=model_stop_distance is not None)
  return update(controller, 0.60, observation(distance - 8, now_ns=int(0.60e9)), v_ego=v_ego,
                model_stop_distance=None if model_stop_distance is None else model_stop_distance - 8,
                model_stop_candidate=model_stop_distance is not None)


def test_red_inside_100m_without_lead_enters_early_approach():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=100.0, v_ego=16.7)
  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert decision.active
  # Approach/braking must stay in normal longitudinal PID and use the planned
  # acceleration target. shouldStop is reserved for the final stationary hold.
  assert not decision.should_stop
  assert 84.0 <= decision.remaining_distance <= 88.5


def test_yellow_green_flicker_keeps_a_confirmed_yellow_stop_active():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, feature_zero_yellow_observation(80.0, now_ns=0), v_ego=10.0)
  update(controller, 0.3, feature_zero_yellow_observation(77.0, now_ns=int(0.3e9)), v_ego=10.0)
  decision = update(
    controller, 0.6, feature_zero_yellow_observation(74.0, now_ns=int(0.6e9)), v_ego=10.0,
  )
  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  event_id = controller.event_id
  green_distance = controller.remaining_distance + controller.stop_reference - 1.0

  flicker = update(
    controller, 0.7,
    raw_ineligible_event_observation(
      green_distance, light=2, state_machine=6, now_ns=int(0.7e9),
    ),
    v_ego=10.0,
  )

  assert flicker.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert controller.event_id == event_id
  assert flicker.apply_constraint


def test_active_red_ignores_interleaved_no_color_frames():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=80.0, v_ego=10.0)
  event_id = controller.event_id
  no_color_distance = controller.remaining_distance + controller.stop_reference - 1.0

  decision = update(
    controller, 0.7,
    raw_ineligible_event_observation(
      no_color_distance, light=0, state_machine=6, now_ns=int(0.7e9),
    ),
    v_ego=10.0,
  )

  assert controller.event_id == event_id
  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert decision.light_state == 1
  assert decision.apply_constraint

  dropped = update(
    controller, 1.6,
    raw_ineligible_event_observation(
      no_color_distance, light=0, state_machine=6, now_ns=int(1.6e9),
    ),
    v_ego=10.0,
  )
  assert dropped.phase == TrafficControlPhase.off
  assert dropped.light_state == 0
  assert not dropped.apply_constraint


def test_unconfirmed_red_candidate_does_not_latch_color_after_dropout():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = update(controller, 0.0, observation(80.0, now_ns=0), v_ego=10.0)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert decision.light_state == 1

  no_color = raw_ineligible_event_observation(
    79.0, light=0, state_machine=6, now_ns=int(0.1e9),
  )
  decision = update(controller, 0.1, no_color, v_ego=10.0)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert decision.light_state == 0

  decision = update(controller, 3.0, no_color, v_ego=10.0)
  assert decision.phase == TrafficControlPhase.off
  assert decision.light_state == 0


def test_yellow_candidate_survives_short_green_flicker_and_confirms():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  sequence = (
    (0.0, feature_zero_yellow_observation(80.0, now_ns=0)),
    (0.2, raw_ineligible_event_observation(78.0, light=2, state_machine=6, now_ns=int(0.2e9))),
    (0.3, feature_zero_yellow_observation(77.0, now_ns=int(0.3e9))),
    (0.4, raw_ineligible_event_observation(76.0, light=2, state_machine=6, now_ns=int(0.4e9))),
    (0.5, feature_zero_yellow_observation(75.0, now_ns=int(0.5e9))),
    (0.7, feature_zero_yellow_observation(73.0, now_ns=int(0.7e9))),
  )

  for now_s, obs in sequence:
    decision = update(controller, now_s, obs, v_ego=10.0)

  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert decision.apply_constraint


def test_late_high_speed_yellow_stays_in_the_dilemma_zone_without_hard_braking():
  controller = TeslaTrafficControlController(TrafficControlConfig(
    mode=TrafficControlMode.stopGo,
    max_control_speed=80.0 / 3.6,
  ))
  for now_s, distance in ((0.0, 49.0), (0.3, 43.0), (0.6, 37.0), (0.9, 31.0)):
    decision = update(
      controller, now_s,
      feature_zero_yellow_observation(distance, now_ns=int(now_s * 1e9)),
      v_ego=69.3 / 3.6,
    )

  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint
  assert controller.event_id == 0


def test_red_event_is_not_established_above_the_configured_control_speed():
  controller = TeslaTrafficControlController(TrafficControlConfig(
    mode=TrafficControlMode.stopGo,
    max_control_speed=50.0 / 3.6,
  ))

  decision = confirmed_red(controller, distance=100.0, v_ego=16.7)

  assert decision.phase == TrafficControlPhase.off
  assert not decision.active
  assert not decision.apply_constraint
  assert controller.event_id == 0


def test_can_authoritative_red_uses_oem_distance_without_model_stop_confirmation():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=None)

  assert decision.phase == TrafficControlPhase.braking
  assert decision.active
  assert decision.apply_constraint
  assert 65.0 <= decision.remaining_distance <= 67.0


def test_can_red_falls_back_to_oem_distance_when_model_stop_is_misaligned():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=25.0)

  assert decision.phase == TrafficControlPhase.braking
  assert decision.active
  assert decision.apply_constraint
  assert 65.0 <= decision.remaining_distance <= 67.0


def test_intermittent_model_stop_does_not_delay_a_can_authoritative_red_event():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(80.0, now_ns=0),
         model_stop_distance=74.0, model_stop_candidate=True)
  update(controller, 0.3, observation(75.5, now_ns=int(0.3e9)),
         model_stop_distance=None, model_stop_candidate=False)
  decision = update(controller, 0.6, observation(71.0, now_ns=int(0.6e9)),
                    model_stop_distance=65.0, model_stop_candidate=True)
  assert decision.active
  assert decision.apply_constraint
  assert decision.remaining_distance == 65.0


def test_model_stop_starts_early_then_binds_to_can_red_and_releases_on_can_green():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  no_can = observation(255.0, valid=False, now_ns=0)
  decision = update(
    controller, 0.0, no_can, v_ego=15.0,
    model_stop_distance=70.0, model_stop_candidate=True,
  )
  assert decision.phase == TrafficControlPhase.off

  decision = update(
    controller, 0.5, observation(255.0, valid=False, now_ns=int(0.5e9)), v_ego=15.0,
    model_stop_distance=65.0, model_stop_candidate=True,
  )
  event_id = controller.event_id
  assert decision.phase == TrafficControlPhase.braking
  assert decision.remaining_distance == 65.0
  assert event_id > 0

  decision = update(
    controller, 0.6, observation(60.0, light=1, now_ns=int(0.6e9)), v_ego=14.0,
    model_stop_distance=60.0, model_stop_candidate=True,
  )
  assert controller.event_id == event_id
  assert controller.event_source_bus == 2
  assert decision.active

  green_distance = controller.remaining_distance + controller.stop_reference
  decision = update(
    controller,
    0.7,
    raw_ineligible_event_observation(
      green_distance, light=2, state_machine=6, now_ns=int(0.7e9),
    ),
    v_ego=14.0,
  )
  assert decision.phase == TrafficControlPhase.release
  assert controller.event_id == event_id


def test_model_stop_does_not_create_a_new_event_while_already_stationary_without_can():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  no_can = observation(255.0, valid=False, now_ns=0)
  update(
    controller, 0.0, no_can, v_ego=0.0,
    model_stop_distance=2.0, model_stop_candidate=True,
  )
  decision = update(
    controller, 0.5, observation(255.0, valid=False, now_ns=int(0.5e9)), v_ego=0.0,
    model_stop_distance=2.0, model_stop_candidate=True,
  )

  assert decision.phase == TrafficControlPhase.off
  assert controller.event_id == 0


def test_red_beyond_100m_is_observed_but_never_applied():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=135.0, v_ego=22.2)
  assert not decision.active
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint


def test_candidate_uses_cp_history_window_but_active_event_keeps_short_dropout_grace():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(80, now_ns=0))
  decision = update(controller, 0.1, observation(255, valid=False, now_ns=int(0.1e9)))
  assert decision.phase == TrafficControlPhase.redCandidate
  decision = update(controller, 0.8, observation(255, valid=False, now_ns=int(0.8e9)))
  assert decision.phase == TrafficControlPhase.redCandidate
  decision = update(controller, 2.6, observation(255, valid=False, now_ns=int(2.6e9)))
  assert decision.phase == TrafficControlPhase.off

  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0)
  remaining = decision.remaining_distance
  decision = update(controller, 0.90, observation(255, valid=False, now_ns=int(0.9e9)), v_ego=15.0)
  assert decision.active
  assert 0 < decision.remaining_distance < remaining
  decision = update(controller, 1.50, observation(255, valid=False, now_ns=int(1.5e9)), v_ego=15.0)
  assert decision.phase == TrafficControlPhase.off


def test_active_event_survives_compatible_raw_oem_states_that_are_not_new_event_eligible():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0)
  event_id = controller.event_id
  assert decision.active

  for now_s, distance, state_machine in ((0.9, 67.5, 3), (1.2, 63.0, 4), (1.5, 58.5, 5)):
    decision = update(
      controller,
      now_s,
      raw_ineligible_event_observation(
        distance, state_machine=state_machine, now_ns=int(now_s * 1e9),
      ),
      v_ego=15.0,
    )

  assert decision.active
  assert controller.event_id == event_id


def test_event_reference_uses_bounded_model_offset_instead_of_fixed_six():
  controller = TeslaTrafficControlController(TrafficControlConfig(
    mode=TrafficControlMode.stopGo, adaptive_reference=True,
  ))
  decision = confirmed_red(controller, distance=50.0, v_ego=10.0, model_stop_distance=42.0)
  assert 6.0 < decision.stop_reference <= 8.0
  assert math.isclose(decision.remaining_distance, 42.0 - decision.stop_reference, abs_tol=1.0)


def test_confirmed_red_uses_cp_model_stop_distance_as_primary_target():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=50.0, v_ego=10.0, model_stop_distance=42.0)
  # At the final confirmation sample both Tesla and CP model distances moved
  # forward by 8 m. CP's 34 m stop target should be used instead of 42-6=36 m.
  assert math.isclose(decision.remaining_distance, 34.0, abs_tol=0.5)


def test_committed_stop_target_freezes_inside_ten_meters():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=20.0, v_ego=5.0)
  assert decision.remaining_distance == 6.0

  decision = update(
    controller,
    0.8,
    observation(2.0, light=1, now_ns=int(0.8e9)),
    v_ego=5.0,
    model_stop_distance=0.0,
    model_stop_candidate=True,
  )

  assert decision.remaining_distance == pytest.approx(5.0)


def test_far_oem_target_closes_at_cp_distance_rate():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=None)
  assert decision.remaining_distance == 66.0

  decision = update(
    controller,
    0.65,
    observation(60.0, light=1, now_ns=int(0.65e9)),
    v_ego=10.0,
  )

  assert decision.remaining_distance == pytest.approx(64.5)


def test_green_at_255_never_releases_hold():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  decision = update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold
  for now_s in (1.0, 1.3, 1.7):
    decision = update(controller, now_s, observation(255, light=2, valid=False, now_ns=int(now_s * 1e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold
  assert decision.should_stop


def test_stable_green_releases_only_same_stopped_event_without_turn_signal_or_lead():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)

  for now_s in (1.0, 1.3, 1.7):
    decision = update(controller, now_s, observation(6.0, light=2, now_ns=int(now_s * 1e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.release
  assert not decision.should_stop

  blocked = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(blocked, distance=20.0, v_ego=5.0)
  update(blocked, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  for now_s in (1.0, 1.3, 1.7):
    decision = update(blocked, now_s, observation(6.0, light=2, now_ns=int(now_s * 1e9)),
                      v_ego=0.0, blinker=True)
  assert decision.phase == TrafficControlPhase.hold

  braking = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(braking, distance=20.0, v_ego=5.0)
  update(braking, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  for now_s in (1.0, 1.3, 1.7):
    decision = update(braking, now_s, observation(6.0, light=2, now_ns=int(now_s * 1e9)),
                      v_ego=0.0, brake=True)
  assert decision.phase == TrafficControlPhase.hold


def test_feature_zero_green_can_release_only_the_same_held_event():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  decision = update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  event_id = controller.event_id
  assert decision.phase == TrafficControlPhase.hold

  for now_s in (1.0, 1.3, 1.7):
    decision = update(
      controller,
      now_s,
      raw_ineligible_event_observation(
        6.0, light=2, state_machine=6, now_ns=int(now_s * 1e9),
      ),
      v_ego=0.0,
    )

  assert decision.phase == TrafficControlPhase.release
  assert controller.event_id == event_id


def test_can_authoritative_green_releases_the_held_event_on_its_first_fresh_transition():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  decision = update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  event_id = controller.event_id
  assert decision.phase == TrafficControlPhase.hold

  decision = update(
    controller,
    1.0,
    raw_ineligible_event_observation(6.0, light=2, state_machine=6, now_ns=int(1.0e9)),
    v_ego=0.0,
  )

  assert decision.phase == TrafficControlPhase.release
  assert controller.event_id == event_id


def test_can_green_releases_immediately_while_the_red_event_is_still_braking():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=20.0, v_ego=5.0)
  event_id = controller.event_id
  assert decision.phase == TrafficControlPhase.braking
  green_distance = controller.remaining_distance + controller.stop_reference

  decision = update(
    controller,
    0.8,
    raw_ineligible_event_observation(
      green_distance, light=2, state_machine=6, now_ns=int(0.8e9),
    ),
    v_ego=5.0,
  )

  assert decision.phase == TrafficControlPhase.release
  assert controller.event_id == event_id
  assert not decision.should_stop


def test_can_green_releases_immediately_during_the_red_approach_phase():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=5.0)
  assert decision.phase == TrafficControlPhase.approachRed
  green_distance = controller.remaining_distance + controller.stop_reference

  decision = update(
    controller,
    0.8,
    raw_ineligible_event_observation(
      green_distance, light=2, state_machine=6, now_ns=int(0.8e9),
    ),
    v_ego=5.0,
  )

  assert decision.phase == TrafficControlPhase.release
  assert not decision.should_stop


def test_can_green_with_a_physical_lead_releases_traffic_hold_to_the_lead_planner():
  controller = TeslaTrafficControlController(TrafficControlConfig(
    mode=TrafficControlMode.stopGo,
    retain_event_with_lead=True,
  ))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  decision = update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold

  decision = update(
    controller,
    1.0,
    raw_ineligible_event_observation(6.0, light=2, state_machine=6, now_ns=int(1.0e9)),
    v_ego=0.0,
    lead_present=True,
  )

  assert decision.phase == TrafficControlPhase.release
  assert not decision.should_stop


def test_release_window_stays_active_long_enough_for_bounded_start_handoff():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  for now_s in (1.0, 1.3, 1.7):
    decision = update(
      controller,
      now_s,
      raw_ineligible_event_observation(
        6.0, light=2, state_machine=6, now_ns=int(now_s * 1e9),
      ),
      v_ego=0.0,
    )
  assert decision.phase == TrafficControlPhase.release

  decision = update(
    controller,
    3.7,
    raw_ineligible_event_observation(6.0, light=2, state_machine=6, now_ns=int(3.7e9)),
    v_ego=1.0,
  )

  assert decision.phase == TrafficControlPhase.release


def test_green_from_different_target_does_not_release_latched_stop():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  for now_s in (1.0, 1.3, 1.7):
    decision = update(controller, now_s, observation(80.0, light=2, now_ns=int(now_s * 1e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold

  for now_s in (1.9, 2.2, 2.5):
    decision = update(controller, now_s, observation(6.0, light=2, now_ns=int(now_s * 1e9), source_bus=0), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold


def test_stop_only_holds_on_green_until_driver_override():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopOnly))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  update(controller, 0.8, observation(6.0, light=1, now_ns=int(0.8e9)), v_ego=0.0)
  for now_s in (1.0, 1.3, 1.7):
    decision = update(controller, now_s, observation(6.0, light=2, now_ns=int(now_s * 1e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold
  decision = update(controller, 1.8, observation(6.0, light=2, now_ns=int(1.8e9)), v_ego=0.0, gas=True)
  assert decision.phase == TrafficControlPhase.bypass


def test_candidate_target_change_restarts_red_confirmation():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(80.0, now_ns=0))
  update(controller, 0.4, observation(75.0, now_ns=int(0.4e9)))
  decision = update(controller, 0.6, observation(74.0, now_ns=int(0.6e9), source_bus=0),
                    model_stop_distance=68.0, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.active
  decision = update(controller, 1.7, observation(58.0, now_ns=int(1.7e9), source_bus=0),
                    model_stop_distance=52.0, model_stop_candidate=True)
  assert decision.active


def test_route_candidate_jitter_preserves_one_candidate_until_a_real_dropout():
  fixture = json.loads((Path(__file__).parent / "fixtures/traffic_candidate_jitter.json").read_text())
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.shadow))
  transitions = []
  last_seq = 0
  decisions = []
  for frame in fixture["frames"]:
    obs = observation(frame["distance"], valid=frame["valid"], now_ns=int(frame["t"] * 1e9))
    decision = update(controller, frame["t"], obs, v_ego=0.0)
    decisions.append(decision)
    if controller.transition_seq != last_seq:
      transitions.append(controller.transition_reason)
      last_seq = controller.transition_seq

  # Short unavailable frames and isolated distance quantization must retain one
  # CAN-authoritative event without candidate replacement or cancellation.
  assert controller.event_id == 1
  assert transitions == ["candidate_started", "stop_confirmed", "stationary_hold"]


def test_large_downward_distance_jump_is_a_new_target_and_restarts_confirmation():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(150.0, now_ns=0), v_ego=10.0)
  update(controller, 0.4, observation(146.0, now_ns=int(0.4e9)), v_ego=10.0)

  decision = update(controller, 0.6, observation(30.0, now_ns=int(0.6e9)), v_ego=10.0,
                    model_stop_distance=24.0, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint

  decision = update(controller, 1.0, observation(26.0, now_ns=int(1.0e9)), v_ego=10.0,
                    model_stop_distance=20.0, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint

  decision = update(controller, 1.7, observation(19.0, now_ns=int(1.7e9)), v_ego=10.0,
                    model_stop_distance=13.0, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint

  decision = update(controller, 2.8, observation(8.0, now_ns=int(2.8e9)), v_ego=10.0,
                    model_stop_distance=2.0, model_stop_candidate=True)
  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert decision.apply_constraint


def test_repeated_small_distance_jumps_cannot_accumulate_into_immediate_stop():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = None
  for index, distance in enumerate((150.0, 143.0, 136.0, 129.0, 122.0, 115.0, 108.0, 101.0, 94.0)):
    now_s = index * 0.1
    decision = update(controller, now_s, observation(distance, now_ns=int(now_s * 1e9)), v_ego=10.0)

  assert decision is not None
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint


def test_dropout_near_stop_keeps_committed_hold_constraint():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  confirmed_red(controller, distance=20.0, v_ego=5.0)
  decision = update(controller, 0.8, observation(6.0, now_ns=int(0.8e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold
  decision = update(controller, 2.0, observation(255.0, valid=False, now_ns=int(2.0e9)), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.hold
  assert decision.apply_constraint


def test_false_near_target_does_not_become_irrevocable_while_vehicle_is_moving():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(14.0, now_ns=0), v_ego=15.0,
         model_stop_distance=8.0, model_stop_candidate=True)
  update(controller, 0.3, observation(10.0, now_ns=int(0.3e9)), v_ego=15.0,
         model_stop_distance=4.0, model_stop_candidate=True)
  decision = update(controller, 0.6, observation(6.0, now_ns=int(0.6e9)), v_ego=15.0,
                    model_stop_distance=0.0, model_stop_candidate=True)
  assert decision.apply_constraint
  assert decision.remaining_distance == 0.0

  decision = update(controller, 1.5, observation(255.0, valid=False, now_ns=int(1.5e9)), v_ego=15.0)
  assert decision.phase == TrafficControlPhase.off
  assert not decision.apply_constraint


def test_stop_control_never_participates_with_lead_or_invalid_radar():
  for lead_present, radar_valid in ((True, True), (False, False)):
    controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
    decision = confirmed_red(controller, distance=80.0, v_ego=15.0)
    assert decision.active

    decision = update(
      controller, 0.7, observation(70.0, now_ns=int(0.7e9)), v_ego=15.0,
      lead_present=lead_present, radar_valid=radar_valid,
    )
    assert decision.phase == TrafficControlPhase.off
    assert not decision.apply_constraint


def test_fixed_six_meter_reference_is_default():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=50.0, v_ego=10.0, model_stop_distance=40.0)
  assert decision.stop_reference == 6.0
