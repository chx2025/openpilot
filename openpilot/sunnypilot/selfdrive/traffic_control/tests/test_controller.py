import json
import math
from pathlib import Path

from openpilot.sunnypilot.selfdrive.traffic_control.controller import (
  TeslaTrafficControlController,
  TrafficControlConfig,
  TrafficControlMode,
  TrafficControlPhase,
)
from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TeslaTrafficControlObservation


def observation(distance=80.0, light=1, now_s=1.0, *, available=True, bus=2,
                control_type=3, dlc=6, feature_state=0, state_machine=0):
  return TeslaTrafficControlObservation(
    available=available,
    valid_for_control=available and 0.0 <= distance <= 200.0,
    source_bus=bus,
    dlc=dlc,
    feature_state=feature_state,
    state_machine=state_machine,
    control_source=3,
    control_type=control_type,
    distance=distance,
    light_state=light,
    frame_mono_time=int(now_s * 1e9),
    quality=2,
  )


def update(controller, now_s, obs, *, v_ego=10.0,
           brake=False, gas=False, blinker=False, enabled=True, long_active=True,
           model_distance=None, model_candidate=False):
  return controller.update(
    obs, int(now_s * 1e9), v_ego=v_ego, a_ego=0.0,
    model_stop_distance=model_distance, model_stop_candidate=model_candidate,
    enabled=enabled, long_active=long_active,
    gas_pressed=gas, brake_pressed=brake, turn_signal_active=blinker,
  )


def controller(mode=TrafficControlMode.stopGo, **kwargs):
  return TeslaTrafficControlController(TrafficControlConfig(mode=mode, **kwargs))


def establish_red(c, *, distance=80.0, speed=10.0, start=1.0):
  first = update(c, start, observation(distance, 1, start), v_ego=speed)
  second = update(c, start + 0.5, observation(distance - speed * 0.5, 1, start + 0.5), v_ego=speed)
  return first, second


def test_red_requires_two_distinct_real_frames():
  c = controller()
  first = update(c, 1.0, observation(80.0, 1, 1.0))
  duplicate = update(c, 1.2, observation(80.0, 1, 1.0))
  confirmed = update(c, 1.5, observation(75.0, 1, 1.5))
  assert first.phase == TrafficControlPhase.redCandidate
  assert duplicate.phase == TrafficControlPhase.redCandidate
  assert confirmed.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)
  assert confirmed.apply_constraint


def test_two_hz_high_speed_red_candidate_compensates_ego_motion():
  c = controller()
  first = update(c, 1.0, observation(80.0, 1, 1.0), v_ego=20.0)
  second = update(c, 1.5, observation(70.0, 1, 1.5), v_ego=20.0)
  assert first.phase == TrafficControlPhase.redCandidate
  assert second.phase in c.ACTIVE_PHASES
  assert c.event_id == 1


def test_far_low_urgency_red_requires_half_second_continuous_evidence():
  c = controller()
  first = update(c, 1.00, observation(199.0, 1, 1.00), v_ego=13.5)
  second = update(c, 1.05, observation(198.3, 1, 1.05), v_ego=13.5)
  third = update(c, 1.17, observation(196.7, 1, 1.17), v_ego=13.5)
  expired = update(c, 1.28, observation(196.7, 0, 1.28), v_ego=13.5)

  assert first.phase == TrafficControlPhase.redCandidate
  assert second.phase == TrafficControlPhase.redCandidate
  assert third.phase == TrafficControlPhase.redCandidate
  assert expired.phase == TrafficControlPhase.off
  assert c.event_id == 0
  assert c.stop_session_id == 0


def test_far_low_urgency_red_confirms_after_half_second():
  c = controller()
  update(c, 1.0, observation(190.0, 1, 1.0), v_ego=10.0)
  update(c, 1.1, observation(189.0, 1, 1.1), v_ego=10.0)
  decision = update(c, 1.5, observation(185.0, 1, 1.5), v_ego=10.0)

  assert decision.phase in c.ACTIVE_PHASES
  assert c.event_id == 1


def test_near_urgent_red_still_confirms_in_two_frames():
  c = controller()
  first = update(c, 1.0, observation(45.0, 1, 1.0), v_ego=15.0)
  second = update(c, 1.1, observation(43.5, 1, 1.1), v_ego=15.0)

  assert first.phase == TrafficControlPhase.redCandidate
  assert second.phase in c.ACTIVE_PHASES
  assert c.event_id == 1


def test_far_low_urgency_replacement_requires_half_second_continuous_evidence():
  c = controller()
  establish_red(c, distance=35.0, speed=5.0)
  original_event = c.event_id
  original_session = c.stop_session_id

  update(c, 2.0, observation(190.0, 1, 2.0), v_ego=10.0)
  early = update(c, 2.1, observation(189.0, 1, 2.1), v_ego=10.0)
  assert c.event_id == original_event
  assert c.stop_session_id == original_session

  confirmed = update(c, 2.5, observation(185.0, 1, 2.5), v_ego=10.0)
  assert c.event_id == original_event + 1
  assert c.stop_session_id == original_session
  assert confirmed.phase in c.ACTIVE_PHASES


def test_internal_tesla_state_fields_do_not_change_color_decision():
  c = controller()
  update(c, 1.0, observation(50, 1, 1.0, feature_state=0, state_machine=6))
  decision = update(c, 1.5, observation(45, 1, 1.5, feature_state=0, state_machine=0))
  assert decision.active


def test_controller_independently_rejects_wrong_bus_type_and_short_dlc():
  for kwargs in ({"bus": 1}, {"control_type": 2}, {"dlc": 5}):
    c = controller()
    update(c, 1.0, observation(40, 1, 1.0, **kwargs))
    decision = update(c, 1.5, observation(35, 1, 1.5, **kwargs))
    assert decision.phase == TrafficControlPhase.off


def test_distance_200_is_trusted_but_201_cannot_create_a_target():
  accepted = controller()
  establish_red(accepted, distance=200.0, speed=10.0)
  assert accepted.phase in accepted.ACTIVE_PHASES

  rejected = controller()
  update(rejected, 1.0, observation(201, 1, 1.0))
  decision = update(rejected, 1.5, observation(201, 1, 1.5))
  assert decision.phase == TrafficControlPhase.off
  assert rejected.event_id == 0


def test_default_reference_is_five_meters_and_model_cannot_replace_can_geometry():
  c = controller()
  update(c, 1.0, observation(50, 1, 1.0), model_distance=8.0, model_candidate=True)
  decision = update(c, 1.5, observation(45, 1, 1.5), model_distance=4.0, model_candidate=True)
  assert decision.stop_reference == 5.0
  assert math.isclose(decision.remaining_distance, 40.0, abs_tol=0.1)


def test_stop_station_keeps_converging_to_fresh_can_inside_ten_meters():
  c = controller()
  establish_red(c, distance=20.0, speed=5.0)
  update(c, 2.2, observation(11.5, 1, 2.2), v_ego=5.0)
  previous_station = c.stop_station
  assert c.remaining_distance <= 10.0
  # Fresh, continuous CAN remains authoritative in the final approach.
  update(c, 2.4, observation(8.0, 1, 2.4), v_ego=5.0)
  assert c.stop_station != previous_station
  assert abs(c.remaining_distance - 3.0) <= 1.5
  assert c.remaining_distance < 10.0


def test_committed_stop_survives_transport_dropout():
  c = controller()
  establish_red(c)
  stale = observation(70, 1, 1.5, available=False)
  decision = update(c, 2.5, stale)
  assert decision.phase in c.ACTIVE_PHASES
  assert decision.apply_constraint


def test_stop_safety_permission_is_independent_from_raw_freshness():
  c = controller()
  establish_red(c)

  stale = update(c, 2.0, observation(70, 1, 1.5, available=False))
  assert stale.stop_safety_allowed
  assert not stale.stop_control_allowed

  direction_shadow = update(c, 2.1, observation(69, 1, 2.1), blinker=True)
  assert not direction_shadow.stop_safety_allowed
  assert not direction_shadow.stop_control_allowed


def test_long_raw_dropout_requires_two_fresh_stop_frames_before_rearming():
  c = controller()
  establish_red(c, distance=60.0, speed=5.0)

  update(c, 2.0, observation(55.0, 1, 1.5, available=False), v_ego=5.0)
  first = update(c, 4.1, observation(44.5, 1, 4.1), v_ego=5.0)
  assert not first.stop_safety_allowed
  assert not first.stop_control_allowed

  duplicate = update(c, 4.2, observation(44.5, 1, 4.1), v_ego=5.0)
  assert not duplicate.stop_safety_allowed
  assert not duplicate.stop_control_allowed

  second = update(c, 4.6, observation(42.0, 1, 4.6), v_ego=5.0)
  assert second.stop_safety_allowed
  assert second.stop_control_allowed


def test_critical_dropout_age_starts_at_the_last_real_can_frame():
  c = controller()
  establish_red(c, distance=60.0, speed=5.0)

  update(c, 2.3, observation(57.5, 1, 1.5, available=False), v_ego=5.0)
  first_recovery = update(c, 3.6, observation(52.5, 1, 3.6), v_ego=5.0)

  assert not first_recovery.stop_safety_allowed
  assert not first_recovery.stop_control_allowed


def test_direct_multi_second_real_frame_gap_requires_reconfirmation():
  c = controller()
  establish_red(c, distance=60.0, speed=5.0)

  first_after_gap = update(c, 4.1, observation(50.0, 1, 4.1), v_ego=5.0)

  assert not first_after_gap.stop_safety_allowed
  assert not first_after_gap.stop_control_allowed


def test_long_dropout_stop_reconfirmation_does_not_delay_green_release():
  c = controller()
  establish_red(c, distance=30.0, speed=5.0)

  first_green = update(c, 4.1, observation(25.0, 2, 4.1), v_ego=5.0)
  second_green = update(c, 4.6, observation(22.5, 2, 4.6), v_ego=5.0)

  assert first_green.phase in c.ACTIVE_PHASES
  assert second_green.phase == TrafficControlPhase.release
  assert second_green.apply_constraint


def test_green_requires_two_real_frames_and_releases_same_event():
  c = controller()
  establish_red(c, distance=30.0, speed=5.0)
  one = update(c, 1.8, observation(26.0, 2, 1.8), v_ego=5.0)
  assert one.phase in c.ACTIVE_PHASES
  duplicate = update(c, 1.9, observation(26.0, 2, 1.8), v_ego=5.0)
  assert duplicate.phase in c.ACTIVE_PHASES
  two = update(c, 2.3, observation(23.5, 2, 2.3), v_ego=5.0)
  assert two.phase == TrafficControlPhase.release


def test_stationary_green_release_survives_the_normal_three_second_timeout():
  c = controller(stationary_release_s=10.0)
  establish_red(c, distance=5.0, speed=0.0)
  update(c, 2.0, observation(5.0, 2, 2.0), v_ego=0.0)
  released = update(c, 2.5, observation(5.0, 2, 2.5), v_ego=0.0)
  assert released.phase == TrafficControlPhase.release

  still_waiting = released
  for step in range(30, 121, 5):
    now_s = step / 10.0
    still_waiting = update(c, now_s, observation(5.0, 2, now_s), v_ego=0.0)
  assert still_waiting.phase == TrafficControlPhase.release
  assert still_waiting.stop_session_id == released.stop_session_id

  expired = update(c, 12.5, observation(5.0, 2, 12.5), v_ego=0.0)
  assert expired.phase == TrafficControlPhase.off
  assert expired.stop_session_id == 0


def test_moving_green_release_keeps_the_existing_three_second_timeout():
  c = controller(stationary_release_s=10.0)
  establish_red(c, distance=5.0, speed=0.0)
  update(c, 2.0, observation(5.0, 2, 2.0), v_ego=0.0)
  update(c, 2.5, observation(5.0, 2, 2.5), v_ego=0.0)

  expired = None
  for step in range(30, 61, 5):
    now_s = step / 10.0
    expired = update(c, now_s, observation(max(0.0, 5.0 - (now_s - 2.5)), 2, now_s), v_ego=1.0)
  assert expired.phase == TrafficControlPhase.off
  assert expired.stop_session_id == 0


def test_long_green_can_dropout_clears_stationary_release_before_recovery():
  c = controller(stationary_release_s=10.0, critical_observation_dropout_s=2.0)
  establish_red(c, distance=5.0, speed=0.0)
  update(c, 2.0, observation(5.0, 2, 2.0), v_ego=0.0)
  released = update(c, 2.5, observation(5.0, 2, 2.5), v_ego=0.0)
  assert released.phase == TrafficControlPhase.release

  stale = update(c, 4.8, observation(5.0, 2, 2.5, available=False), v_ego=0.0)
  assert stale.phase == TrafficControlPhase.release
  recovered = update(c, 5.0, observation(5.0, 2, 5.0), v_ego=0.0)
  assert recovered.phase == TrafficControlPhase.off
  assert recovered.stop_session_id == 0


def test_far_red_after_release_gets_a_fresh_stop_station():
  c = controller()
  establish_red(c, distance=5.0, speed=0.0)
  update(c, 2.0, observation(5.0, 2, 2.0), v_ego=0.0)
  released = update(c, 2.5, observation(5.0, 2, 2.5), v_ego=0.0)
  assert released.phase == TrafficControlPhase.release

  update(c, 3.0, observation(150.0, 1, 3.0), v_ego=15.0)
  new_red = update(c, 3.5, observation(142.5, 1, 3.5), v_ego=15.0)

  assert new_red.phase in c.ACTIVE_PHASES
  assert math.isclose(new_red.remaining_distance, 137.5, abs_tol=0.1)


def test_stop_only_mode_does_not_auto_release_on_green():
  c = controller(TrafficControlMode.stopOnly)
  establish_red(c, distance=30.0, speed=5.0)
  update(c, 1.8, observation(26.0, 2, 1.8), v_ego=5.0)
  decision = update(c, 2.3, observation(23.5, 2, 2.3), v_ego=5.0)
  assert decision.phase in c.ACTIVE_PHASES


def test_green_off_green_off_pattern_latches_flashing_green_stop():
  c = controller()
  update(c, 1.0, observation(80.0, 2, 1.0), v_ego=8.0)
  update(c, 1.1, observation(79.2, 2, 1.1), v_ego=8.0)
  first_off = update(c, 1.2, observation(78.4, 0, 1.2), v_ego=8.0)
  assert first_off.phase == TrafficControlPhase.greenFlashCandidate
  update(c, 1.7, observation(74.4, 2, 1.7), v_ego=8.0)
  second_off = update(c, 2.2, observation(70.4, 0, 2.2), v_ego=8.0)
  assert second_off.phase == TrafficControlPhase.flashingGreenStop
  assert second_off.apply_constraint
  assert c.flash_latched


def test_route5a_far_green_off_cadence_arms_at_first_trusted_green_distance():
  c = controller()
  # Tesla publishes OFF as 254 m while a distant green lamp is flashing.
  # Colors above 200 m may establish cadence, but cannot establish geometry.
  for now_s, distance, light in (
    (1.0, 210.0, 2), (1.9, 254.0, 0),
    (2.5, 238.0, 2), (3.0, 254.0, 0),
    (3.5, 223.0, 2), (4.0, 254.0, 0),
  ):
    decision = update(c, now_s, observation(distance, light, now_s), v_ego=13.0)
    assert decision.phase == TrafficControlPhase.off
    assert c.event_id == 0

  trusted = update(c, 4.5, observation(200.0, 2, 4.5), v_ego=13.0)
  assert trusted.phase == TrafficControlPhase.flashingGreenStop
  assert trusted.remaining_distance == 195.0
  assert trusted.apply_constraint
  assert c.flash_latched


def test_single_far_green_off_dropout_cannot_create_a_flashing_green_stop():
  c = controller()
  update(c, 1.0, observation(210.0, 2, 1.0), v_ego=13.0)
  update(c, 1.9, observation(254.0, 0, 1.9), v_ego=13.0)
  trusted = update(c, 2.5, observation(200.0, 2, 2.5), v_ego=13.0)

  assert trusted.phase == TrafficControlPhase.off
  assert not c.flash_latched
  assert c.event_id == 0


def test_full_flash_pattern_reconfirms_stop_after_direction_shadow():
  c = controller()
  for now_s, distance, light in ((1.0, 80.0, 2), (1.1, 79.2, 2), (1.2, 78.4, 0),
                                 (1.7, 74.4, 2), (2.2, 70.4, 0)):
    shadow = update(c, now_s, observation(distance, light, now_s), v_ego=8.0, blinker=True)
  assert shadow.phase == TrafficControlPhase.flashingGreenStop
  assert not shadow.stop_control_allowed

  for now_s, distance, light in ((2.7, 66.4, 2), (3.2, 62.4, 0),
                                 (3.7, 58.4, 2), (4.2, 54.4, 0)):
    reconfirmed = update(c, now_s, observation(distance, light, now_s), v_ego=8.0, blinker=False)

  assert reconfirmed.phase == TrafficControlPhase.flashingGreenStop
  assert reconfirmed.stop_safety_allowed
  assert reconfirmed.stop_control_allowed


def test_route4f_discontinuous_off_cannot_form_flash_but_yellow_can_stop():
  c = controller()
  update(c, 1.0, observation(62.5, 2, 1.0), v_ego=5.0)
  update(c, 1.5, observation(60.0, 2, 1.5), v_ego=5.0)
  discontinuous_off = update(c, 2.0, observation(94.0, 0, 2.0), v_ego=5.0)
  assert discontinuous_off.phase != TrafficControlPhase.greenFlashCandidate
  assert not c.flash_latched
  update(c, 2.5, observation(49.0, 3, 2.5), v_ego=8.0)
  yellow = update(c, 3.0, observation(45.0, 3, 3.0), v_ego=8.0)
  assert yellow.phase == TrafficControlPhase.yellowStop
  assert yellow.remaining_distance == 40.0


def test_flashing_green_stop_is_not_released_by_later_green_pulse():
  c = controller()
  for now_s, distance, light in ((1.0, 80, 2), (1.1, 79.2, 2), (1.2, 78.4, 0),
                                 (1.7, 74.4, 2), (2.2, 70.4, 0)):
    update(c, now_s, observation(distance, light, now_s), v_ego=8.0)
  update(c, 2.7, observation(66.4, 2, 2.7), v_ego=8.0)
  decision = update(c, 3.2, observation(62.4, 2, 3.2), v_ego=8.0)
  assert decision.phase == TrafficControlPhase.flashingGreenStop


def test_early_yellow_stops_and_late_yellow_passes_once():
  early = controller()
  update(early, 1.0, observation(80, 3, 1.0), v_ego=10.0)
  early_decision = update(early, 1.5, observation(75, 3, 1.5), v_ego=10.0)
  assert early_decision.phase == TrafficControlPhase.yellowStop

  late = controller()
  update(late, 1.0, observation(20, 3, 1.0), v_ego=15.0)
  late_decision = update(late, 1.2, observation(17, 3, 1.2), v_ego=15.0)
  assert late_decision.phase == TrafficControlPhase.yellowPass
  assert not late_decision.apply_constraint


def test_yellow_pass_latch_does_not_leak_into_the_next_intersection():
  c = controller()
  update(c, 1.0, observation(20, 3, 1.0), v_ego=15.0)
  first = update(c, 1.2, observation(17, 3, 1.2), v_ego=15.0)
  assert first.phase == TrafficControlPhase.yellowPass
  update(c, 2.0, observation(254, 2, 2.0), v_ego=15.0)
  update(c, 3.0, observation(80, 3, 3.0), v_ego=5.0)
  second = update(c, 3.5, observation(77.5, 3, 3.5), v_ego=5.0)
  assert second.phase == TrafficControlPhase.yellowStop
  assert second.stop_control_allowed


def test_mid_dilemma_yellow_makes_and_latches_one_decision():
  c = controller(comfort_brake=2.4)
  # The second frame requires about 2.6 m/s^2: inside the dilemma band but
  # above the configured comfortable brake, so the one-shot result is PASS.
  update(c, 1.0, observation(35, 3, 1.0), v_ego=12.0)
  decision = update(c, 1.5, observation(32.7, 3, 1.5), v_ego=12.0)
  assert decision.phase == TrafficControlPhase.yellowPass
  assert c.yellow_latched is False


def test_existing_red_stop_treats_first_yellow_as_stop_without_reconfirmation():
  c = controller()
  establish_red(c)
  decision = update(c, 1.8, observation(72.0, 3, 1.8))
  assert decision.phase == TrafficControlPhase.yellowStop
  assert decision.apply_constraint


def test_zero_to_254_distance_wrap_is_passed_not_a_new_forward_target():
  c = controller()
  establish_red(c, distance=10.0, speed=5.0)
  update(c, 2.0, observation(1.0, 1, 2.0), v_ego=5.0)
  decision = update(c, 2.1, observation(254.0, 1, 2.1), v_ego=5.0)
  assert decision.phase == TrafficControlPhase.passed
  assert not decision.apply_constraint
  assert decision.remaining_distance == 0.0


def test_signal_event_keeps_its_identity_and_stop_geometry_across_updates():
  c = controller()
  establish_red(c)
  event_id = c.event_id
  before = c.stop_station
  decision = update(c, 2.0, observation(70, 1, 2.0))
  assert decision.active
  assert c.event_id == event_id
  assert c.stop_station == before or c.remaining_distance > 10.0


def test_active_event_replacement_requires_two_continuous_stop_color_frames():
  c = controller()
  establish_red(c, distance=100.0, speed=10.0)
  original_event = c.event_id
  first = update(c, 2.0, observation(35.0, 1, 2.0), v_ego=10.0)
  assert c.event_id == original_event
  assert first.remaining_distance > 80.0
  second = update(c, 2.5, observation(30.0, 1, 2.5), v_ego=10.0)
  assert c.event_id == original_event + 1
  # A same-session replacement may update identity immediately, but its
  # geometry must enter through bounded station fusion instead of teleporting.
  naturally_predicted = max(0.0, first.remaining_distance - 5.0)
  assert second.remaining_distance > 25.0
  assert second.remaining_distance >= naturally_predicted - 10.0


def test_target_replacement_preserves_the_stop_session():
  c = controller()
  establish_red(c, distance=100.0, speed=10.0)
  session_id = c.stop_session_id
  event_id = c.event_id
  update(c, 2.0, observation(35.0, 1, 2.0), v_ego=10.0)
  update(c, 2.5, observation(30.0, 1, 2.5), v_ego=10.0)
  assert c.event_id == event_id + 1
  assert c.stop_session_id == session_id


def test_route63_style_can_fusion_does_not_hold_ten_meters_early():
  c = controller()
  fixture = json.loads((Path(__file__).parent / "fixtures/route63_regressions.json").read_text())
  sequence = fixture["event6Fusion"]
  t0 = sequence[0]["t"] - 1.0
  update(c, 0.5, observation(66.0, 1, 0.5), v_ego=11.5)
  for sample in sequence:
    now_s = sample["t"] - t0
    distance, speed = sample["rawDistance"], sample["vEgo"]
    decision = update(c, now_s, observation(distance, 1, now_s), v_ego=speed)
  assert decision.phase != TrafficControlPhase.hold
  assert decision.remaining_distance >= 8.0
  assert abs(decision.remaining_distance - (16.0 - decision.stop_reference)) <= 2.0


def test_driver_override_keeps_observing_and_rearms_after_short_cooldown():
  c = controller()
  update(c, 1.0, observation(40.0, 1, 1.0), gas=True)
  update(c, 1.5, observation(35.0, 1, 1.5), gas=False)
  decision = update(c, 2.0, observation(30.0, 1, 2.0), gas=False)
  assert decision.phase in c.ACTIVE_PHASES
  assert not decision.driver_override_active
  assert not decision.apply_constraint
  rearmed = update(c, 2.5, observation(25.0, 1, 2.5), gas=False)
  assert rearmed.apply_constraint


def test_turning_marks_generic_signal_direction_unknown_and_disables_control():
  c = controller()
  update(c, 1.0, observation(40.0, 1, 1.0), blinker=True)
  decision = update(c, 1.5, observation(35.0, 1, 1.5), blinker=True)
  assert decision.direction_unknown
  assert decision.active
  assert not decision.apply_constraint


def test_stop_requires_two_fresh_frames_after_direction_shadow_clears():
  c = controller()
  establish_red(c, distance=40.0, speed=5.0)
  shadow = update(c, 1.6, observation(34.5, 1, 1.6), v_ego=5.0, blinker=True)
  assert not shadow.stop_control_allowed
  cached = update(c, 1.7, observation(34.5, 1, 1.6), v_ego=5.0, blinker=False)
  assert not cached.stop_control_allowed
  first = update(c, 2.1, observation(32.0, 1, 2.1), v_ego=5.0)
  assert not first.stop_control_allowed
  second = update(c, 2.6, observation(29.5, 1, 2.6), v_ego=5.0)
  assert second.stop_control_allowed


def test_discontinuous_red_cannot_clear_direction_stop_reconfirmation():
  c = controller()
  establish_red(c, distance=40.0, speed=5.0)
  update(c, 1.6, observation(34.5, 1, 1.6), v_ego=5.0, blinker=True)
  first = update(c, 2.0, observation(32.5, 1, 2.0), v_ego=5.0, blinker=False)
  assert not first.stop_control_allowed
  discontinuous = update(c, 2.5, observation(100.0, 1, 2.5), v_ego=5.0)
  assert not discontinuous.stop_control_allowed
  update(c, 3.0, observation(97.5, 1, 3.0), v_ego=5.0)
  continuous = update(c, 3.5, observation(95.0, 1, 3.5), v_ego=5.0)
  assert continuous.stop_control_allowed


def test_far_active_stop_expires_after_raw_can_dropout():
  c = controller()
  establish_red(c, distance=80.0, speed=10.0)
  stale = observation(70.0, 1, 1.5, available=False)
  update(c, 2.0, stale, v_ego=10.0)
  expired = update(c, 3.0, stale, v_ego=10.0)
  assert not expired.stop_control_allowed
  assert expired.phase in c.ACTIVE_PHASES


def test_near_active_stop_has_bounded_dropout_but_hold_remains_latched():
  c = controller()
  establish_red(c, distance=20.0, speed=2.0)
  stale = observation(15.0, 1, 1.5, available=False)
  update(c, 2.0, stale, v_ego=1.0)
  expired = update(c, 4.1, stale, v_ego=1.0)
  assert expired.phase in c.ACTIVE_PHASES
  assert not expired.stop_control_allowed

  held = controller()
  establish_red(held, distance=6.0, speed=0.0)
  update(held, 2.0, observation(6.0, 1, 2.0), v_ego=0.0)
  assert held.phase == TrafficControlPhase.hold
  still_held = update(held, 5.0, observation(6.0, 1, 2.0, available=False), v_ego=0.0)
  assert still_held.phase == TrafficControlPhase.hold


def test_stop_reconfirmation_never_adds_extra_green_go_delay():
  c = controller()
  establish_red(c, distance=30.0, speed=5.0)
  update(c, 1.7, observation(26.5, 1, 1.7), v_ego=5.0, blinker=True)
  first_green = update(c, 2.0, observation(25.0, 2, 2.0), v_ego=5.0, blinker=False)
  second_green = update(c, 2.5, observation(22.5, 2, 2.5), v_ego=5.0, blinker=False)
  assert first_green.phase in c.ACTIVE_PHASES
  assert second_green.phase == TrafficControlPhase.release


def test_far_red_after_release_starts_a_new_session():
  c = controller()
  establish_red(c, distance=20.0, speed=0.0)
  old_session = c.stop_session_id
  update(c, 2.0, observation(20.0, 2, 2.0), v_ego=0.0)
  update(c, 2.5, observation(20.0, 2, 2.5), v_ego=0.0)
  update(c, 3.0, observation(150.0, 1, 3.0), v_ego=10.0)
  update(c, 3.5, observation(145.0, 1, 3.5), v_ego=10.0)
  assert c.stop_session_id != old_session


def test_same_target_red_flicker_after_release_preserves_session():
  c = controller()
  establish_red(c, distance=20.0, speed=0.0)
  old_session = c.stop_session_id
  update(c, 2.0, observation(20.0, 2, 2.0), v_ego=0.0)
  update(c, 2.5, observation(20.0, 2, 2.5), v_ego=0.0)
  update(c, 3.0, observation(19.0, 1, 3.0), v_ego=0.0)
  update(c, 3.5, observation(18.0, 1, 3.5), v_ego=0.0)
  assert c.stop_session_id == old_session


def test_flashing_green_latch_clears_after_passed_before_new_red():
  c = controller()
  for now_s, distance, light in ((1.0, 80, 2), (1.1, 79.2, 2), (1.2, 78.4, 0),
                                 (1.7, 74.4, 2), (2.2, 70.4, 0)):
    update(c, now_s, observation(distance, light, now_s), v_ego=8.0)
  assert c.flash_latched
  now_s = 2.2
  for distance in (65.4, 60.4, 55.4, 50.4, 45.4, 40.4, 35.4, 30.4, 25.4, 20.4, 15.4, 10.4, 5.4, 1.0):
    now_s += 0.5
    update(c, now_s, observation(distance, 1, now_s), v_ego=10.0)
  update(c, now_s + 0.1, observation(254.0, 1, now_s + 0.1), v_ego=10.0)
  assert not c.flash_latched
  update(c, now_s + 1.0, observation(50.0, 1, now_s + 1.0), v_ego=5.0)
  decision = update(c, now_s + 1.5, observation(47.5, 1, now_s + 1.5), v_ego=5.0)
  assert decision.phase in (TrafficControlPhase.approachRed, TrafficControlPhase.braking)


def test_driver_gas_temporarily_suppresses_control_and_disabled_longitudinal_resets():
  c = controller()
  establish_red(c)
  override = update(c, 2.0, observation(70, 1, 2.0), gas=True)
  assert override.driver_override_active
  assert not override.apply_constraint
  reset = update(c, 20.0, observation(60, 1, 20.0), enabled=False)
  assert reset.phase == TrafficControlPhase.off
