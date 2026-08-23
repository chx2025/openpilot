import math

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


def update(controller, now_s, obs, *, v_ego=10.0, lead=False, radar=True,
           brake=False, gas=False, enabled=True, long_active=True,
           model_distance=None, model_candidate=False):
  return controller.update(
    obs, int(now_s * 1e9), v_ego=v_ego, a_ego=0.0,
    model_stop_distance=model_distance, model_stop_candidate=model_candidate,
    lead_present=lead, radar_valid=radar, enabled=enabled, long_active=long_active,
    gas_pressed=gas, brake_pressed=brake, turn_signal_active=False,
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


def test_stop_station_tracks_in_world_coordinates_then_freezes_inside_ten_meters():
  c = controller()
  establish_red(c, distance=20.0, speed=5.0)
  update(c, 2.2, observation(11.5, 1, 2.2), v_ego=5.0)
  frozen_station = c.stop_station
  assert c.remaining_distance <= 10.0
  # A noisy CAN correction inside the final zone cannot move the stop point.
  update(c, 2.4, observation(8.0, 1, 2.4), v_ego=5.0)
  assert c.stop_station == frozen_station
  assert c.remaining_distance < 10.0


def test_committed_stop_survives_transport_dropout_and_radar_health_blip():
  c = controller()
  establish_red(c)
  stale = observation(70, 1, 1.5, available=False)
  decision = update(c, 2.5, stale, radar=False)
  assert decision.phase in c.ACTIVE_PHASES
  assert decision.apply_constraint


def test_green_requires_two_real_frames_and_releases_same_event():
  c = controller()
  establish_red(c, distance=30.0, speed=5.0)
  one = update(c, 1.8, observation(26.0, 2, 1.8), v_ego=5.0)
  assert one.phase in c.ACTIVE_PHASES
  duplicate = update(c, 1.9, observation(26.0, 2, 1.8), v_ego=5.0)
  assert duplicate.phase in c.ACTIVE_PHASES
  two = update(c, 2.3, observation(23.5, 2, 2.3), v_ego=5.0)
  assert two.phase == TrafficControlPhase.release


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


def test_lead_does_not_erase_the_signal_event_and_never_changes_stop_geometry():
  c = controller(retain_event_with_lead=True)
  establish_red(c)
  event_id = c.event_id
  before = c.stop_station
  decision = update(c, 2.0, observation(70, 1, 2.0), lead=True)
  assert decision.active
  assert c.event_id == event_id
  assert c.stop_station == before or c.remaining_distance > c.config.final_distance_freeze_m


def test_active_event_replacement_requires_two_continuous_stop_color_frames():
  c = controller()
  establish_red(c, distance=100.0, speed=10.0)
  original_event = c.event_id
  first = update(c, 2.0, observation(35.0, 1, 2.0), v_ego=10.0)
  assert c.event_id == original_event
  assert first.remaining_distance > 80.0
  second = update(c, 2.5, observation(30.0, 1, 2.5), v_ego=10.0)
  assert c.event_id == original_event + 1
  assert second.remaining_distance == 25.0


def test_driver_gas_bypasses_and_disabled_longitudinal_resets():
  c = controller()
  establish_red(c)
  bypass = update(c, 2.0, observation(70, 1, 2.0), gas=True)
  assert bypass.phase == TrafficControlPhase.bypass
  reset = update(c, 20.0, observation(60, 1, 20.0), enabled=False)
  assert reset.phase == TrafficControlPhase.off
