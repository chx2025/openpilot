import math

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


def test_oem_red_without_model_stop_confirmation_never_constrains_planner():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=None)

  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.active
  assert not decision.apply_constraint


def test_oem_red_with_misaligned_model_stop_never_constrains_planner():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=80.0, v_ego=15.0, model_stop_distance=25.0)

  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.active
  assert not decision.apply_constraint


def test_model_stop_confirmation_must_be_continuous():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(80.0, now_ns=0),
         model_stop_distance=74.0, model_stop_candidate=True)
  update(controller, 0.3, observation(75.5, now_ns=int(0.3e9)),
         model_stop_distance=None, model_stop_candidate=False)
  decision = update(controller, 0.6, observation(71.0, now_ns=int(0.6e9)),
                    model_stop_distance=65.0, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  decision = update(controller, 0.9, observation(66.5, now_ns=int(0.9e9)),
                    model_stop_distance=60.5, model_stop_candidate=True)
  assert decision.phase == TrafficControlPhase.redCandidate
  decision = update(controller, 1.1, observation(63.5, now_ns=int(1.1e9)),
                    model_stop_distance=57.5, model_stop_candidate=True)
  assert decision.active
  assert decision.apply_constraint


def test_red_beyond_100m_is_observed_but_never_applied():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  decision = confirmed_red(controller, distance=135.0, v_ego=22.2)
  assert not decision.active
  assert decision.phase == TrafficControlPhase.redCandidate
  assert not decision.apply_constraint


def test_dropout_cancels_far_stop_after_grace_but_not_immediately():
  controller = TeslaTrafficControlController(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  update(controller, 0.0, observation(80, now_ns=0))
  decision = update(controller, 0.1, observation(255, valid=False, now_ns=int(0.1e9)))
  assert decision.phase == TrafficControlPhase.off

  decision = confirmed_red(controller, distance=80.0)
  remaining = decision.remaining_distance
  decision = update(controller, 0.90, observation(255, valid=False, now_ns=int(0.9e9)), v_ego=15.0)
  assert decision.active
  assert 0 < decision.remaining_distance < remaining
  decision = update(controller, 1.50, observation(255, valid=False, now_ns=int(1.5e9)), v_ego=15.0)
  assert decision.phase == TrafficControlPhase.off


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
