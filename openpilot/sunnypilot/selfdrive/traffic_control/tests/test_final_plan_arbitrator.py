from types import SimpleNamespace as ns

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.final_plan_arbitrator import (
  FinalPlanArbitrator,
  TrafficPlanAction,
  TrafficStartBlockReason,
  create_final_plan_arbitrator,
)


NOW_NS = 1_000_000_000


class FakeSubMaster:
  def __init__(self, values):
    self.values = values
    self.seen = dict.fromkeys(values, True)
    self.alive = dict.fromkeys(values, True)
    self.valid = dict.fromkeys(values, True)

  def __getitem__(self, key):
    return self.values[key]


def base_plan(*, a_target=0.4, should_stop=False):
  return ns(
    speeds=[8.0] * 17,
    accels=[a_target] * 17,
    jerks=[0.0] * 17,
    aTarget=a_target,
    shouldStop=should_stop,
    allowThrottle=True,
  )


def fake_sm(*, phase=TrafficControlPhase.off, light_state=0, target=False,
            allowed=False, start=False, event_id=0, distance=30.0,
            v_ego=8.0, base_model_stop=False,
            personality=log.LongitudinalPersonality.standard):
  traffic = ns(
    phase=int(phase), lightState=light_state, targetPresent=target,
    controlAllowed=allowed, plannerStartRequested=start, eventId=event_id,
    distanceToStopPoint=distance, publishMonoTime=NOW_NS, confidence=1.0,
    shouldStop=phase == TrafficControlPhase.hold, mode=4,
    oemTargetDistance=distance + 5.0, rawDistance=distance + 5.0, sourceBus=2, quality=2,
  )
  no_lead = ns(present=False)
  return FakeSubMaster({
    "trafficRadarState": traffic,
    "radarState": ns(leadOne=no_lead, leadTwo=ns(present=False)),
    "carState": ns(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False, vCruise=50.0),
    "carControl": ns(enabled=True, longActive=True, leftBlinker=False, rightBlinker=False),
    "modelV2": ns(action=ns(shouldStop=base_model_stop)),
    "selfdriveState": ns(personality=personality),
  })


def test_no_target_is_output_transparent():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  original = (list(plan.speeds), list(plan.accels), list(plan.jerks), plan.aTarget,
              plan.shouldStop, plan.allowThrottle)

  arbitrator.apply(plan, fake_sm(), NOW_NS)

  assert (plan.speeds, plan.accels, plan.jerks, plan.aTarget,
          plan.shouldStop, plan.allowThrottle) == original
  assert arbitrator.diagnostics.action == TrafficPlanAction.none


def test_far_red_is_tracked_without_constraining_before_the_dynamic_braking_horizon():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan(a_target=0.4)
  original = (list(plan.speeds), list(plan.accels), list(plan.jerks), plan.aTarget,
              plan.shouldStop, plan.allowThrottle)
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=70, distance=190.0, v_ego=20.0,
  )

  arbitrator.apply(plan, sm, NOW_NS)

  assert (plan.speeds, plan.accels, plan.jerks, plan.aTarget,
          plan.shouldStop, plan.allowThrottle) == original
  assert arbitrator.diagnostics.action == TrafficPlanAction.none


def test_dynamic_braking_horizon_tracks_speed_and_longitudinal_personality():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  relaxed = arbitrator._traffic_activation_distance(fake_sm(
    v_ego=14.0, personality=log.LongitudinalPersonality.relaxed,
  ))
  standard = arbitrator._traffic_activation_distance(fake_sm(
    v_ego=14.0, personality=log.LongitudinalPersonality.standard,
  ))
  aggressive = arbitrator._traffic_activation_distance(fake_sm(
    v_ego=14.0, personality=log.LongitudinalPersonality.aggressive,
  ))
  low_speed = arbitrator._traffic_activation_distance(fake_sm(v_ego=5.0))

  assert relaxed > standard > aggressive > low_speed


def test_dynamic_braking_horizon_latches_until_the_stop_event_ends():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  entering = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=74, distance=55.0, v_ego=14.0,
  )
  arbitrator.apply(base_plan(), entering, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  # Slowing down shrinks the computed horizon below the remaining distance,
  # but an already armed event must keep a continuous stop constraint.
  slowed = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=74, distance=50.0, v_ego=5.0,
  )
  arbitrator.apply(base_plan(), slowed, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop


def test_green_start_survives_one_cycle_of_radar_source_and_live_radar_disagreement():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=73, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(should_stop=True), hold, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, target=False,
    allowed=True, start=True, event_id=73, distance=0.0, v_ego=0.0,
  )
  first = base_plan(a_target=0.4, should_stop=True)
  arbitrator.apply(first, green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.start_applied

  # trafficRadarState can still report suppression for one cycle after the
  # live radar has cleared. That cycle must pause, not complete, the GO event.
  green["trafficRadarState"].plannerStartRequested = False
  arbitrator.apply(base_plan(a_target=0.4, should_stop=True), green, NOW_NS + 100_000_000)

  green["trafficRadarState"].plannerStartRequested = True
  resumed = base_plan(a_target=0.4, should_stop=True)
  arbitrator.apply(resumed, green, NOW_NS + 150_000_000)
  assert arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.none


def test_moving_lead_green_handoff_only_clears_should_stop_after_two_cycles():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold_sm = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=71, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(should_stop=True), hold_sm, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, target=False,
    allowed=False, start=False, event_id=71, distance=0.0, v_ego=0.0,
  )
  green["radarState"].leadOne = ns(present=True, dRel=8.0, vRel=1.0)
  green["radarState"].leadTwo = ns(present=False, dRel=0.0, vRel=0.0)

  first = base_plan(a_target=0.4, should_stop=True)
  first.speeds = np.linspace(0.0, 2.0, 17).tolist()
  original_first = (list(first.speeds), list(first.accels), list(first.jerks), first.aTarget)
  arbitrator.apply(first, green, NOW_NS + 50_000_000)
  assert first.shouldStop
  assert (first.speeds, first.accels, first.jerks, first.aTarget) == original_first

  second = base_plan(a_target=0.4, should_stop=True)
  second.speeds = np.linspace(0.0, 2.0, 17).tolist()
  original_second = (list(second.speeds), list(second.accels), list(second.jerks), second.aTarget)
  arbitrator.apply(second, green, NOW_NS + 100_000_000)
  assert not second.shouldStop
  assert (second.speeds, second.accels, second.jerks, second.aTarget) == original_second


def test_moving_lead_handoff_does_not_require_traffic_module_to_have_owned_the_hold():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, target=False,
    allowed=False, start=False, event_id=72, distance=0.0, v_ego=0.0,
  )
  green["radarState"].leadOne = ns(present=True, dRel=8.0, vRel=1.0)
  green["radarState"].leadTwo = ns(present=False, dRel=0.0, vRel=0.0)
  for cycle in range(2):
    plan = base_plan(a_target=0.4, should_stop=True)
    plan.speeds = np.linspace(0.0, 2.0, 17).tolist()
    arbitrator.apply(plan, green, NOW_NS + cycle * 50_000_000)
  assert not plan.shouldStop


def test_confirmed_red_builds_a_bounded_complete_stop_plan():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=7, distance=24.0,
  )

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.applied
  assert plan.aTarget <= 0.0
  assert np.all(np.asarray(plan.speeds) <= 8.0)
  assert np.all(np.asarray(plan.accels) <= 0.4)
  assert len(plan.speeds) == len(plan.accels) == len(plan.jerks) == 17


def test_traffic_stop_initial_deceleration_scales_with_current_speed():
  def traffic_accel(v_ego):
    arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
    sm = fake_sm(
      phase=TrafficControlPhase.braking, light_state=1, target=True,
      allowed=True, event_id=40, distance=70.0, v_ego=v_ego,
    )
    arbitrator.apply(base_plan(), sm, NOW_NS)
    return arbitrator.diagnostics.traffic_a_target

  low_speed_accel = traffic_accel(8.0)
  high_speed_accel = traffic_accel(16.0)

  assert high_speed_accel < low_speed_accel - 0.02


def test_traffic_stop_uses_the_selected_longitudinal_personality():
  def traffic_accel(personality):
    arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
    sm = fake_sm(
      phase=TrafficControlPhase.braking, light_state=1, target=True,
        allowed=True, event_id=41, distance=50.0, v_ego=14.0,
      personality=personality,
    )
    arbitrator.apply(base_plan(), sm, NOW_NS)
    return arbitrator.diagnostics.traffic_a_target

  relaxed = traffic_accel(log.LongitudinalPersonality.relaxed)
  standard = traffic_accel(log.LongitudinalPersonality.standard)
  aggressive = traffic_accel(log.LongitudinalPersonality.aggressive)

  assert relaxed > standard > aggressive


def test_fresh_raw_can_target_survives_a_300ms_interprocess_planner_gap():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=7, distance=24.0,
  )

  arbitrator.apply(plan, sm, NOW_NS + 300_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.applied
  assert plan.aTarget < 0.4


def test_zero_remaining_distance_while_moving_keeps_braking_until_stopped():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=8, distance=0.0, v_ego=1.1,
  )

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.applied
  assert plan.aTarget < 0.0
  assert not plan.shouldStop
  assert not plan.allowThrottle


def test_hold_phase_while_vehicle_is_still_moving_keeps_braking():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  sm = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=8, distance=0.0, v_ego=0.7,
  )

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.hold
  assert arbitrator.diagnostics.applied
  assert plan.aTarget < 0.0
  assert plan.shouldStop
  assert not plan.allowThrottle


def test_terminal_stop_does_not_sample_zero_after_the_predicted_stop():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.5))
  plan = base_plan(a_target=0.1)
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=8, distance=0.0, v_ego=0.8,
  )
  sm["carState"].aEgo = -2.4

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.terminal_catch_active
  assert arbitrator.diagnostics.traffic_a_target < 0.0
  assert plan.aTarget < 0.0
  assert not plan.shouldStop
  assert not plan.allowThrottle


def test_low_speed_stop_enters_terminal_catch_before_distance_is_exhausted():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=9, distance=0.5, v_ego=1.1,
  )

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.terminal_catch_active
  assert plan.aTarget < 0.0
  assert not plan.shouldStop
  assert not plan.allowThrottle


def test_terminal_catch_survives_a_short_traffic_publisher_gap():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=9, distance=0.8, v_ego=1.5,
  )
  arbitrator.apply(base_plan(), sm, NOW_NS)
  sm.alive["trafficRadarState"] = False
  sm["carState"].vEgo = 1.4
  latched = base_plan(a_target=0.3)

  arbitrator.apply(latched, sm, NOW_NS + 100_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.hold
  assert not latched.shouldStop
  assert not latched.allowThrottle
  assert latched.aTarget < 0.0


def test_terminal_to_hold_sequence_brakes_until_actual_standstill():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.5))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=10, distance=0.8, v_ego=1.5,
  )

  terminal = base_plan(a_target=0.3)
  arbitrator.apply(terminal, sm, NOW_NS)

  sm.alive["trafficRadarState"] = False
  sm["carState"].vEgo = 1.0
  publisher_gap = base_plan(a_target=0.3)
  arbitrator.apply(publisher_gap, sm, NOW_NS + 50_000_000)

  sm["carState"].vEgo = 0.0
  standstill = base_plan(a_target=0.3)
  arbitrator.apply(standstill, sm, NOW_NS + 100_000_000)

  for moving_plan in (terminal, publisher_gap):
    assert moving_plan.aTarget < 0.0
    assert not moving_plan.shouldStop
    assert not moving_plan.allowThrottle
  assert standstill.aTarget == 0.0
  assert standstill.shouldStop
  assert not standstill.allowThrottle


def test_green_start_requires_same_event_hold_and_base_permission():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=9, distance=0.0, v_ego=0.0,
  )
  hold_plan = base_plan(a_target=0.0)
  arbitrator.apply(hold_plan, hold, NOW_NS)
  assert hold_plan.shouldStop

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=9, distance=0.0, v_ego=0.0,
  )
  start_plan = base_plan(a_target=0.1, should_stop=False)
  arbitrator.apply(start_plan, green, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.start
  assert arbitrator.diagnostics.start_applied
  assert 0.0 < start_plan.aTarget <= 1.60
  assert not start_plan.shouldStop

  continuing = base_plan(a_target=0.1, should_stop=False)
  arbitrator.apply(continuing, green, NOW_NS + 100_000_000)
  assert arbitrator.diagnostics.start_applied
  assert 0.0 < continuing.aTarget <= 1.60


def test_can_green_start_reaches_the_cp_low_speed_acceleration_envelope():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=10, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=10, distance=0.0, v_ego=0.0,
  )

  plan = base_plan(a_target=0.1, should_stop=False)
  for cycle in range(1, 21):
    plan = base_plan(a_target=0.1, should_stop=False)
    now_ns = NOW_NS + cycle * 50_000_000
    green["trafficRadarState"].publishMonoTime = now_ns
    arbitrator.apply(plan, green, now_ns)

  assert arbitrator.diagnostics.start_applied
  assert 1.20 <= plan.aTarget <= 1.60


def test_green_start_resumes_after_a_single_transient_physical_lead_cycle():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=30, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=30, distance=0.0, v_ego=0.0,
  )

  first = base_plan(a_target=0.1)
  arbitrator.apply(first, green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.start_applied

  green["radarState"].leadOne.present = True
  blocked = base_plan(a_target=0.1)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 100_000_000
  arbitrator.apply(blocked, green, NOW_NS + 100_000_000)
  assert not arbitrator.diagnostics.start_applied

  green["radarState"].leadOne.present = False
  resumed = base_plan(a_target=0.1)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 150_000_000
  arbitrator.apply(resumed, green, NOW_NS + 150_000_000)

  assert arbitrator.diagnostics.start_applied
  assert resumed.aTarget > 0.1


def test_can_authoritative_green_overrides_model_stop_and_negative_base_plan():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=11, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0), hold, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=11, distance=0.0, v_ego=0.0,
    base_model_stop=True,
  )
  start = base_plan(a_target=-0.2, should_stop=True)
  arbitrator.apply(start, green, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.start_applied
  assert start.aTarget > 0.0
  assert not start.shouldStop


def test_committed_hold_survives_traffic_publisher_loss_until_driver_override():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=12, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0), hold, NOW_NS)
  hold.alive["trafficRadarState"] = False
  latched = base_plan(a_target=0.3)

  arbitrator.apply(latched, hold, NOW_NS + 300_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.hold
  assert latched.shouldStop
  assert not latched.allowThrottle
  assert latched.aTarget == 0.0

  hold["carState"].brakePressed = True
  released = base_plan(a_target=0.3)
  arbitrator.apply(released, hold, NOW_NS + 350_000_000)
  assert not arbitrator.diagnostics.applied
  assert released.aTarget == 0.3


def test_physical_lead_suppresses_both_stop_and_start():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=13,
  )
  sm["radarState"].leadOne.present = True
  plan = base_plan()

  arbitrator.apply(plan, sm, NOW_NS)

  assert not arbitrator.diagnostics.applied
  assert plan.aTarget == 0.4


def test_turn_signal_allows_red_stop_but_still_blocks_green_start():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=31, distance=50.0, v_ego=12.0,
  )
  red["carControl"].rightBlinker = True
  stop_plan = base_plan()

  arbitrator.apply(stop_plan, red, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.applied
  assert stop_plan.aTarget < 0.4

  red["carControl"].rightBlinker = False
  red["carState"].vEgo = 0.0
  red["trafficRadarState"].phase = int(TrafficControlPhase.hold)
  red["trafficRadarState"].shouldStop = True
  red["trafficRadarState"].distanceToStopPoint = 0.0
  arbitrator.apply(base_plan(a_target=0.0), red, NOW_NS + 50_000_000)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=31, distance=0.0, v_ego=0.0,
  )
  green["carControl"].rightBlinker = True
  blocked_start = base_plan(a_target=0.1)
  arbitrator.apply(blocked_start, green, NOW_NS + 100_000_000)

  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.driverOverride


def test_green_start_remains_blocked_when_longitudinal_control_is_disabled():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=32, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0), hold, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True,
    start=True, event_id=32, distance=0.0, v_ego=0.0,
  )
  green["carControl"].enabled = False
  green["carControl"].longActive = False
  plan = base_plan(a_target=0.1)

  arbitrator.apply(plan, green, NOW_NS + 50_000_000)

  assert not arbitrator.diagnostics.start_applied
  assert plan.aTarget == 0.1


def test_publish_sink_forwards_unrelated_services_unchanged():
  sent = []
  pm = ns(send=lambda service, message: sent.append((service, message)))
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm()
  sink = arbitrator.publisher(pm, sm, NOW_NS)
  message = object()

  sink.send("driverAssistance", message)

  assert sent == [("driverAssistance", message)]


def test_disabled_or_non_tesla_sessions_do_not_create_an_arbitrator():
  params = ns(get_bool=lambda key: False)
  assert create_final_plan_arbitrator(ns(brand="tesla", longitudinalActuatorDelay=0.2), params) is None

  enabled = ns(get_bool=lambda key: True)
  assert create_final_plan_arbitrator(ns(brand="toyota", longitudinalActuatorDelay=0.2), enabled) is None
  assert isinstance(
    create_final_plan_arbitrator(ns(brand="tesla", longitudinalActuatorDelay=0.2), enabled),
    FinalPlanArbitrator,
  )


def test_plan_sp_schema_records_base_final_and_start_diagnostics():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  plan = base_plan()
  arbitrator.apply(
    plan,
    fake_sm(
      phase=TrafficControlPhase.braking, light_state=1, target=True,
      allowed=True, event_id=21, distance=18.0,
    ),
    NOW_NS,
  )
  message = messaging.new_message("longitudinalPlanSP")

  arbitrator.annotate_plan_sp(message.longitudinalPlanSP)

  diagnostics = message.longitudinalPlanSP.teslaTrafficControl
  assert diagnostics.applied
  assert diagnostics.action == int(TrafficPlanAction.stop)
  assert diagnostics.eventId == 21
  assert diagnostics.rawDistance == pytest.approx(23.0)
  assert diagnostics.baseATarget == pytest.approx(0.4)
  assert diagnostics.finalATarget == pytest.approx(plan.aTarget)
  assert not diagnostics.terminalCatchActive
  assert message.longitudinalPlanSP.aTarget == pytest.approx(plan.aTarget)
