from types import SimpleNamespace as ns

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.final_plan_arbitrator import (
  FinalPlanArbitrator,
  START_JERK_LIMIT,
  START_MAX_ACCEL,
  START_MAX_DURATION_NS,
  START_NEAR_LEAD_DISTANCE,
  START_MAX_SPEED,
  TrafficPlanAction,
  TrafficStartBlockReason,
  create_final_plan_arbitrator,
)


NOW_NS = 1_000_000_000


def test_go_constants_and_first_cycle_numerics_are_frozen():
  assert START_MAX_ACCEL == 1.6
  assert START_MAX_SPEED == 2.5
  assert START_MAX_DURATION_NS == 3_000_000_000
  assert START_JERK_LIMIT == 1.0
  assert START_NEAR_LEAD_DISTANCE == 20.0

  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=100, session_id=100, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=101, session_id=100, distance=0.0, v_ego=0.0,
  )
  plan = base_plan(a_target=0.1)
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)
  assert plan.aTarget == pytest.approx(0.25)
  assert not plan.shouldStop
  assert plan.allowThrottle
  np.testing.assert_allclose(plan.speeds, [
    0.0, 0.000095367431640625, 0.001239776611328125, 0.00553131103515625,
    0.01621246337890625, 0.037670135498046875, 0.07543563842773438,
    0.1361846923828125, 0.2277374267578125, 0.3590583801269531,
    0.5402565002441406, 0.7825851440429688, 1.0984420776367188,
    1.4890670776367188, 1.9109420776367188, 2.3640670776367188,
    2.8484420776367188,
  ])
  np.testing.assert_allclose(plan.accels, [
    0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625,
    0.3515625, 0.478515625, 0.625, 0.791015625, 0.9765625, 1.181640625,
    1.40625, 1.6, 1.6, 1.6, 1.6,
  ])
  np.testing.assert_allclose(plan.jerks, [
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    0.7936000000000004, 0.0, 0.0, 0.0, 0.0,
  ])


def test_moving_green_release_leaves_the_base_plan_unchanged():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=102, session_id=102, distance=25.0, v_ego=8.0,
  )
  arbitrator.apply(base_plan(a_target=-0.8, should_stop=True), red, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=103, session_id=102, distance=0.0, v_ego=8.0,
  )
  plan = base_plan(a_target=-0.8, should_stop=True)
  original = plan_output(plan)
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)
  assert plan_output(plan) == original
  assert arbitrator.diagnostics.start_requested
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.action == TrafficPlanAction.none


class FakeSubMaster:
  def __init__(self, values):
    self.values = values
    self.seen = dict.fromkeys(values, True)
    self.alive = dict.fromkeys(values, True)
    self.valid = dict.fromkeys(values, True)

  def __getitem__(self, key):
    return self.values[key]


def base_plan(*, a_target=0.4, should_stop=False, has_lead=False,
              source=log.LongitudinalPlan.LongitudinalPlanSource.cruise):
  return ns(
    speeds=[8.0] * 17,
    accels=[a_target] * 17,
    jerks=[0.0] * 17,
    aTarget=a_target,
    shouldStop=should_stop,
    allowThrottle=True,
    hasLead=has_lead,
    longitudinalPlanSource=source,
  )


def plan_output(plan):
  return (list(plan.speeds), list(plan.accels), list(plan.jerks), plan.aTarget,
          plan.shouldStop, plan.allowThrottle, plan.hasLead)


def fake_sm(*, phase=TrafficControlPhase.off, light_state=0, target=False,
            allowed=False, start=False, event_id=0, distance=30.0,
            v_ego=8.0, base_model_stop=False, session_id=None, direction_unknown=False,
            personality=log.LongitudinalPersonality.standard):
  traffic = ns(
    phase=int(phase), lightState=light_state, targetPresent=target,
    controlAllowed=allowed, plannerStartRequested=start, eventId=event_id,
    distanceToStopPoint=distance, publishMonoTime=NOW_NS, confidence=1.0,
    shouldStop=phase == TrafficControlPhase.hold, mode=4,
    oemTargetDistance=distance + 5.0, rawDistance=distance + 5.0, sourceBus=2, quality=2,
    stopSessionId=event_id if session_id is None else session_id,
    directionUnknown=direction_unknown,
    driverOverrideActive=False, canRemaining=distance,
    stationInnovation=0.0,
    stopControlAllowed=allowed, stopSafetyAllowed=allowed,
    rawObservationFresh=True, observationAgeMs=0.0,
    stopDirectionUnknown=direction_unknown,
  )
  no_lead = ns(present=False, dRel=0.0)
  return FakeSubMaster({
    "trafficRadarState": traffic,
    "radarState": ns(leadOne=no_lead, leadTwo=ns(present=False, dRel=0.0)),
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


def test_stop_and_start_do_not_require_a_radar_state_subscription():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=110, session_id=110, distance=0.0, v_ego=0.0,
  )
  for health in (red.seen, red.alive, red.valid):
    health.pop("radarState")
  red.values.pop("radarState")
  held = base_plan(a_target=0.0, should_stop=True)
  arbitrator.apply(held, red, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.hold

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=111, session_id=110, distance=0.0, v_ego=0.0,
  )
  for health in (green.seen, green.alive, green.valid):
    health.pop("radarState")
  green.values.pop("radarState")
  plan = base_plan(a_target=0.1)
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.start
  assert arbitrator.diagnostics.start_applied
  assert plan.aTarget == pytest.approx(0.25)

  rolling = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  moving_red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=112, session_id=112, distance=25.0, v_ego=8.0,
  )
  for health in (moving_red.seen, moving_red.alive, moving_red.valid):
    health.pop("radarState")
  moving_red.values.pop("radarState")
  rolling.apply(base_plan(a_target=-0.8, should_stop=True), moving_red, NOW_NS)

  moving_green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=113, session_id=112, distance=0.0, v_ego=8.0,
  )
  for health in (moving_green.seen, moving_green.alive, moving_green.valid):
    health.pop("radarState")
  moving_green.values.pop("radarState")
  rolling_plan = base_plan(a_target=-0.8, should_stop=True)
  rolling.apply(rolling_plan, moving_green, NOW_NS + 50_000_000)

  assert rolling.diagnostics.action == TrafficPlanAction.none
  assert rolling.diagnostics.start_requested
  assert not rolling.diagnostics.start_applied
  assert rolling_plan.aTarget == pytest.approx(-0.8)
  assert rolling_plan.shouldStop


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


def test_standard_style_arms_before_route5a_high_speed_stop_becomes_harsh():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=75, session_id=75, distance=115.0, v_ego=17.3,
    personality=log.LongitudinalPersonality.standard,
  )
  plan = base_plan(a_target=-0.3)

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.traffic_a_target < 0.0


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


def test_target_replacement_keeps_an_armed_stop_for_the_same_session():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  first = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=80, session_id=20, distance=30.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), first, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  replacement = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=81, session_id=20, distance=100.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), replacement, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop


def test_missing_radar_state_preserves_committed_traffic_hold():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=82, session_id=21, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)
  for health in (hold.seen, hold.alive, hold.valid):
    health.pop("radarState")
  hold.values.pop("radarState")
  plan = base_plan(a_target=0.25, should_stop=False)
  arbitrator.apply(plan, hold, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.hold
  assert plan.shouldStop
  assert plan.aTarget <= 0.0


def test_armed_stop_has_bounded_raw_can_dropout_grace_and_hold_stays_latched():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  stopping = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=85, session_id=24, distance=30.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), stopping, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  stopping["trafficRadarState"].stopControlAllowed = False
  stopping["trafficRadarState"].rawObservationFresh = False
  stopping["trafficRadarState"].observationAgeMs = 900.0
  arbitrator.apply(base_plan(), stopping, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  stopping["trafficRadarState"].observationAgeMs = 5000.0
  arbitrator.apply(base_plan(), stopping, NOW_NS + 100_000_000)
  assert arbitrator.diagnostics.action != TrafficPlanAction.stop

  held = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=86, session_id=25, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), held, NOW_NS + 150_000_000)
  held["trafficRadarState"].stopControlAllowed = False
  held["trafficRadarState"].rawObservationFresh = False
  held["trafficRadarState"].observationAgeMs = 5000.0
  plan = base_plan(a_target=0.2)
  arbitrator.apply(plan, held, NOW_NS + 200_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.hold
  assert plan.shouldStop


def test_stale_grace_never_bypasses_a_stop_safety_gate():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  stopping = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=114, session_id=114, distance=30.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), stopping, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  stopping["trafficRadarState"].stopControlAllowed = False
  stopping["trafficRadarState"].stopSafetyAllowed = False
  stopping["trafficRadarState"].stopDirectionUnknown = True
  stopping["trafficRadarState"].rawObservationFresh = False
  stopping["trafficRadarState"].observationAgeMs = 900.0
  plan = base_plan()
  arbitrator.apply(plan, stopping, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.release
  assert plan.aTarget > 0.0


def test_new_off_frame_at_zero_age_keeps_an_armed_stop_continuous():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  stopping = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=115, session_id=115, distance=30.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), stopping, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  stopping["trafficRadarState"].stopControlAllowed = False
  stopping["trafficRadarState"].stopSafetyAllowed = True
  stopping["trafficRadarState"].rawObservationFresh = False
  stopping["trafficRadarState"].observationAgeMs = 0.0
  stopping["trafficRadarState"].rawDistance = 254.0
  arbitrator.apply(base_plan(), stopping, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop


def test_missing_raw_frame_still_cannot_use_zero_age_dropout_grace():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  stopping = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=116, session_id=116, distance=30.0, v_ego=10.0,
  )
  arbitrator.apply(base_plan(), stopping, NOW_NS)
  stopping["trafficRadarState"].stopControlAllowed = False
  stopping["trafficRadarState"].rawObservationFresh = False
  stopping["trafficRadarState"].observationAgeMs = 0.0
  stopping["trafficRadarState"].rawDistance = 255.0

  arbitrator.apply(base_plan(), stopping, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action != TrafficPlanAction.stop


def test_stop_is_shadow_when_physics_cannot_stop_within_trusted_distance():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=83, session_id=22, distance=195.0,
    v_ego=120.0 / 3.6,
  )
  plan = base_plan(a_target=0.2)
  arbitrator.apply(plan, sm, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.none
  assert plan.aTarget == 0.2


def test_high_speed_stop_still_applies_when_physics_are_feasible():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=84, session_id=23, distance=150.0,
    v_ego=95.0 / 3.6,
  )
  plan = base_plan(a_target=0.2)
  arbitrator.apply(plan, sm, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert plan.aTarget < 0.2


def test_green_start_survives_one_cycle_of_start_request_dropout():
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

  # A single publisher-cycle request dropout must pause, not complete, GO.
  green["trafficRadarState"].plannerStartRequested = False
  arbitrator.apply(base_plan(a_target=0.4, should_stop=True), green, NOW_NS + 100_000_000)

  green["trafficRadarState"].plannerStartRequested = True
  resumed = base_plan(a_target=0.4, should_stop=True)
  arbitrator.apply(resumed, green, NOW_NS + 150_000_000)
  assert arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.none


def test_green_release_uses_stable_stop_session_across_target_event_replacement():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True, allowed=True,
    event_id=70, session_id=9, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(should_stop=True), red, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=71, session_id=9, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=-0.1, should_stop=True), green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.none


def test_seen_only_session_keeps_low_speed_go_but_moving_release_is_transparent():
  rolling = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  far_red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=87, session_id=26, distance=190.0, v_ego=8.0,
  )
  rolling.apply(base_plan(a_target=-0.8, should_stop=True), far_red, NOW_NS)
  assert rolling.diagnostics.action == TrafficPlanAction.none
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, target=False,
    allowed=True, start=True, event_id=88, session_id=26, distance=0.0, v_ego=8.0,
  )
  plan = base_plan(a_target=-0.8, should_stop=True)
  original = plan_output(plan)
  rolling.apply(plan, green, NOW_NS + 50_000_000)
  assert rolling.diagnostics.action == TrafficPlanAction.none
  assert not rolling.diagnostics.start_applied
  assert plan_output(plan) == original

  low = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  low_red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=89, session_id=27, distance=190.0, v_ego=0.0,
  )
  low.apply(base_plan(a_target=0.0), low_red, NOW_NS)
  low_green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, target=False,
    allowed=True, start=True, event_id=90, session_id=27, distance=0.0, v_ego=0.0,
  )
  low_plan = base_plan(a_target=0.1)
  low.apply(low_plan, low_green, NOW_NS + 50_000_000)
  assert low.diagnostics.start_applied
  assert low_plan.aTarget == pytest.approx(0.25)


def test_owned_green_while_moving_clears_traffic_state_without_overriding_model():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True, allowed=True,
    event_id=72, session_id=10, distance=25.0, v_ego=8.0,
  )
  arbitrator.apply(base_plan(a_target=-0.8, should_stop=True), red, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=73, session_id=10, distance=0.0, v_ego=8.0,
  )
  plan = base_plan(a_target=-0.8, should_stop=True)
  original = plan_output(plan)
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.action == TrafficPlanAction.none
  assert not arbitrator.diagnostics.start_applied
  assert plan_output(plan) == original


def test_unowned_generic_green_never_overrides_base_e2e_stop():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  green = fake_sm(
    phase=TrafficControlPhase.off, light_state=2, target=False, allowed=False,
    event_id=0, session_id=0, distance=0.0, v_ego=8.0,
  )
  plan = base_plan(a_target=-0.8, should_stop=True)
  original = (list(plan.speeds), list(plan.accels), plan.aTarget, plan.shouldStop)
  arbitrator.apply(plan, green, NOW_NS)
  assert (plan.speeds, plan.accels, plan.aTarget, plan.shouldStop) == original


def test_passed_event_immediately_removes_the_traffic_release_tail():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  red = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=116, session_id=116, distance=20.0, v_ego=6.0,
  )
  arbitrator.apply(base_plan(a_target=-0.5), red, NOW_NS)
  assert arbitrator.diagnostics.action == TrafficPlanAction.stop

  passed = fake_sm(
    phase=TrafficControlPhase.passed, light_state=1, target=False,
    allowed=False, event_id=116, session_id=0, distance=0.0, v_ego=6.0,
  )
  passed["trafficRadarState"].rawDistance = 254.0
  plan = base_plan(a_target=0.4)
  original = (list(plan.speeds), list(plan.accels), list(plan.jerks), plan.aTarget,
              plan.shouldStop, plan.allowThrottle)
  arbitrator.apply(plan, passed, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.action == TrafficPlanAction.none
  assert (plan.speeds, plan.accels, plan.jerks, plan.aTarget,
          plan.shouldStop, plan.allowThrottle) == original


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
  assert plan.aTarget == pytest.approx(1.2)


def test_distant_unselected_visual_lead_does_not_block_green_start():
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

  green["radarState"].leadOne.present = True
  green["radarState"].leadOne.dRel = START_NEAR_LEAD_DISTANCE + 30.0
  unaffected = base_plan(
    a_target=0.1, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.e2e,
  )
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(unaffected, green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.start_applied
  assert unaffected.aTarget > 0.1


def test_real_capnp_plan_source_enum_is_supported():
  message = messaging.new_message("longitudinalPlan")
  plan = message.longitudinalPlan

  plan.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.lead0
  assert FinalPlanArbitrator._base_plan_lead_source(plan) == int(
    log.LongitudinalPlan.LongitudinalPlanSource.lead0,
  )

  plan.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.e2e
  assert FinalPlanArbitrator._base_plan_lead_source(plan) is None


def test_fake_lead_plan_source_without_a_published_lead_does_not_block_go():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=35, session_id=35, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(
    base_plan(
      a_target=0.0, should_stop=True, has_lead=False,
      source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
    ),
    hold, NOW_NS,
  )
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=36, session_id=35, distance=0.0, v_ego=0.0,
  )
  plan = base_plan(
    a_target=0.1, should_stop=True, has_lead=False,
    source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
  )
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)

  assert arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.none
  assert plan.aTarget > 0.1


def test_hold_near_lead_delegation_survives_first_green_frame_dropout():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=31, session_id=31, distance=0.0, v_ego=0.0,
  )
  hold["radarState"].leadOne.present = True
  hold["radarState"].leadOne.dRel = 4.0
  arbitrator.apply(
    base_plan(
      a_target=0.0, should_stop=True, has_lead=True,
      source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
    ),
    hold, NOW_NS,
  )

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=32, session_id=31, distance=0.0, v_ego=0.0,
  )
  dropped = base_plan(
    a_target=0.1, should_stop=True, has_lead=False,
    source=log.LongitudinalPlan.LongitudinalPlanSource.e2e,
  )
  original = plan_output(dropped)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(dropped, green, NOW_NS + 50_000_000)

  assert plan_output(dropped) == original
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead


def test_unhealthy_radar_falls_back_to_published_has_lead():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=33, session_id=33, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=34, session_id=33, distance=0.0, v_ego=0.0,
  )
  green.alive["radarState"] = False
  plan = base_plan(
    a_target=0.1, should_stop=True, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.e2e,
  )
  original = plan_output(plan)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)

  assert plan_output(plan) == original
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead


@pytest.mark.parametrize(("v_ego", "a_target", "should_stop"), [
  (0.0, -0.4, True),
  (8.0, -0.7, True),
  (1.0, 0.5, False),
])
def test_green_go_with_selected_lead_leaves_base_plan_unchanged(v_ego, a_target, should_stop):
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=130, session_id=130, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)

  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=131, session_id=130, distance=0.0, v_ego=v_ego,
  )
  green["radarState"].leadOne.present = True
  green["radarState"].leadOne.dRel = 50.0
  plan = base_plan(
    a_target=a_target, should_stop=should_stop, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
  )
  original = plan_output(plan)

  arbitrator.apply(plan, green, NOW_NS + 50_000_000)

  assert plan_output(plan) == original
  assert arbitrator.diagnostics.start_requested
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead
  assert arbitrator.diagnostics.action == TrafficPlanAction.none


def test_selected_lead_delegates_the_entire_stop_session_to_the_base_planner():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=132, session_id=132, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=133, session_id=132, distance=0.0, v_ego=0.0,
  )
  green["radarState"].leadOne.present = True
  green["radarState"].leadOne.dRel = 50.0

  blocked = base_plan(
    a_target=-0.3, should_stop=True, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
  )
  original = plan_output(blocked)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(blocked, green, NOW_NS + 50_000_000)
  assert plan_output(blocked) == original
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead

  # Once delegated, a distant/unselected or temporarily missing lead cannot
  # produce an acceleration pulse later in the same stop session.
  green["radarState"].leadOne.present = True
  green["radarState"].leadOne.dRel = START_NEAR_LEAD_DISTANCE + 30.0
  still_delegated = base_plan(
    a_target=-0.2, should_stop=True, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.e2e,
  )
  original = plan_output(still_delegated)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 550_000_000
  arbitrator.apply(still_delegated, green, NOW_NS + 550_000_000)
  assert plan_output(still_delegated) == original
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead

def test_route6d_lead_flicker_cannot_emit_a_start_pulse():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=134, session_id=134, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=135, session_id=134, distance=0.0, v_ego=0.0,
  )

  for offset_ns, has_lead in (
    (50_000_000, True),
    (100_000_000, False),
    (200_000_000, False),
    (300_000_000, True),
    (350_000_000, False),
    (450_000_000, False),
    (550_000_000, False),
    (650_000_000, False),
  ):
    green["radarState"].leadOne.present = has_lead
    green["radarState"].leadOne.dRel = 4.0 if has_lead else 0.0
    source = (log.LongitudinalPlan.LongitudinalPlanSource.lead0 if has_lead
              else log.LongitudinalPlan.LongitudinalPlanSource.e2e)
    plan = base_plan(a_target=-0.1, should_stop=True, has_lead=has_lead, source=source)
    original = plan_output(plan)
    green["trafficRadarState"].publishMonoTime = NOW_NS + offset_ns
    arbitrator.apply(plan, green, NOW_NS + offset_ns)
    assert plan_output(plan) == original
    assert not arbitrator.diagnostics.start_applied
    assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead

  stable_clear = base_plan(a_target=0.1)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 750_000_000
  arbitrator.apply(stable_clear, green, NOW_NS + 750_000_000)
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead


def test_near_visual_lead_delegation_survives_an_interrupted_release():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=140, session_id=140, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), hold, NOW_NS)
  green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=141, session_id=140, distance=0.0, v_ego=0.0,
  )

  green["radarState"].leadOne.present = True
  green["radarState"].leadOne.dRel = START_NEAR_LEAD_DISTANCE
  plan = base_plan(
    a_target=0.0, should_stop=True, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.e2e,
  )
  green["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  arbitrator.apply(plan, green, NOW_NS + 50_000_000)
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead

  green["trafficRadarState"].plannerStartRequested = False
  green["trafficRadarState"].publishMonoTime = NOW_NS + 250_000_000
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), green, NOW_NS + 250_000_000)

  green["trafficRadarState"].plannerStartRequested = True
  green["radarState"].leadOne.present = False
  still_delegated = base_plan(a_target=0.0, should_stop=True)
  original = plan_output(still_delegated)
  green["trafficRadarState"].publishMonoTime = NOW_NS + 600_000_000
  arbitrator.apply(still_delegated, green, NOW_NS + 600_000_000)
  assert plan_output(still_delegated) == original
  assert not arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead


def test_lead_delegation_does_not_leak_into_a_new_stop_session():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  first_hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=136, session_id=136, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), first_hold, NOW_NS)
  first_green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=137, session_id=136, distance=0.0, v_ego=0.0,
  )
  blocked = base_plan(
    a_target=0.0, should_stop=True, has_lead=True,
    source=log.LongitudinalPlan.LongitudinalPlanSource.lead0,
  )
  first_green["radarState"].leadOne.present = True
  first_green["radarState"].leadOne.dRel = 4.0
  arbitrator.apply(blocked, first_green, NOW_NS + 50_000_000)
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.physicalLead

  second_hold = fake_sm(
    phase=TrafficControlPhase.hold, light_state=1, target=True,
    allowed=True, event_id=138, session_id=138, distance=0.0, v_ego=0.0,
  )
  arbitrator.apply(base_plan(a_target=0.0, should_stop=True), second_hold, NOW_NS + 100_000_000)
  second_green = fake_sm(
    phase=TrafficControlPhase.release, light_state=2, allowed=True, start=True,
    event_id=139, session_id=138, distance=0.0, v_ego=0.0,
  )
  immediate = base_plan(a_target=0.1)
  arbitrator.apply(immediate, second_green, NOW_NS + 150_000_000)

  assert arbitrator.diagnostics.start_applied
  assert arbitrator.diagnostics.start_block_reason == TrafficStartBlockReason.none
  assert immediate.aTarget > 0.1


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


def test_published_lead_does_not_suppress_traffic_stop():
  arbitrator = FinalPlanArbitrator(ns(longitudinalActuatorDelay=0.2))
  sm = fake_sm(
    phase=TrafficControlPhase.braking, light_state=1, target=True,
    allowed=True, event_id=13,
  )
  sm["radarState"].leadOne.present = True
  plan = base_plan(has_lead=True)

  arbitrator.apply(plan, sm, NOW_NS)

  assert arbitrator.diagnostics.action == TrafficPlanAction.stop
  assert arbitrator.diagnostics.applied
  assert plan.aTarget < 0.4


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
      allowed=True, event_id=21, distance=24.0,
    ),
    NOW_NS,
  )
  message = messaging.new_message("longitudinalPlanSP")

  arbitrator.annotate_plan_sp(message.longitudinalPlanSP)

  diagnostics = message.longitudinalPlanSP.teslaTrafficControl
  assert diagnostics.applied
  assert diagnostics.action == int(TrafficPlanAction.stop)
  assert diagnostics.eventId == 21
  assert diagnostics.rawDistance == pytest.approx(29.0)
  assert diagnostics.baseATarget == pytest.approx(0.4)
  assert diagnostics.finalATarget == pytest.approx(plan.aTarget)
  assert not diagnostics.terminalCatchActive
  assert message.longitudinalPlanSP.aTarget == pytest.approx(plan.aTarget)
