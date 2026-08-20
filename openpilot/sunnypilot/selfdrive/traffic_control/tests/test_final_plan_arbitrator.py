from types import SimpleNamespace as ns

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.final_plan_arbitrator import (
  FinalPlanArbitrator,
  TrafficPlanAction,
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
            v_ego=8.0, base_model_stop=False):
  traffic = ns(
    phase=int(phase), lightState=light_state, targetPresent=target,
    controlAllowed=allowed, plannerStartRequested=start, eventId=event_id,
    distanceToStopPoint=distance, publishMonoTime=NOW_NS, confidence=1.0,
    shouldStop=phase == TrafficControlPhase.hold, mode=4,
    oemTargetDistance=distance + 6.0, sourceBus=2, quality=2,
  )
  no_lead = ns(present=False)
  return FakeSubMaster({
    "trafficRadarState": traffic,
    "radarState": ns(leadOne=no_lead, leadTwo=ns(present=False)),
    "carState": ns(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False, vCruise=50.0),
    "carControl": ns(enabled=True, longActive=True, leftBlinker=False, rightBlinker=False),
    "modelV2": ns(action=ns(shouldStop=base_model_stop)),
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
  assert plan.shouldStop
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
  assert plan.shouldStop
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
  assert plan.shouldStop
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
  assert latched.shouldStop
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

  sm["trafficRadarState"].phase = int(TrafficControlPhase.hold)
  sm["trafficRadarState"].shouldStop = True
  sm["trafficRadarState"].distanceToStopPoint = 0.0
  sm["trafficRadarState"].publishMonoTime = NOW_NS + 50_000_000
  sm["carState"].vEgo = 0.7
  moving_hold = base_plan(a_target=0.3)
  arbitrator.apply(moving_hold, sm, NOW_NS + 50_000_000)

  sm.alive["trafficRadarState"] = False
  sm["carState"].vEgo = 0.4
  publisher_gap = base_plan(a_target=0.3)
  arbitrator.apply(publisher_gap, sm, NOW_NS + 100_000_000)

  sm["carState"].vEgo = 0.0
  standstill = base_plan(a_target=0.3)
  arbitrator.apply(standstill, sm, NOW_NS + 150_000_000)

  for moving_plan in (terminal, moving_hold, publisher_gap):
    assert moving_plan.aTarget < 0.0
    assert moving_plan.shouldStop
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
  assert 0.0 < start_plan.aTarget <= 0.60
  assert not start_plan.shouldStop

  continuing = base_plan(a_target=0.1, should_stop=False)
  arbitrator.apply(continuing, green, NOW_NS + 100_000_000)
  assert arbitrator.diagnostics.start_applied
  assert 0.0 < continuing.aTarget <= 0.60


def test_stable_green_start_reaches_a_responsive_but_bounded_acceleration():
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
  assert 0.45 <= plan.aTarget <= 0.60


def test_green_start_never_overrides_base_stop_or_negative_acceleration():
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
  blocked = base_plan(a_target=-0.2, should_stop=True)
  arbitrator.apply(blocked, green, NOW_NS + 50_000_000)

  assert not arbitrator.diagnostics.start_applied
  assert blocked.aTarget == -0.2
  assert blocked.shouldStop


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
  assert diagnostics.baseATarget == pytest.approx(0.4)
  assert diagnostics.finalATarget == pytest.approx(plan.aTarget)
  assert not diagnostics.terminalCatchActive
  assert message.longitudinalPlanSP.aTarget == pytest.approx(plan.aTarget)
