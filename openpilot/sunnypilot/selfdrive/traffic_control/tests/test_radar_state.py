"""Public behavior tests for the independent Traffic Radar state."""

import openpilot.cereal.messaging as messaging
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlConfig, TrafficControlMode
from openpilot.sunnypilot.selfdrive.traffic_control.radar_state import (
  TrafficRadarGoPolicy,
  TrafficRadarSource,
)


def test_traffic_radar_channel_does_not_mutate_radar_state():
  radar_msg = messaging.new_message("radarState")
  traffic_msg = messaging.new_message("trafficRadarState")

  target = traffic_msg.trafficRadarState
  target.targetPresent = True
  target.oemTargetDistance = 42.0
  target.distanceToStopPoint = 36.0

  assert target.targetPresent
  assert target.oemTargetDistance == 42.0
  assert target.distanceToStopPoint == 36.0
  assert not radar_msg.radarState.leadOne.present
  assert not radar_msg.radarState.leadTwo.present


def ns(**kwargs):
  return SimpleNamespace(**kwargs)


def red_light_sm():
  traffic = ns(
    available=True, validForControl=True, sourceBus=2, dlc=6, featureState=3,
    stateMachine=4, controlSource=3, controlType=3, distance=80.0,
    lightState=1, continuationReason=0, confirmationType=0,
    warningSuppressionReason=0, unavailableReason=0, visionLight=True,
    visionSign=False, visionRoadMarking=False, visionLine=False,
    frameMonoTime=0, quality=2,
  )
  messages = {
    "carStateSP": ns(teslaTrafficControl=traffic),
    "carState": ns(vEgo=10.0, aEgo=0.0, gasPressed=False, brakePressed=False),
    "carControl": ns(enabled=True, longActive=True, leftBlinker=False, rightBlinker=False),
    "radarState": ns(leadOne=ns(present=False, dRel=0.0), leadTwo=ns(present=False, dRel=0.0)),
    "modelV2": ns(position=ns(x=[74.0] * 33), velocity=ns(x=[0.0] * 33)),
  }

  class FakeSubMaster:
    seen = dict.fromkeys(messages, True)
    alive = dict.fromkeys(messages, True)
    valid = dict.fromkeys(messages, True)

    def __getitem__(self, key):
      return messages[key]

  return FakeSubMaster()


def test_confirmed_red_is_published_as_a_separate_radar_like_target():
  sm = red_light_sm()
  source = TrafficRadarSource(TrafficControlConfig(mode=TrafficControlMode.stopGo))

  message = None
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    message = source.update(sm, now_ns)

  target = message.trafficRadarState
  assert target.targetPresent
  assert target.controlAllowed
  assert target.oemTargetDistance == target.distanceToStopPoint + 6.0
  assert 0.0 < target.distanceToStopPoint < 74.0
  assert target.eventId > 0
  assert not sm["radarState"].leadOne.present
  assert not sm["radarState"].leadTwo.present


def test_stale_observation_keeps_raw_age_and_distance_only_for_diagnostics():
  sm = red_light_sm()
  source = TrafficRadarSource(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 100_000_000

  target = source.update(sm, 400_000_000).trafficRadarState

  assert not target.targetPresent
  assert not target.controlAllowed
  assert target.observationAgeMs == 300.0
  assert target.rawDistance == 80.0


def test_real_lead_suppresses_traffic_target_without_being_replaced():
  sm = red_light_sm()
  source = TrafficRadarSource(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    source.update(sm, now_ns)

  real_lead = sm["radarState"].leadOne
  real_lead.present = True
  real_lead.dRel = 18.0
  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_100_000_000
  target = source.update(sm, 1_100_000_000).trafficRadarState

  assert not target.controlAllowed
  assert target.suppressedByPhysicalLead
  assert real_lead.present
  assert real_lead.dRel == 18.0


def test_suppressed_red_event_requires_model_reconfirmation_after_the_lead_clears():
  sm = red_light_sm()
  source = TrafficRadarSource(TrafficControlConfig(
    mode=TrafficControlMode.stopGo, retain_event_with_lead=True,
  ))
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    confirmed = source.update(sm, now_ns).trafficRadarState
  event_id = confirmed.eventId

  sm["radarState"].leadOne.present = True
  sm["radarState"].leadOne.dRel = 18.0
  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_100_000_000
  suppressed = source.update(sm, 1_100_000_000).trafficRadarState
  assert suppressed.suppressedByPhysicalLead
  assert suppressed.eventId == event_id

  sm["radarState"].leadOne.present = False
  for now_ns in range(1_200_000_000, 1_500_000_000, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    reacquiring = source.update(sm, now_ns).trafficRadarState
    assert not reacquiring.controlAllowed

  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_600_000_000
  reacquired = source.update(sm, 1_600_000_000).trafficRadarState
  assert reacquired.controlAllowed
  assert reacquired.eventId == event_id


def released_green(go_policy: TrafficRadarGoPolicy, *, feature_zero_green: bool = False):
  sm = red_light_sm()
  sm["carState"].vEgo = 0.0
  sm["carStateSP"].teslaTrafficControl.distance = 12.0
  sm["modelV2"].position.x = [6.0] * 33
  source = TrafficRadarSource(
    TrafficControlConfig(mode=TrafficControlMode.stopGo), go_policy=go_policy,
  )
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    source.update(sm, now_ns)

  sm["carStateSP"].teslaTrafficControl.lightState = 2
  if feature_zero_green:
    sm["carStateSP"].teslaTrafficControl.validForControl = False
    sm["carStateSP"].teslaTrafficControl.featureState = 0
    sm["carStateSP"].teslaTrafficControl.stateMachine = 6
    sm["carStateSP"].teslaTrafficControl.unavailableReason = 1
  message = None
  for now_ns in range(1_100_000_000, 1_900_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    message = source.update(sm, now_ns)
  return message.trafficRadarState


def test_passive_green_removes_target_without_start_request():
  target = released_green(TrafficRadarGoPolicy.passive)
  assert not target.targetPresent
  assert not target.plannerStartRequested


def test_active_green_requests_longitudinal_start_without_creating_a_target():
  target = released_green(TrafficRadarGoPolicy.active)
  assert not target.targetPresent
  assert target.plannerStartRequested
  assert target.eventId > 0


def test_feature_zero_green_requests_start_only_after_the_same_event_holds():
  target = released_green(TrafficRadarGoPolicy.active, feature_zero_green=True)

  assert not target.targetPresent
  assert target.plannerStartRequested
  assert target.eventId > 0
  assert target.rawGreenSeen
  assert target.releaseEligible
  assert target.eventTransitionReason > 0


def test_feature_zero_green_without_a_held_red_event_never_requests_start():
  sm = red_light_sm()
  traffic = sm["carStateSP"].teslaTrafficControl
  traffic.validForControl = False
  traffic.featureState = 0
  traffic.stateMachine = 6
  traffic.unavailableReason = 1
  traffic.lightState = 2
  traffic.distance = 12.0
  sm["carState"].vEgo = 0.0
  source = TrafficRadarSource(
    TrafficControlConfig(mode=TrafficControlMode.stopGo),
    go_policy=TrafficRadarGoPolicy.active,
  )

  target = None
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    traffic.frameMonoTime = now_ns
    target = source.update(sm, now_ns).trafficRadarState

  assert not target.targetPresent
  assert not target.plannerStartRequested
  assert target.eventId == 0
  assert target.rawGreenSeen
  assert not target.releaseEligible
