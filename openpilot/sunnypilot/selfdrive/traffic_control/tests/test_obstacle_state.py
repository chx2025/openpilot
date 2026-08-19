import openpilot.cereal.messaging as messaging
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlConfig, TrafficControlMode
from openpilot.sunnypilot.selfdrive.traffic_control.obstacle_state import (
  TrafficObstacleGoPolicy,
  TrafficObstacleMpcAdapter,
  TrafficObstacleSource,
)


def test_traffic_obstacle_channel_does_not_mutate_radar_state():
  radar_msg = messaging.new_message("radarState")
  obstacle_msg = messaging.new_message("trafficObstacleState")

  obstacle = obstacle_msg.trafficObstacleState
  obstacle.present = True
  obstacle.dRel = 42.0
  obstacle.desiredStopDistance = 36.0

  assert obstacle.present
  assert obstacle.dRel == 42.0
  assert obstacle.desiredStopDistance == 36.0
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
  source = TrafficObstacleSource(TrafficControlConfig(mode=TrafficControlMode.stopGo))

  message = None
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    message = source.update(sm, now_ns)

  obstacle = message.trafficObstacleState
  assert obstacle.present
  assert obstacle.validForControl
  assert obstacle.dRel == obstacle.desiredStopDistance + 6.0
  assert 0.0 < obstacle.desiredStopDistance < 74.0
  assert obstacle.eventId > 0
  assert not sm["radarState"].leadOne.present
  assert not sm["radarState"].leadTwo.present


def test_real_lead_suppresses_traffic_target_without_being_replaced():
  sm = red_light_sm()
  source = TrafficObstacleSource(TrafficControlConfig(mode=TrafficControlMode.stopGo))
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    source.update(sm, now_ns)

  real_lead = sm["radarState"].leadOne
  real_lead.present = True
  real_lead.dRel = 18.0
  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_100_000_000
  obstacle = source.update(sm, 1_100_000_000).trafficObstacleState

  assert not obstacle.present
  assert obstacle.suppressedByLead
  assert real_lead.present
  assert real_lead.dRel == 18.0


def test_suppressed_red_event_requires_model_reconfirmation_after_the_lead_clears():
  sm = red_light_sm()
  source = TrafficObstacleSource(TrafficControlConfig(
    mode=TrafficControlMode.stopGo, retain_event_with_lead=True,
  ))
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    confirmed = source.update(sm, now_ns).trafficObstacleState
  event_id = confirmed.eventId

  sm["radarState"].leadOne.present = True
  sm["radarState"].leadOne.dRel = 18.0
  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_100_000_000
  suppressed = source.update(sm, 1_100_000_000).trafficObstacleState
  assert suppressed.suppressedByLead
  assert suppressed.eventId == event_id

  sm["radarState"].leadOne.present = False
  for now_ns in range(1_200_000_000, 1_500_000_000, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    reacquiring = source.update(sm, now_ns).trafficObstacleState
    assert not reacquiring.present

  sm["carStateSP"].teslaTrafficControl.frameMonoTime = 1_600_000_000
  reacquired = source.update(sm, 1_600_000_000).trafficObstacleState
  assert reacquired.present
  assert reacquired.eventId == event_id


def released_green(go_policy: TrafficObstacleGoPolicy):
  sm = red_light_sm()
  sm["carState"].vEgo = 0.0
  sm["carStateSP"].teslaTrafficControl.distance = 12.0
  sm["modelV2"].position.x = [6.0] * 33
  source = TrafficObstacleSource(
    TrafficControlConfig(mode=TrafficControlMode.stopGo), go_policy=go_policy,
  )
  for now_ns in range(0, 1_000_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    source.update(sm, now_ns)

  sm["carStateSP"].teslaTrafficControl.lightState = 2
  message = None
  for now_ns in range(1_100_000_000, 1_900_000_001, 100_000_000):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    message = source.update(sm, now_ns)
  return message.trafficObstacleState


def test_passive_green_removes_target_without_start_request():
  obstacle = released_green(TrafficObstacleGoPolicy.passive)
  assert not obstacle.present
  assert not obstacle.startRequested


def test_active_green_requests_longitudinal_start_without_creating_a_target():
  obstacle = released_green(TrafficObstacleGoPolicy.active)
  assert not obstacle.present
  assert obstacle.startRequested
  assert obstacle.eventId > 0


def test_mpc_adapter_consumes_target_without_publishing_a_fake_radar_lead():
  class RecordingMpc:
    runtime_tuning = ns(stop_distance=5.5)

    def update(self, radar_state, personality=0):
      self.received = radar_state

  physical_radar = ns(
    leadOne=ns(present=False),
    leadTwo=ns(present=False),
  )
  mpc = TrafficObstacleMpcAdapter(RecordingMpc())
  mpc.set_obstacle(ns(present=True, desiredStopDistance=30.0))

  mpc.update(physical_radar)

  assert mpc.received.leadTwo.present
  assert mpc.received.leadTwo.dRel == 35.5
  assert mpc.received.leadTwo.vLead == 0.0
  assert not physical_radar.leadOne.present
  assert not physical_radar.leadTwo.present
