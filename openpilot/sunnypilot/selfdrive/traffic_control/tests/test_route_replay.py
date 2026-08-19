import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlConfig, TrafficControlMode
from openpilot.sunnypilot.selfdrive.traffic_control.obstacle_state import TrafficObstacleGoPolicy, TrafficObstacleSource


FIXTURE = Path(__file__).parent / "fixtures" / "tesla_traffic_routes.json"


def ns(**kwargs):
  return SimpleNamespace(**kwargs)


class ReplaySubMaster:
  SERVICES = ('carStateSP', 'carState', 'carControl', 'radarState', 'modelV2')

  def __init__(self) -> None:
    self.seen = dict.fromkeys(self.SERVICES, True)
    self.alive = dict.fromkeys(self.SERVICES, True)
    self.valid = dict.fromkeys(self.SERVICES, True)
    self.messages = {}

  def __getitem__(self, key):
    return self.messages[key]

  def load(self, frame) -> int:
    now_ns = frame['tNs']
    available, valid_for_control, bus, control_source, distance, light, quality = frame['traffic']
    v_ego, a_ego, brake, left_blinker, right_blinker = frame['car']
    radar_valid, lead_present, lead_distance = frame['radar']
    model_valid, model_distance, model_terminal_speed = frame['model']
    traffic = ns(
      available=available, validForControl=valid_for_control, sourceBus=bus, dlc=6,
      featureState=3, stateMachine=4, controlSource=control_source, controlType=3,
      distance=distance, lightState=light, continuationReason=0, confirmationType=0,
      warningSuppressionReason=0, unavailableReason=0, visionLight=True,
      visionSign=False, visionRoadMarking=False, visionLine=False,
      frameMonoTime=now_ns - frame['trafficAgeNs'], quality=quality,
    )
    self.messages = {
      'carStateSP': ns(teslaTrafficControl=traffic),
      # The retained routes were recorded while disengaged. Replay forces the
      # agreed closed-course control gates so it exercises detection only; all
      # Tesla CAN, ego motion, model and real-lead observations remain recorded.
      'carState': ns(vEgo=v_ego, aEgo=a_ego, gasPressed=False, brakePressed=brake),
      'carControl': ns(enabled=True, longActive=True,
                       leftBlinker=left_blinker, rightBlinker=right_blinker),
      'radarState': ns(
        leadOne=ns(present=lead_present, dRel=lead_distance),
        leadTwo=ns(present=False, dRel=0.0),
      ),
      'modelV2': ns(
        position=ns(x=[model_distance] * 33),
        velocity=ns(x=[model_terminal_speed] * 33),
      ),
    }
    self.valid['carStateSP'] = frame['carStateSPValid']
    self.valid['radarState'] = radar_valid
    self.valid['modelV2'] = model_valid
    return now_ns


@pytest.mark.parametrize(
  ("route_name", "expected_present", "minimum_suppressed"),
  [
    ("00000009--72ea96171f--1", 28, 0),
    ("00000009--72ea96171f--2", 31, 0),
    ("00000009--72ea96171f--4", 0, 50),
  ],
)
def test_recorded_traffic_candidates_replay_deterministically(route_name, expected_present, minimum_suppressed):
  routes = {route['route']: route for route in json.loads(FIXTURE.read_text())['routes']}
  source = TrafficObstacleSource(
    TrafficControlConfig(
      mode=TrafficControlMode.stopGo, adaptive_reference=True, retain_event_with_lead=True,
    ),
    go_policy=TrafficObstacleGoPolicy.active,
  )
  sm = ReplaySubMaster()
  present_distances = []
  suppressed_frames = 0
  start_requests = 0

  for frame in routes[route_name]['frames']:
    obstacle = source.update(sm, sm.load(frame)).trafficObstacleState
    suppressed_frames += int(obstacle.suppressedByLead)
    start_requests += int(obstacle.startRequested)
    if obstacle.present:
      present_distances.append(float(obstacle.desiredStopDistance))

  positive_jumps = [
    current - previous
    for previous, current in zip(present_distances, present_distances[1:], strict=False)
    if current > previous
  ]
  assert len(present_distances) == expected_present
  assert suppressed_frames >= minimum_suppressed
  assert max(positive_jumps, default=0.0) <= 0.05
  assert start_requests == 0
