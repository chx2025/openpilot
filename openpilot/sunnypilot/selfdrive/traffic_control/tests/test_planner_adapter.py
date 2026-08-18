from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.traffic_control import decorate_planner
from openpilot.sunnypilot.selfdrive.traffic_control import planner_adapter as planner_adapter_module
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlDecision, TrafficControlMode, TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.planner_adapter import TrafficControlPlannerAdapter


class FakeParams:
  def __init__(self, mode=0, adaptive_reference=False):
    self.mode = mode
    self.adaptive_reference = adaptive_reference

  def get(self, key, *, return_default=False):
    assert return_default
    return {
      "TeslaTrafficControlMode": self.mode,
      "TeslaTrafficStopReference": 60,
      "TeslaTrafficAdaptiveReference": self.adaptive_reference,
    }[key]


class FakePlanner:
  def __init__(self):
    self.v_desired_trajectory = np.linspace(10.0, 8.0, 17)
    self.a_desired_trajectory = np.linspace(-0.1, -0.3, 17)
    self.j_desired_trajectory = np.zeros(17)
    self.output_a_target = -0.2
    self.output_should_stop = False
    self.allow_throttle = True
    self.a_desired = -0.2
    self.v_desired_filter = ns(x=9.5)
    self.update_calls = 0

  def update(self, sm):
    del sm
    self.update_calls += 1

  def publish(self, sm, pm):
    del sm, pm


def ns(**kwargs):
  return SimpleNamespace(**kwargs)


def fake_sm():
  traffic = ns(
    available=True, validForControl=True, sourceBus=2, dlc=6, featureState=3,
    stateMachine=4, controlSource=3, controlType=3, distance=80.0, lightState=1,
    continuationReason=0, confirmationType=0, warningSuppressionReason=0,
    unavailableReason=0, visionLight=True, visionSign=False, visionRoadMarking=False,
    visionLine=False, frameMonoTime=1, quality=2,
  )
  messages = {
    "carStateSP": ns(teslaTrafficControl=traffic),
    "carState": ns(vEgo=15.0, aEgo=0.0, gasPressed=False, brakePressed=False),
    "carControl": ns(enabled=True, longActive=True, leftBlinker=False, rightBlinker=False),
    "radarState": ns(leadOne=ns(present=False, dRel=0.0), leadTwo=ns(present=False, dRel=0.0)),
    "modelV2": ns(position=ns(x=[200.0] * 33), velocity=ns(x=[15.0] * 33)),
  }

  class FakeSubMaster:
    seen = {"carStateSP": True, "radarState": True, "modelV2": True}
    alive = {"carStateSP": True, "radarState": True, "modelV2": True}
    valid = {"carStateSP": True, "radarState": True, "modelV2": True}

    def __getitem__(self, key):
      return messages[key]

  return FakeSubMaster()


def test_stale_observation_is_rejected_at_planner_boundary():
  msg = fake_sm()["carStateSP"].teslaTrafficControl
  msg.frameMonoTime = 1_000_000_000
  assert not TrafficControlPlannerAdapter._observation(msg, 1_000_000_000 + 250_000_001).available


def test_future_observation_is_rejected_at_planner_boundary():
  msg = fake_sm()["carStateSP"].teslaTrafficControl
  msg.frameMonoTime = 1_000_000_001
  assert not TrafficControlPlannerAdapter._observation(msg, 1_000_000_000).available


def test_mode_off_is_output_identical_to_unwrapped_backend():
  planner = FakePlanner()
  before = deepcopy((planner.v_desired_trajectory, planner.a_desired_trajectory,
                     planner.j_desired_trajectory, planner.output_a_target,
                     planner.output_should_stop, planner.allow_throttle))
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=0),
  )
  adapter.update(fake_sm())
  after = (planner.v_desired_trajectory, planner.a_desired_trajectory,
           planner.j_desired_trajectory, planner.output_a_target,
           planner.output_should_stop, planner.allow_throttle)

  assert planner.update_calls == 1
  for expected, actual in zip(before[:3], after[:3], strict=True):
    assert np.array_equal(expected, actual)
  assert before[3:] == after[3:]


def test_more_restrictive_backend_command_is_never_replaced():
  planner = FakePlanner()
  planner.v_desired_trajectory = np.zeros(17)
  planner.a_desired_trajectory = np.full(17, -2.0)
  planner.output_a_target = -2.0
  before = deepcopy((planner.v_desired_trajectory, planner.a_desired_trajectory,
                     planner.j_desired_trajectory, planner.output_a_target))
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=3),
  )
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.braking, active=True,
    apply_constraint=True, shadow=False, should_stop=True, remaining_distance=70.0,
    stop_reference=6.0, light_state=1, source_bus=2, quality=2,
  )
  applied, _ = adapter._apply_constraint(decision, fake_sm())

  assert not applied
  assert np.array_equal(before[0], planner.v_desired_trajectory)
  assert np.array_equal(before[1], planner.a_desired_trajectory)
  assert np.array_equal(before[2], planner.j_desired_trajectory)
  assert planner.output_a_target == before[3]


def test_mode_is_latched_once_at_planner_initialization():
  params = FakeParams(mode=TrafficControlMode.shadow)
  adapter = TrafficControlPlannerAdapter(
    FakePlanner(), ns(longitudinalActuatorDelay=0.2), params,
  )
  assert adapter._controller.config.mode == TrafficControlMode.shadow

  params.mode = TrafficControlMode.stopGo
  adapter.update(fake_sm())
  assert adapter._controller.config.mode == TrafficControlMode.shadow


@pytest.mark.parametrize("mode", list(TrafficControlMode))
def test_all_monitoring_stages_share_one_decision_adapter(mode):
  params = FakeParams(mode=mode)
  base = FakePlanner()
  planner = decorate_planner(base, ns(brand="tesla", longitudinalActuatorDelay=0.2), params)

  if mode == TrafficControlMode.off:
    assert planner is base
  else:
    assert isinstance(planner, TrafficControlPlannerAdapter)
    assert planner._planner is base


def test_non_tesla_never_installs_traffic_control_adapter():
  base = FakePlanner()
  planner = decorate_planner(
    base, ns(brand="toyota", longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  assert planner is base


def test_stop_constraint_never_raises_any_backend_trajectory_value():
  planner = FakePlanner()
  base_speeds = planner.v_desired_trajectory.copy()
  base_accels = planner.a_desired_trajectory.copy()
  base_output_accel = planner.output_a_target
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.braking, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=8.0,
    stop_reference=6.0, light_state=1, source_bus=2, quality=2,
  )

  applied, constraint_accel = adapter._apply_constraint(decision, fake_sm())

  assert applied
  assert np.all(planner.v_desired_trajectory <= base_speeds + 1e-9)
  assert np.all(planner.a_desired_trajectory <= base_accels + 1e-9)
  assert planner.output_a_target <= base_output_accel
  assert constraint_accel == planner.output_a_target
  assert not planner.output_should_stop
  assert planner.allow_throttle


def test_green_release_only_removes_constraint_and_never_builds_acceleration():
  planner = FakePlanner()
  before = deepcopy((planner.v_desired_trajectory, planner.a_desired_trajectory,
                     planner.j_desired_trajectory, planner.output_a_target,
                     planner.output_should_stop, planner.allow_throttle))
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.release, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=0.0,
    stop_reference=6.0, light_state=2, source_bus=2, quality=2,
  )

  applied, constraint_accel = adapter._apply_constraint(decision, fake_sm())

  assert not applied
  assert constraint_accel == 0.0
  after = (planner.v_desired_trajectory, planner.a_desired_trajectory,
           planner.j_desired_trajectory, planner.output_a_target,
           planner.output_should_stop, planner.allow_throttle)
  for expected, actual in zip(before[:3], after[:3], strict=True):
    assert np.array_equal(expected, actual)
  assert before[3:] == after[3:]


def test_stopped_explicit_green_uses_bounded_cp_cruise_departure():
  planner = FakePlanner()
  planner.v_desired_trajectory = np.linspace(0.0, 4.0, 17)
  planner.a_desired_trajectory = np.full(17, 0.4)
  planner.output_a_target = -0.2
  planner.output_should_stop = True
  planner.a_desired = -0.2
  planner.a_cruise = 0.25
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  sm = fake_sm()
  sm["carState"].vEgo = 0.0
  before = deepcopy((planner.v_desired_trajectory, planner.a_desired_trajectory,
                     planner.j_desired_trajectory, planner.output_a_target,
                     planner.output_should_stop, planner.allow_throttle,
                     planner.a_desired, planner.v_desired_filter.x))
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.release, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=0.0,
    stop_reference=6.0, light_state=2, source_bus=2, quality=2,
  )

  applied, constraint_accel = adapter._apply_constraint(decision, sm)

  assert applied
  assert constraint_accel == 0.25
  after = (planner.v_desired_trajectory, planner.a_desired_trajectory,
           planner.j_desired_trajectory, planner.output_a_target,
           planner.output_should_stop, planner.allow_throttle,
           planner.a_desired, planner.v_desired_filter.x)
  for expected, actual in zip(before[:3], after[:3], strict=True):
    assert np.array_equal(expected, actual)
  assert planner.output_a_target == 0.25
  assert not planner.output_should_stop
  assert planner.allow_throttle
  # Do not feed the local release into the backend's persistent state.
  assert planner.a_desired == before[6]
  assert planner.v_desired_filter.x == before[7]


def test_stopped_green_release_does_not_override_lead_or_base_stop():
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.release, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=0.0,
    stop_reference=6.0, light_state=2, source_bus=2, quality=2,
  )

  for lead_present, departure_accel in ((True, 0.4), (False, 0.0)):
    planner = FakePlanner()
    planner.v_desired_trajectory = np.linspace(0.0, departure_accel * 4.0, 17)
    planner.a_desired_trajectory = np.full(17, departure_accel)
    planner.output_should_stop = True
    adapter = TrafficControlPlannerAdapter(
      planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
    )
    sm = fake_sm()
    sm["carState"].vEgo = 0.0
    sm["radarState"].leadTwo.present = lead_present
    before = (planner.output_a_target, planner.output_should_stop, planner.a_desired)

    applied, _ = adapter._apply_constraint(decision, sm)

    assert not applied
    assert (planner.output_a_target, planner.output_should_stop, planner.a_desired) == before


def test_stop_constraint_never_mutates_backend_persistent_state():
  planner = FakePlanner()
  planner.a_desired = 0.35
  planner.v_desired_filter.x = 12.5
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.braking, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=8.0,
    stop_reference=6.0, light_state=1, source_bus=2, quality=2,
  )

  applied, _ = adapter._apply_constraint(decision, fake_sm())

  assert applied
  assert planner.a_desired == 0.35
  assert planner.v_desired_filter.x == 12.5


@pytest.mark.parametrize("publish_between", [False, True])
def test_constrained_output_is_restored_before_next_backend_cycle(publish_between):
  class FeedbackPlanner(FakePlanner):
    def __init__(self):
      super().__init__()
      self.entry_accels = []

    def update(self, sm):
      del sm
      self.entry_accels.append(self.output_a_target)
      self.v_desired_trajectory = np.linspace(10.0, 8.0, 17)
      self.a_desired_trajectory = np.linspace(0.3, 0.1, 17)
      self.j_desired_trajectory = np.zeros(17)
      self.output_a_target = 0.25
      self.output_should_stop = False
      self.allow_throttle = True

  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.braking, active=True,
    apply_constraint=True, shadow=False, should_stop=False, remaining_distance=8.0,
    stop_reference=6.0, light_state=1, source_bus=2, quality=2,
  )
  planner = FeedbackPlanner()
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  adapter._controller.update = lambda *args, **kwargs: decision
  sm = fake_sm()

  adapter.update(sm)
  assert planner.output_a_target < 0.25
  if publish_between:
    adapter.publish(sm, None)
    assert planner.output_a_target == 0.25

  adapter.update(sm)
  assert planner.entry_accels == [-0.2, 0.25]


def test_dead_radar_and_invalid_model_fail_closed_before_controller():
  adapter = TrafficControlPlannerAdapter(
    FakePlanner(), ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  sm = fake_sm()
  sm.alive["radarState"] = False
  sm.valid["modelV2"] = False
  captured = {}
  decision = TrafficControlDecision(
    mode=TrafficControlMode.stopGo, phase=TrafficControlPhase.off, active=False,
    apply_constraint=False, shadow=False, should_stop=False, remaining_distance=0.0,
    stop_reference=6.0, light_state=1, source_bus=2, quality=2,
  )

  def capture_update(*args, **kwargs):
    captured.update(kwargs)
    return decision

  adapter._controller.update = capture_update
  adapter.update(sm)

  assert captured["radar_valid"] is False
  assert captured["lead_present"] is False
  assert captured["model_stop_distance"] is None
  assert captured["model_stop_candidate"] is False


def test_transition_logging_records_candidate_replacement_without_per_frame_spam(monkeypatch):
  events = []
  update_times = (0, int(0.4e9), int(0.6e9))
  times = iter(update_times)
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: next(times))
  monkeypatch.setattr(
    planner_adapter_module.cloudlog, "event",
    lambda name, **kwargs: events.append((name, kwargs)),
  )
  adapter = TrafficControlPlannerAdapter(
    FakePlanner(), ns(longitudinalActuatorDelay=0.2), FakeParams(mode=TrafficControlMode.stopGo),
  )
  sm = fake_sm()

  for now_ns, distance in zip(update_times, (150.0, 146.0, 30.0), strict=True):
    sm["carStateSP"].teslaTrafficControl.frameMonoTime = now_ns
    sm["carStateSP"].teslaTrafficControl.distance = distance
    adapter.update(sm)

  assert [event[1]["transition"] for event in events] == ["candidate_started", "candidate_replaced"]
  assert events[-1][1]["observationDistance"] == 30.0
  assert events[-1][1]["distanceInnovation"] < -100.0
