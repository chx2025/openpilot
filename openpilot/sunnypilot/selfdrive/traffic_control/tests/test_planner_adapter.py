from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.traffic_control import decorate_planner
from openpilot.sunnypilot.selfdrive.traffic_control import planner_adapter as planner_adapter_module
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlMode, TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.planner_adapter import TrafficControlPlannerAdapter


def ns(**kwargs):
  return SimpleNamespace(**kwargs)


class FakeParams:
  def __init__(self, mode=0, strategy=0):
    self.values = {
      "TeslaTrafficControlMode": mode,
      "TeslaTrafficControlStrategy": strategy,
    }

  def get(self, key, *, return_default=False):
    assert return_default
    return self.values[key]


class FakeMpc:
  def __init__(self):
    self.target = None
    self.source = log.LongitudinalPlan.LongitudinalPlanSource.cruise

  def set_traffic_target(self, target):
    self.target = target


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
    self.mpc = FakeMpc()
    self.update_calls = 0
    self.received_targets = []

  def update(self, sm):
    del sm
    self.update_calls += 1
    self.received_targets.append(self.mpc.target)
    self.mpc.source = (
      log.LongitudinalPlan.LongitudinalPlanSource.lead2
      if self.mpc.target is not None
      else log.LongitudinalPlan.LongitudinalPlanSource.cruise
    )

  def publish(self, sm, pm):
    del sm, pm


def fake_sm():
  messages = {
    "carState": ns(vEgo=0.0, aEgo=0.0, vCruise=50.0, gasPressed=False, brakePressed=False),
    "carControl": ns(enabled=True, longActive=True, leftBlinker=False, rightBlinker=False),
    "radarState": ns(leadOne=ns(present=False, dRel=0.0), leadTwo=ns(present=False, dRel=0.0)),
    "trafficRadarState": ns(
      targetPresent=False,
      oemTargetDistance=0.0,
      targetRelativeVelocity=0.0,
      targetRelativeAcceleration=0.0,
      distanceToStopPoint=0.0,
      phase=0,
      lightState=0,
      sourceBus=0,
      quality=0,
      confidence=0.0,
      eventId=0,
      publishMonoTime=0,
      controlAllowed=False,
      suppressedByPhysicalLead=False,
      shouldStop=False,
      plannerStartRequested=False,
      mode=0,
    ),
  }

  class FakeSubMaster:
    seen = {"radarState": True, "trafficRadarState": True}
    alive = {"radarState": True, "trafficRadarState": True}
    valid = {"radarState": True, "trafficRadarState": True}

    def __getitem__(self, key):
      return messages[key]

  return FakeSubMaster()


def set_red_target(sm, now_ns: int, *, distance=30.0, phase=TrafficControlPhase.braking, event_id=7):
  target = sm["trafficRadarState"]
  target.targetPresent = True
  target.oemTargetDistance = distance + 6.0
  target.distanceToStopPoint = distance
  target.phase = int(phase)
  target.lightState = 1
  target.eventId = event_id
  target.publishMonoTime = now_ns
  target.controlAllowed = True
  target.shouldStop = phase == TrafficControlPhase.hold
  target.mode = int(TrafficControlMode.stopGo)
  return target


def test_traffic_radar_reaches_mpc_without_replacing_physical_leads(monkeypatch):
  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  planner = FakePlanner()
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  lead_one = sm["radarState"].leadOne
  lead_two = sm["radarState"].leadTwo
  set_red_target(sm, now_ns)

  adapter.update(sm)

  assert planner.received_targets[0].distance_to_stop_point == 30.0
  assert planner.mpc.target is None
  assert sm["radarState"].leadOne is lead_one and sm["radarState"].leadTwo is lead_two
  assert not lead_one.present and not lead_two.present


def test_real_lead_suppresses_traffic_again_at_the_planner_boundary(monkeypatch):
  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  planner = FakePlanner()
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  set_red_target(sm, now_ns)
  sm["radarState"].leadOne.present = True
  sm["radarState"].leadOne.dRel = 18.0

  adapter.update(sm)

  assert planner.received_targets == [None]
  assert sm["radarState"].leadOne.present and sm["radarState"].leadOne.dRel == 18.0


@pytest.mark.parametrize("age_ns", [250_000_001, -1])
def test_stale_or_future_traffic_radar_is_rejected(monkeypatch, age_ns):
  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  planner = FakePlanner()
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  set_red_target(sm, now_ns - age_ns)

  adapter.update(sm)

  assert planner.received_targets == [None]


def test_confirmed_hold_survives_a_short_publisher_loss(monkeypatch):
  times = iter((1_000_000_000, 2_000_000_000))
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: next(times))
  planner = FakePlanner()
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  set_red_target(sm, 1_000_000_000, distance=0.0, phase=TrafficControlPhase.hold)

  adapter.update(sm)
  sm.alive["trafficRadarState"] = False
  adapter.update(sm)

  assert planner.received_targets[0] is not None and planner.received_targets[1] is not None
  assert planner.output_should_stop
  assert not planner.allow_throttle


def test_passive_release_only_removes_the_target(monkeypatch):
  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  planner = FakePlanner()
  planner.output_should_stop = True
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  target = set_red_target(sm, now_ns, phase=TrafficControlPhase.release)
  target.targetPresent = False
  target.controlAllowed = True

  adapter.update(sm)

  assert planner.received_targets == [None]
  assert planner.output_a_target == -0.2
  assert planner.output_should_stop


def test_planner_start_is_bounded_deduplicated_and_never_mutates_car_control(monkeypatch):
  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  planner = FakePlanner()
  planner.output_should_stop = True
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=1),
  )
  sm = fake_sm()
  original_car_control = deepcopy(sm["carControl"])
  target = set_red_target(sm, now_ns, phase=TrafficControlPhase.release, event_id=9)
  target.targetPresent = False
  target.controlAllowed = True
  target.lightState = 2
  target.plannerStartRequested = True

  adapter.update(sm)
  first_accel = planner.output_a_target
  adapter.publish(sm, None)
  adapter.update(sm)

  assert 0.0 < first_accel <= 0.4
  assert planner.output_a_target == -0.2
  assert sm["carControl"] == original_car_control


@pytest.mark.parametrize(
  "mode", (TrafficControlMode.off, TrafficControlMode.observe, TrafficControlMode.shadow),
)
def test_off_and_monitoring_modes_match_the_unwrapped_planner_output(monkeypatch, mode):
  class DeterministicPlanner(FakePlanner):
    def update(self, sm):
      super().update(sm)
      self.v_desired_trajectory = np.linspace(12.0, 9.0, 17)
      self.a_desired_trajectory = np.linspace(0.3, -0.4, 17)
      self.j_desired_trajectory = np.linspace(-0.1, 0.1, 17)
      self.output_a_target = 0.15
      self.output_should_stop = True
      self.allow_throttle = False
      self.a_desired = 0.12
      self.v_desired_filter.x = 10.25

  now_ns = 1_000_000_000
  monkeypatch.setattr(planner_adapter_module.time, "monotonic_ns", lambda: now_ns)
  expected = DeterministicPlanner()
  expected.update(fake_sm())
  expected.publish(fake_sm(), None)
  planner = decorate_planner(
    DeterministicPlanner(), ns(brand="tesla", longitudinalActuatorDelay=0.2),
    FakeParams(mode=mode, strategy=1),
  )
  sm = fake_sm()
  target = set_red_target(sm, now_ns)
  target.mode = int(mode)
  target.controlAllowed = False

  planner.update(sm)
  planner.publish(sm, None)
  expected_output = (
    expected.v_desired_trajectory, expected.a_desired_trajectory, expected.j_desired_trajectory,
    expected.output_a_target, expected.output_should_stop, expected.allow_throttle,
    expected.a_desired, expected.v_desired_filter.x,
  )
  actual_output = (
    planner.v_desired_trajectory, planner.a_desired_trajectory, planner.j_desired_trajectory,
    planner.output_a_target, planner.output_should_stop, planner.allow_throttle,
    planner.a_desired, planner.v_desired_filter.x,
  )

  for expected_value, actual_value in zip(expected_output[:3], actual_output[:3], strict=True):
    assert np.array_equal(expected_value, actual_value)
  assert expected_output[3:] == actual_output[3:]


def test_stop_profile_never_raises_a_backend_trajectory():
  planner = FakePlanner()
  base_speeds = planner.v_desired_trajectory.copy()
  base_accels = planner.a_desired_trajectory.copy()
  base_output = planner.output_a_target
  adapter = TrafficControlPlannerAdapter(
    planner, ns(longitudinalActuatorDelay=0.2),
    FakeParams(mode=TrafficControlMode.stopGo, strategy=0),
  )
  sm = fake_sm()
  decision = planner_adapter_module.TrafficControlDecision(
    mode=TrafficControlMode.stopGo,
    phase=TrafficControlPhase.braking,
    active=True,
    apply_constraint=True,
    shadow=False,
    should_stop=False,
    remaining_distance=8.0,
    stop_reference=6.0,
    light_state=1,
    source_bus=2,
    quality=2,
  )

  applied, _ = adapter._apply_stop_profile(decision, sm)

  assert applied
  assert np.all(planner.v_desired_trajectory <= base_speeds)
  assert np.all(planner.a_desired_trajectory <= base_accels)
  assert planner.output_a_target <= base_output


@pytest.mark.parametrize("mode", list(TrafficControlMode))
def test_adapter_is_installed_for_tesla_only_when_traffic_is_not_off(mode):
  params = FakeParams(mode=mode)
  base = FakePlanner()
  planner = decorate_planner(base, ns(brand="tesla", longitudinalActuatorDelay=0.2), params)
  assert (planner is base) if mode == TrafficControlMode.off else isinstance(planner, TrafficControlPlannerAdapter)

  non_tesla = FakePlanner()
  assert decorate_planner(
    non_tesla, ns(brand="toyota", longitudinalActuatorDelay=0.2), params,
  ) is non_tesla
