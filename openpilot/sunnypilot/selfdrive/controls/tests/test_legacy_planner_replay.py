import hashlib
import json
import platform
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name
from openpilot.tools.lib.logreader import LogReader


FIXTURE_DIR = Path(__file__).parent / "fixtures"
ROUTE_FIXTURE = FIXTURE_DIR / "tesla_legacy_planner_warm.rlog.zst"
ROUTE_FIXTURE_SHA256 = "36e4a6e774d33839b2b9f78d9f90d14b132020fb1cb88511d1096c737b0747f4"
LINUX_AARCH64_GOLDEN_SHA256 = "30e5da2925023786fe21199c74bac81caf924791b81f93b5a2bf3fccd6c71c50"


def _legacy_golden(backend: str) -> Path:
  target = (platform.system(), platform.machine().lower())
  if target == ("Darwin", "arm64"):
    return FIXTURE_DIR / f"tesla_legacy_{backend}.json"
  if target == ("Linux", "aarch64"):
    golden = FIXTURE_DIR / "tesla_legacy_linux_aarch64.json"
    assert hashlib.sha256(golden.read_bytes()).hexdigest() == LINUX_AARCH64_GOLDEN_SHA256
    return golden
  pytest.skip(f"no old-tree numerical golden recorded for {target[0]}/{target[1]}")


def test_minimized_route_fixture_is_present_and_intact_in_a_clean_checkout():
  assert ROUTE_FIXTURE.is_file(), "the minimized replay route must be tracked with this test"
  assert hashlib.sha256(ROUTE_FIXTURE.read_bytes()).hexdigest() == ROUTE_FIXTURE_SHA256


def _replay(backend: int):
  return replay_process_with_name(
    "plannerd", list(LogReader(str(ROUTE_FIXTURE))), fingerprint="TESLA_MODEL_Y",
    custom_params={
      "LongitudinalPlannerMode": backend,
      "MpcTuningProfile": 0,
      "DynamicExperimentalControl": False,
      "SmartCruiseControlVision": False,
      "SmartCruiseControlMap": False,
      "SpeedLimitMode": 0,
    },
    disable_progress=True,
  )


def _plan_rows(messages):
  return [message for message in messages if message.which() == "longitudinalPlan"]


def _assert_matches_legacy(actual_messages, expected_path: Path) -> None:
  actual = _plan_rows(actual_messages)
  expected = json.loads(expected_path.read_text())
  assert len(actual) == len(expected)

  for message, reference in zip(actual, expected, strict=True):
    plan = message.longitudinalPlan
    assert int(message.logMonoTime) == reference["logMonoTime"]
    assert str(plan.longitudinalPlanSource) == reference["source"]
    assert bool(plan.shouldStop) is reference["shouldStop"]
    assert bool(plan.allowThrottle) is reference["allowThrottle"]
    assert np.isclose(float(plan.aTarget), reference["aTarget"], atol=1e-5)
    assert np.allclose(plan.speeds, reference["speeds"], atol=1e-5)
    assert np.allclose(plan.accels, reference["accels"], atol=1e-5)
    assert np.allclose(plan.jerks, reference["jerks"], atol=1e-5)


def test_experimental_matches_the_comfortable_legacy_route_with_traffic_off():
  _assert_matches_legacy(_replay(1), _legacy_golden("experimental"))


def test_tn_matches_the_comfortable_legacy_route_with_traffic_off():
  _assert_matches_legacy(_replay(2), _legacy_golden("tn"))
