import json
from pathlib import Path

import numpy as np

from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name
from openpilot.tools.lib.logreader import LogReader


FIXTURE_DIR = Path(__file__).parent / "fixtures"
ROUTE_FIXTURE = FIXTURE_DIR / "tesla_legacy_planner_warm.rlog.zst"


def _replay(backend: int):
  return replay_process_with_name(
    "plannerd", list(LogReader(str(ROUTE_FIXTURE))), fingerprint="TESLA_MODEL_Y",
    custom_params={
      "LongitudinalPlannerMode": backend,
      "MpcTuningProfile": 0,
      "TeslaTrafficControlMode": 0,
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
  # This route includes the legacy primary solver's one inactive status-4
  # reset. Recovery is deliberately gated by carControl.longActive, so every
  # decision field and numeric output remains the recorded rs408 value here.
  _assert_matches_legacy(_replay(1), FIXTURE_DIR / "tesla_legacy_experimental.json")


def test_tn_matches_the_comfortable_legacy_route_with_traffic_off():
  # Only an active primary failure may use the numerically robust solver; the
  # inactive reference route must remain exact within solver float tolerance.
  _assert_matches_legacy(_replay(2), FIXTURE_DIR / "tesla_legacy_tn.json")
