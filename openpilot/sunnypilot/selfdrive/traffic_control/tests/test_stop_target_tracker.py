import pytest

from openpilot.sunnypilot.selfdrive.traffic_control.stop_target_tracker import StopTargetTracker


def test_model_target_closes_at_cp_rate_but_can_move_farther_immediately():
  tracker = StopTargetTracker(median_window=1, average_window=1)

  assert tracker.update_model(60.0, v_ego=10.0, now_ns=0) == pytest.approx(60.0)
  assert tracker.update_model(20.0, v_ego=10.0, now_ns=50_000_000) == pytest.approx(59.0)
  assert tracker.update_model(80.0, v_ego=10.0, now_ns=100_000_000) == pytest.approx(80.0)


def test_invalid_model_target_does_not_replace_the_last_filtered_target():
  tracker = StopTargetTracker(median_window=1, average_window=1)
  tracker.update_model(40.0, v_ego=5.0, now_ns=0)

  assert tracker.update_model(None, v_ego=5.0, now_ns=50_000_000) == pytest.approx(40.0)
