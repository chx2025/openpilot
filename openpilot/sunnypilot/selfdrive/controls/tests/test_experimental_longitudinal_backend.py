import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.cruise_obstacle import build_cruise_obstacle


def test_cruise_obstacle_tracks_set_speed_with_bounded_acceleration():
  time_idxs = np.array([0.0, 0.4, 1.0, 2.0, 4.0])

  slower = build_cruise_obstacle(10.0, 8.0, time_idxs, t_follow=1.45, comfort_brake=2.5, stop_distance=6.0)
  faster = build_cruise_obstacle(10.0, 20.0, time_idxs, t_follow=1.45, comfort_brake=2.5, stop_distance=6.0)

  assert np.all(np.isfinite(slower))
  assert np.all(np.diff(slower) > 0.0)
  assert np.all(np.diff(faster) > 0.0)
  assert faster[-1] > slower[-1]
  assert faster[0] == slower[0]
