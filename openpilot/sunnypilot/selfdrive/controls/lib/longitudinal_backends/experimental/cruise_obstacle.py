import numpy as np


CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6


def build_cruise_obstacle(v_ego: float, v_cruise: float, time_idxs: np.ndarray, *,
                          t_follow: float, comfort_brake: float, stop_distance: float) -> np.ndarray:
  """Convert a cruise target into the third obstacle used by the Experimental MPC."""
  time_idxs = np.asarray(time_idxs, dtype=float)
  time_diffs = np.diff(time_idxs, prepend=time_idxs[0])
  v_lower = v_ego + time_idxs * CRUISE_MIN_ACCEL * 1.05
  v_upper = v_ego + time_idxs * CRUISE_MAX_ACCEL * 1.05
  v_target = np.clip(np.full_like(time_idxs, v_cruise), v_lower, v_upper)
  safe_distance = v_target ** 2 / (2.0 * comfort_brake) + t_follow * v_target + stop_distance
  return np.cumsum(time_diffs * v_target) + safe_distance
