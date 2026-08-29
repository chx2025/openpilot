"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Pure, stateless helper functions for the traffic-light / stop-sign virtual
stop-line obstacle. Ported from carrot openpilot fork's
`selfdrive/carrot/traffic_stop.py`.

These functions carry no internal state on purpose: all smoothing / rate
limiting / debouncing lives in TrafficStopController (traffic_stop_controller.py).
See docs/traffic_stop.md (or the porting spec) for the full derivation of the
constants below.
"""
from numpy import interp

# steering angle above this magnitude suppresses *entering* a new traffic-stop
# management cycle (assumed to be an intentional turn, not a stop)
TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0

# at 0 kph the virtual stop line is placed at 100% of the model-predicted
# distance; at 100 kph it is pulled in to 70% (stop earlier at speed)
TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH = (0.0, 100.0)
TRAFFIC_STOP_DISTANCE_RATIO = (1.0, 0.7)

# within the last 50m before the (adjusted) stop line, fade the ratio back to
# 1.0 so the car actually stops at the model's predicted line, not short of it
TRAFFIC_STOP_DISTANCE_FADE_BP_M = (0.0, 50.0)


def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """Large steering angle -> likely an intentional turn, not a stop. Blocks new entries only."""
  return abs(steering_angle_deg) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_reference_speed(v_ego_kph: float, previous_reference_kph: float | None) -> float:
  """Latch the highest speed seen so far during this stop approach (used by the v_cruise soft-limit)."""
  return max(0.0, v_ego_kph, previous_reference_kph or 0.0)


def get_virtual_traffic_stop_distance(model_distance: float, v_ego_kph: float) -> float:
  """Pull the stop line closer at speed (brake earlier), fading back to 100% within the last 50m.

  This only changes *when* deceleration begins, not *where* the car ends up stopping.
  """
  distance_ratio = interp(v_ego_kph, TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH, TRAFFIC_STOP_DISTANCE_RATIO)
  applied_ratio = interp(model_distance, TRAFFIC_STOP_DISTANCE_FADE_BP_M, [1.0, distance_ratio])
  return max(0.0, model_distance * applied_ratio)


def get_traffic_stop_obstacle_distance(stop_distance: float, distance_adjust: float) -> float:
  """Apply a fixed/UI-adjustable offset to the stop distance, clamped to 0."""
  return max(0.0, stop_distance + distance_adjust)
