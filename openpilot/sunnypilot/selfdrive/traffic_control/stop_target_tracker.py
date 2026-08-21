from __future__ import annotations

from collections import deque
import math

import numpy as np


class StopTargetTracker:
  """CP-style filtering and close-rate limiting for the model stop target."""

  def __init__(self, *, median_window: int = 3, average_window: int = 15,
               close_margin_m: float = 0.5) -> None:
    self._raw = deque(maxlen=median_window)
    self._median = deque(maxlen=average_window)
    self._close_margin_m = close_margin_m
    self._last_update_ns: int | None = None
    self._filtered_distance: float | None = None

  @property
  def filtered_distance(self) -> float | None:
    return self._filtered_distance

  def reset(self) -> None:
    self._raw.clear()
    self._median.clear()
    self._last_update_ns = None
    self._filtered_distance = None

  def update_model(self, distance: float | None, *, v_ego: float, now_ns: int) -> float | None:
    dt = (0.0 if self._last_update_ns is None else
          max(0.0, min((now_ns - self._last_update_ns) / 1e9, 0.5)))
    self._last_update_ns = now_ns

    if distance is None or not math.isfinite(distance) or distance < 0.0:
      return self._filtered_distance

    self._raw.append(float(distance))
    self._median.append(float(np.median(self._raw)))
    smoothed = float(np.mean(self._median))

    if self._filtered_distance is None or smoothed >= self._filtered_distance:
      self._filtered_distance = smoothed
    else:
      max_close = max(float(v_ego), 0.0) * dt + self._close_margin_m
      self._filtered_distance = max(self._filtered_distance - max_close, smoothed)
    return self._filtered_distance
