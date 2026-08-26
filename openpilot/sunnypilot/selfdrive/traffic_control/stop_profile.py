from __future__ import annotations

import numpy as np


class StopProfileGenerator:
  def __init__(self, comfort_brake: float = 2.4, jerk_limit: float = 0.8,
               release_jerk_limit: float = 0.5, actuator_delay: float = 0.2,
               planner_dt: float = 0.05) -> None:
    self.comfort_brake = comfort_brake
    self.jerk_limit = jerk_limit
    self.release_jerk_limit = release_jerk_limit
    self.actuator_delay = actuator_delay
    self.planner_dt = planner_dt
    self.previous_accel: float | None = None

  def reset(self) -> None:
    self.previous_accel = None

  @staticmethod
  def required_stop_distance(*, v_ego: float, a_ego: float,
                             actuator_delay: float, max_brake: float,
                             jerk_limit: float, dt: float = 0.05) -> float:
    """Numerically predict the distance of the same delayed, jerk-limited stop."""
    velocity = max(float(v_ego), 0.0)
    acceleration = float(np.clip(a_ego, -max_brake, max_brake))
    distance = 0.0
    delay_remaining = max(float(actuator_delay), 0.0)
    step = max(float(dt), 1e-3)

    while velocity > 0.0 and delay_remaining > 1e-9:
      cycle = min(step, delay_remaining)
      next_velocity = max(0.0, velocity + acceleration * cycle)
      distance += max(0.0, (velocity + next_velocity) * 0.5 * cycle)
      velocity = next_velocity
      delay_remaining -= cycle

    for _ in range(60_000):
      if velocity <= 0.0:
        break
      acceleration += float(np.clip(
        -max_brake - acceleration,
        -max(jerk_limit, 0.1) * step,
        max(jerk_limit, 0.1) * step,
      ))
      acceleration = float(np.clip(acceleration, -max_brake, max_brake))
      next_velocity = max(0.0, velocity + acceleration * step)
      distance += max(0.0, (velocity + next_velocity) * 0.5 * step)
      velocity = next_velocity
    return distance

  @staticmethod
  def _dt(times: np.ndarray, index: int) -> float:
    return max(float(times[index] - times[index - 1]), 1e-3)

  def _next_cycle_accel(self, times: np.ndarray, accels: np.ndarray) -> float:
    return float(np.interp(self.planner_dt, times, accels))

  def build_stop(self, *, v_ego: float, a_ego: float, remaining_distance: float,
                 times: np.ndarray, hold: bool = False,
                 comfort_brake: float | None = None, jerk_limit: float | None = None,
                 decel_margin: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    speeds = np.zeros(len(times), dtype=float)
    accels = np.zeros(len(times), dtype=float)
    jerks = np.zeros(max(len(times) - 1, 0), dtype=float)
    if len(times) == 0:
      return speeds, accels, jerks

    speeds[0] = max(0.0, v_ego)
    if v_ego <= 0.01:
      self.previous_accel = 0.0
      return speeds, accels, jerks

    stop_brake = self.comfort_brake if comfort_brake is None else max(float(comfort_brake), 0.1)
    stop_jerk = self.jerk_limit if jerk_limit is None else max(float(jerk_limit), 0.1)
    stop_margin = max(float(decel_margin), 1.0)
    current_a = float(np.clip(a_ego if self.previous_accel is None else self.previous_accel,
                              -stop_brake, 0.0))
    accels[0] = current_a
    remaining = 0.0 if hold else max(remaining_distance, 0.0)
    for i in range(1, len(times)):
      dt = self._dt(times, i)
      effective_distance = max(remaining - speeds[i - 1] * self.actuator_delay, 0.5)
      target_a = -stop_margin * speeds[i - 1] ** 2 / (2.0 * effective_distance)
      target_a = float(np.clip(target_a, -stop_brake, 0.0))
      delta_a = float(np.clip(target_a - current_a, -stop_jerk * dt, stop_jerk * dt))
      current_a += delta_a
      next_v = max(0.0, speeds[i - 1] + current_a * dt)
      travelled = max(0.0, (speeds[i - 1] + next_v) * 0.5 * dt)
      remaining = max(0.0, remaining - travelled)
      speeds[i] = next_v
      accels[i] = current_a if next_v > 0.0 else 0.0
      jerks[i - 1] = delta_a / dt

    self.previous_accel = self._next_cycle_accel(times, accels)
    return speeds, accels, jerks

  def build_release(self, *, v_ego: float, base_accel: float,
                    times: np.ndarray, preserve_positive_accel: bool = False,
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    speeds = np.zeros(len(times), dtype=float)
    accels = np.zeros(len(times), dtype=float)
    jerks = np.zeros(max(len(times) - 1, 0), dtype=float)
    if len(times) == 0:
      return speeds, accels, jerks

    speeds[0] = max(0.0, v_ego)
    current_a = self.previous_accel or 0.0
    if not preserve_positive_accel:
      current_a = min(0.0, current_a)
    accels[0] = current_a
    for i in range(1, len(times)):
      dt = self._dt(times, i)
      target_a = max(current_a, base_accel)
      delta_a = float(np.clip(target_a - current_a,
                              -self.release_jerk_limit * dt, self.release_jerk_limit * dt))
      current_a += delta_a
      speeds[i] = max(0.0, speeds[i - 1] + current_a * dt)
      accels[i] = current_a
      jerks[i - 1] = delta_a / dt
    self.previous_accel = self._next_cycle_accel(times, accels)
    return speeds, accels, jerks
