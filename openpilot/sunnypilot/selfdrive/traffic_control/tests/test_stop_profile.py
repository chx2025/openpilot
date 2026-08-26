import numpy as np

from openpilot.sunnypilot.selfdrive.traffic_control.stop_profile import StopProfileGenerator


TIMES = np.array([0.0, 0.05, 0.15, 0.30, 0.50, 0.80, 1.20, 1.70, 2.30, 3.00])


def test_profile_starts_gentle_deceleration_for_red_at_100m():
  profile = StopProfileGenerator(comfort_brake=2.4, jerk_limit=0.8)
  speeds, accels, jerks = profile.build_stop(v_ego=16.7, a_ego=0.0, remaining_distance=94.0, times=TIMES)
  assert speeds[-1] < speeds[0]
  assert accels[1] < 0.0
  assert np.min(accels) >= -2.4
  assert np.max(np.abs(jerks)) <= 0.8 + 1e-6


def test_profile_holds_zero_speed_without_negative_velocity():
  profile = StopProfileGenerator()
  speeds, accels, _ = profile.build_stop(v_ego=0.0, a_ego=0.0, remaining_distance=0.0, times=TIMES, hold=True)
  assert np.all(speeds == 0.0)
  assert np.all(accels == 0.0)


def test_release_profile_limits_initial_acceleration_and_ramps():
  profile = StopProfileGenerator(release_jerk_limit=0.5)
  profile.previous_accel = -0.2
  speeds, accels, jerks = profile.build_release(v_ego=0.0, base_accel=1.0, times=TIMES)
  assert accels[1] < 0.0
  assert accels[-1] > accels[1]
  assert np.max(jerks) <= 0.5 + 1e-6


def test_required_stop_distance_uses_delay_jerk_and_brake_consistently():
  gentle = StopProfileGenerator.required_stop_distance(
    v_ego=15.0, a_ego=0.0, actuator_delay=0.2, max_brake=2.2, jerk_limit=0.55,
  )
  standard = StopProfileGenerator.required_stop_distance(
    v_ego=15.0, a_ego=0.0, actuator_delay=0.2, max_brake=2.5, jerk_limit=0.8,
  )
  maximum = StopProfileGenerator.required_stop_distance(
    v_ego=15.0, a_ego=0.0, actuator_delay=0.2, max_brake=3.0, jerk_limit=1.1,
  )
  assert gentle > standard > maximum > 0.0


def test_positive_acceleration_and_longer_delay_increase_required_stop_distance():
  baseline = StopProfileGenerator.required_stop_distance(
    v_ego=10.0, a_ego=0.0, actuator_delay=0.2, max_brake=3.0, jerk_limit=1.0,
  )
  delayed = StopProfileGenerator.required_stop_distance(
    v_ego=10.0, a_ego=0.5, actuator_delay=0.5, max_brake=3.0, jerk_limit=1.0,
  )
  assert delayed > baseline
