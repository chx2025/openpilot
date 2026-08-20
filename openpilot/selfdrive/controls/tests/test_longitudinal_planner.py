from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_BP, J_CRUISE_VALS, get_cruise_accel


def test_e2e_cruise_accel_respects_jerk_limit():
  v_ego = 15.0
  a_cruise_prev = -1.0
  dt = 0.05

  accel = get_cruise_accel(
    True,
    v_cruise=40.0,
    v_ego=v_ego,
    a_cruise_prev=a_cruise_prev,
    angle_steers=0.0,
    CP=SimpleNamespace(steerRatio=1.0, wheelbase=1.0),
    dt=dt,
    accel_coast=0.0,
    allow_throttle=True,
  )

  jerk_limit = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  assert accel == pytest.approx(a_cruise_prev + jerk_limit * dt)
