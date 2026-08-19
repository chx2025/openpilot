from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LIMIT_COST, N, T_IDXS
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.long_mpc import (
  LongitudinalMpc as ExperimentalLongitudinalMpc,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc import long_mpc as legacy_mpc_module
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.long_mpc import LegacyCruiseLongitudinalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.long_mpc import (
  LongitudinalMpc as TNLongitudinalMpc,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tuning import (
  CRAZYMAX_VALUES, LongitudinalTuning,
)


def _radar_without_leads():
  lead = SimpleNamespace(present=False, modelProb=0.0)
  return SimpleNamespace(leadOne=lead, leadTwo=lead)


class FakeAcadosSolver:
  """Generated acados is the test boundary; retain exactly what Python sends it."""

  def __init__(self, _model_name, _solver_type, _horizon):
    self.costs = {}
    self.values = {}
    self.solve_calls = 0

  def reset(self):
    pass

  def cost_set(self, stage, field, value):
    self.costs[(stage, field)] = np.array(value, copy=True)

  def set(self, stage, field, value):
    self.values[(stage, field)] = np.array(value, copy=True)

  def constraints_set(self, stage, field, value):
    self.values[(stage, field)] = np.array(value, copy=True)

  def solve(self):
    self.solve_calls += 1
    return 0

  def get_stats(self, _field):
    return np.array([0.0])

  def get(self, _stage, field):
    return np.zeros(3 if field == "x" else 1)


def _fake_mpc(tuning: LongitudinalTuning) -> LegacyCruiseLongitudinalMpc:
  mpc = LegacyCruiseLongitudinalMpc(FakeAcadosSolver, FakeAcadosSolver, dt=0.05)
  mpc.runtime_tuning = tuning
  mpc.solver.costs.clear()
  mpc.solver.values.clear()
  return mpc


def _far_lead():
  return SimpleNamespace(
    present=True, dRel=1_000.0, vLead=10.0, aLeadK=0.0, aLeadTau=1.5, modelProb=0.0,
  )


def _solver_with_status(status):
  class Solver(FakeAcadosSolver):
    def solve(self):
      self.solve_calls += 1
      return status

  return Solver


@pytest.mark.parametrize("mpc_class", [ExperimentalLongitudinalMpc, TNLongitudinalMpc], ids=["experimental", "tn"])
@pytest.mark.parametrize("tuning", [LongitudinalTuning(), LongitudinalTuning(**CRAZYMAX_VALUES)], ids=["default", "crazymax"])
@pytest.mark.parametrize("v_ego", [0.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0])
@pytest.mark.parametrize("cruise_delta", [-10.0, 0.0, 5.0, 20.0])
def test_legacy_solver_converges_across_the_cruise_state_grid(mpc_class, tuning, v_ego, cruise_delta):
  mpc = mpc_class(dt=0.05)
  mpc.runtime_tuning = tuning
  mpc.set_recovery_enabled(True)
  mpc.set_cur_state(v_ego, 0.0)
  mpc.set_weights(prev_accel_constraint=False, personality=log.LongitudinalPersonality.standard)
  mpc.update(
    _radar_without_leads(), max(0.0, v_ego + cruise_delta),
    personality=log.LongitudinalPersonality.standard,
  )

  assert mpc.last_solution_status == 0
  assert np.all(np.isfinite(mpc.v_solution))
  assert np.all(np.isfinite(mpc.a_solution))
  assert np.all(np.isfinite(mpc.j_solution))
  assert np.isfinite(mpc.solve_time)
  assert 0.0 <= mpc.solve_time < 0.05
  assert not (
    np.allclose(mpc.v_solution, 0.0)
    and np.allclose(mpc.a_solution, 0.0)
    and np.allclose(mpc.j_solution, 0.0)
  )


def test_inactive_primary_failure_preserves_the_legacy_reset_path():
  mpc = LegacyCruiseLongitudinalMpc(_solver_with_status(4), _solver_with_status(0), dt=0.05)

  mpc.run()

  assert mpc.last_primary_solution_status == 4
  assert mpc.last_fallback_solution_status is None
  assert mpc.last_solution_status == 4
  assert mpc.fallback_solver.solve_calls == 0
  assert np.allclose(mpc.v_solution, 0.0)


def test_active_primary_failure_uses_the_robust_solver_once(monkeypatch):
  warnings = []
  monkeypatch.setattr(legacy_mpc_module.cloudlog, "warning", warnings.append)
  monotonic_values = iter([10.0, 12.0, 16.0])
  monkeypatch.setattr(legacy_mpc_module.time, "monotonic", lambda: next(monotonic_values))
  mpc = LegacyCruiseLongitudinalMpc(_solver_with_status(4), _solver_with_status(0), dt=0.05)
  mpc.set_recovery_enabled(True)

  for _ in range(3):
    mpc.run()

  assert mpc.last_primary_solution_status == 4
  assert mpc.last_fallback_solution_status == 0
  assert mpc.last_solution_status == 0
  assert mpc.fallback_solver.solve_calls == 3
  assert np.all(np.isfinite(mpc.v_solution))
  assert len(warnings) == 2
  assert all("primary_status: 4" in warning and "fallback_status: 0" in warning for warning in warnings)


def test_successful_primary_never_calls_the_fallback(monkeypatch):
  warnings = []
  monkeypatch.setattr(legacy_mpc_module.cloudlog, "warning", warnings.append)
  mpc = LegacyCruiseLongitudinalMpc(_solver_with_status(0), _solver_with_status(0), dt=0.05)
  mpc.set_recovery_enabled(True)

  mpc.run()

  assert mpc.last_primary_solution_status == 0
  assert mpc.last_fallback_solution_status is None
  assert mpc.last_solution_status == 0
  assert mpc.fallback_solver.solve_calls == 0
  assert warnings == []


def test_legacy_solver_consumes_all_weight_and_constraint_tuning():
  tuning = LongitudinalTuning(
    x_ego_obstacle_cost=4.1,
    j_ego_cost=7.2,
    a_change_cost=123.0,
    danger_zone_cost=87.0,
    jerk_factor_relaxed=1.7,
  )
  mpc = _fake_mpc(tuning)

  mpc.set_weights(prev_accel_constraint=True, personality=log.LongitudinalPersonality.relaxed)

  stage_zero_weights = np.diag(mpc.solver.costs[(0, "W")])
  assert stage_zero_weights == pytest.approx([4.1, 0.0, 0.0, 0.0, 123.0 * 1.7, 7.2 * 1.7])
  assert mpc.solver.costs[(0, "Zl")] == pytest.approx([LIMIT_COST, LIMIT_COST, LIMIT_COST, 87.0])


@pytest.mark.parametrize("personality, expected_t_follow", [
  (log.LongitudinalPersonality.relaxed, 1.91),
  (log.LongitudinalPersonality.standard, 1.52),
  (log.LongitudinalPersonality.aggressive, 1.13),
])
def test_legacy_solver_consumes_following_and_obstacle_tuning(personality, expected_t_follow):
  tuning = LongitudinalTuning(
    t_follow_relaxed=1.91,
    t_follow_standard=1.52,
    t_follow_aggressive=1.13,
    lead_danger_factor=0.62,
    comfort_brake=2.8,
    stop_distance=4.7,
  )
  mpc = _fake_mpc(tuning)
  mpc.set_cur_state(10.0, 0.0)
  lead = _far_lead()

  mpc.update(SimpleNamespace(leadOne=lead, leadTwo=lead), v_cruise=20.0, personality=personality)

  expected_first_cruise_obstacle = 10.0 ** 2 / (2.0 * 2.8) + expected_t_follow * 10.0 + 4.7
  assert mpc.params.shape == (N + 1, 8)
  assert mpc.params[:, 4] == pytest.approx(expected_t_follow)
  assert mpc.params[:, 5] == pytest.approx(0.62)
  assert mpc.params[:, 6] == pytest.approx(2.8)
  assert mpc.params[:, 7] == pytest.approx(4.7)
  assert mpc.params[0, 2] == pytest.approx(expected_first_cruise_obstacle)
  assert np.all(np.diff(mpc.params[:, 2]) > 0.0)
  assert len(T_IDXS) == N + 1
