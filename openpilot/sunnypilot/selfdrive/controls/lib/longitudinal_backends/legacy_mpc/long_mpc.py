#!/usr/bin/env python3
import os
import time

import numpy as np

from openpilot.cereal import log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  ACADOS_SOLVER_TYPE,
  A_EGO_COST,
  COST_DIM,
  COST_E_DIM,
  CRASH_DISTANCE,
  FCW_IDXS,
  LIMIT_COST,
  LongitudinalMpc as UpstreamLongitudinalMpc,
  LongitudinalPlanSource,
  N,
  T_DIFFS,
  T_IDXS,
  X_EGO_COST,
  V_EGO_COST,
  get_jerk_factor,
)
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tuning import LongitudinalTuning


MODEL_NAME = "sp_legacy_cruise_v1"
FALLBACK_MODEL_NAME = f"{MODEL_NAME}_fallback"
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
FALLBACK_EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code_fallback")
JSON_FILE = os.path.join(LONG_MPC_DIR, f"acados_ocp_{MODEL_NAME}.json")
FALLBACK_JSON_FILE = os.path.join(LONG_MPC_DIR, f"acados_ocp_{FALLBACK_MODEL_NAME}.json")

PARAM_DIM = 8
CONSTR_DIM = 4
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise)


def get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance):
  return v_ego ** 2 / (2 * comfort_brake) + t_follow * v_ego + stop_distance


def generate_legacy_ocp(model_name=MODEL_NAME, export_dir=EXPORT_DIR, qp_solver_cond_n=1):
  from acados.acados_template import AcadosModel, AcadosOcp
  from casadi import SX, vertcat
  from openpilot.selfdrive.modeld.constants import index_function

  model = AcadosModel()
  model.name = model_name

  x_ego, v_ego, a_ego = SX.sym("x_ego"), SX.sym("v_ego"), SX.sym("a_ego")
  model.x = vertcat(x_ego, v_ego, a_ego)
  j_ego = SX.sym("j_ego")
  model.u = vertcat(j_ego)
  xdot = vertcat(SX.sym("x_ego_dot"), SX.sym("v_ego_dot"), SX.sym("a_ego_dot"))
  model.xdot = xdot

  a_min = SX.sym("a_min")
  a_max = SX.sym("a_max")
  x_obstacle = SX.sym("x_obstacle")
  a_prev = SX.sym("a_prev")
  lead_t_follow = SX.sym("lead_t_follow")
  lead_danger_factor = SX.sym("lead_danger_factor")
  comfort_brake = SX.sym("comfort_brake")
  stop_distance = SX.sym("stop_distance")
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor,
                    comfort_brake, stop_distance)

  dynamics = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = xdot - dynamics
  model.f_expl_expr = dynamics

  desired_distance = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)
  costs = [
    ((x_obstacle - x_ego) - desired_distance) / (v_ego + 10.0),
    x_ego,
    v_ego,
    a_ego,
    a_ego - a_prev,
    j_ego,
  ]
  model.cost_y_expr = vertcat(*costs)
  model.cost_y_expr_e = vertcat(*costs[:-1])
  model.con_h_expr = vertcat(
    v_ego,
    a_ego - a_min,
    a_max - a_ego,
    ((x_obstacle - x_ego) - lead_danger_factor * desired_distance) / (v_ego + 10.0),
  )

  ocp = AcadosOcp()
  ocp.model = model
  ocp.dims.N = N
  ocp.cost.cost_type = "NONLINEAR_LS"
  ocp.cost.cost_type_e = "NONLINEAR_LS"
  ocp.cost.W = np.zeros((COST_DIM, COST_DIM))
  ocp.cost.W_e = np.zeros((COST_E_DIM, COST_E_DIM))
  ocp.cost.yref = np.zeros(COST_DIM)
  ocp.cost.yref_e = np.zeros(COST_E_DIM)
  ocp.cost.zl = np.zeros(CONSTR_DIM)
  ocp.cost.Zl = np.zeros(CONSTR_DIM)
  ocp.cost.Zu = np.zeros(CONSTR_DIM)
  ocp.cost.zu = np.zeros(CONSTR_DIM)
  ocp.constraints.x0 = np.zeros(3)
  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)
  ocp.parameter_values = np.array([
    ACCEL_MIN, ACCEL_MAX, 0.0, 0.0, 1.45, 0.75, 2.5, 6.0,
  ])

  ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
  ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
  ocp.solver_options.integrator_type = "ERK"
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = qp_solver_cond_n
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3
  ocp.solver_options.tf = T_IDXS[-1]
  ocp.solver_options.shooting_nodes = np.array([
    index_function(index, max_val=10.0, max_idx=N) for index in range(N + 1)
  ])
  ocp.code_export_directory = export_dir
  return ocp


class LegacyCruiseLongitudinalMpc(UpstreamLongitudinalMpc):
  """The final rs408 eight-parameter cruise-obstacle MPC on the current runtime."""

  def __init__(self, solver_class, fallback_solver_class, dt):
    self.dt = dt
    self.runtime_tuning = LongitudinalTuning()
    self._tuning_controller = None
    self._recovery_enabled = False
    self.last_solution_status = 0
    self.last_primary_solution_status = 0
    self.last_fallback_solution_status: int | None = None
    self.solver = solver_class(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.fallback_solver = fallback_solver_class(FALLBACK_MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = LongitudinalPlanSource.cruise

  def reset(self):
    self.solver.reset()
    self.fallback_solver.reset()
    self.x_sol = np.zeros((N + 1, 3))
    self.u_sol = np.zeros((N, 1))
    self.v_solution = np.zeros(N + 1)
    self.a_solution = np.zeros(N + 1)
    self.j_solution = np.zeros(N)
    self.a_prev = np.array(self.a_solution)
    self.yref = np.zeros((N + 1, COST_DIM))
    for index in range(N):
      self.solver.cost_set(index, "yref", self.yref[index])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.params = np.zeros((N + 1, PARAM_DIM))
    for index in range(N + 1):
      self.solver.set(index, "x", np.zeros(3))
    self.last_cloudlog_t = 0
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.solve_time = 0.0
    self.x0 = np.zeros(3)
    self.set_weights()

  def _scale_legacy_jerk_cost(self, jerk_cost: float) -> float:
    return jerk_cost

  def _apply_legacy_backend_params(self) -> None:
    pass

  def _save_backend_solution_status(self) -> None:
    # Upstream reset clears solution_status after a failed solve. Retain the
    # result so callers and the deterministic convergence grid can fail closed.
    self.last_solution_status = self.solution_status

  @staticmethod
  def _set_cost_weights_on_solver(solver, cost_weights, constraint_cost_weights) -> None:
    weights = np.asfortranarray(np.diag(cost_weights))
    for index in range(N):
      weights[4, 4] = cost_weights[4] * np.interp(T_IDXS[index], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      solver.cost_set(index, "W", weights)
    solver.cost_set(N, "W", np.copy(weights[:COST_E_DIM, :COST_E_DIM]))
    for index in range(N):
      solver.cost_set(index, "Zl", np.asarray(constraint_cost_weights))

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    self._last_cost_weights = tuple(cost_weights)
    self._last_constraint_cost_weights = tuple(constraint_cost_weights)
    self._set_cost_weights_on_solver(self.solver, cost_weights, constraint_cost_weights)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    self._refresh_runtime_tuning()
    tuning = self.runtime_tuning
    jerk_factor = get_jerk_factor(personality)
    if personality == log.LongitudinalPersonality.relaxed:
      jerk_factor *= tuning.jerk_factor_relaxed
    a_change_cost = tuning.a_change_cost if prev_accel_constraint else 0.0
    cost_weights = [
      tuning.x_ego_obstacle_cost,
      X_EGO_COST,
      V_EGO_COST,
      A_EGO_COST,
      jerk_factor * a_change_cost,
      self._scale_legacy_jerk_cost(jerk_factor * tuning.j_ego_cost),
    ]
    constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, tuning.danger_zone_cost]
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def update(self, radarstate, v_cruise, personality=log.LongitudinalPersonality.standard):
    tuning = self.runtime_tuning
    if personality == log.LongitudinalPersonality.relaxed:
      t_follow = tuning.t_follow_relaxed
    elif personality == log.LongitudinalPersonality.standard:
      t_follow = tuning.t_follow_standard
    elif personality == log.LongitudinalPersonality.aggressive:
      t_follow = tuning.t_follow_aggressive
    else:
      raise NotImplementedError("Longitudinal personality not supported")

    v_ego = self.x0[1]
    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)
    lead_0_obstacle = lead_xv_0[:, 0] + lead_xv_0[:, 1] ** 2 / (2 * tuning.comfort_brake)
    lead_1_obstacle = lead_xv_1[:, 0] + lead_xv_1[:, 1] ** 2 / (2 * tuning.comfort_brake)

    v_lower = v_ego + T_IDXS * CRUISE_MIN_ACCEL * 1.05
    v_upper = v_ego + T_IDXS * self._legacy_cruise_accel_max(CRUISE_MAX_ACCEL) * 1.05
    v_cruise_clipped = np.clip(np.full(N + 1, v_cruise), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(
      v_cruise_clipped, t_follow, tuning.comfort_brake, tuning.stop_distance,
    )
    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:, :] = 0.0
    for index in range(N):
      self.solver.set(index, "yref", self.yref[index])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])
    self.params[:, 0] = ACCEL_MIN
    self.params[:, 1] = ACCEL_MAX
    self._apply_legacy_backend_params()
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.a_prev)
    self.params[:, 4] = t_follow
    self.params[:, 5] = tuning.lead_danger_factor
    self.params[:, 6] = tuning.comfort_brake
    self.params[:, 7] = tuning.stop_distance
    self.run()

    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE)
        and radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def _legacy_cruise_accel_max(self, stock_accel_max: float) -> float:
    return stock_accel_max

  def set_recovery_enabled(self, enabled: bool) -> None:
    self._recovery_enabled = bool(enabled)

  def _prepare_fallback_solver(self) -> None:
    solver = self.fallback_solver
    solver.reset()
    for index in range(N + 1):
      solver.set(index, "x", self.x0)
      solver.set(index, "p", self.params[index])
    for index in range(N):
      solver.cost_set(index, "yref", self.yref[index])
    solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self._set_cost_weights_on_solver(solver, self._last_cost_weights, self._last_constraint_cost_weights)
    solver.constraints_set(0, "lbx", self.x0)
    solver.constraints_set(0, "ubx", self.x0)

  def run(self):
    for index in range(N + 1):
      self.solver.set(index, "p", self.params[index])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    primary_status = self.solver.solve()
    self.last_primary_solution_status = primary_status
    self.last_fallback_solution_status = None
    selected_solver = self.solver
    solve_time = float(self.solver.get_stats("time_tot")[0])

    if primary_status != 0 and self._recovery_enabled:
      self._prepare_fallback_solver()
      fallback_status = self.fallback_solver.solve()
      self.last_fallback_solution_status = fallback_status
      solve_time += float(self.fallback_solver.get_stats("time_tot")[0])
      if fallback_status == 0:
        selected_solver = self.fallback_solver
      self.solution_status = fallback_status
    else:
      self.solution_status = primary_status

    self._save_backend_solution_status()
    if primary_status != 0 and self.last_fallback_solution_status == 0:
      now = time.monotonic()
      if now > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = now
        warning = f"Legacy long mpc recovered, primary_status: {primary_status}, " + \
                  f"fallback_status: {self.last_fallback_solution_status}"
        cloudlog.warning(warning)
    self.solve_time = solve_time
    for index in range(N + 1):
      self.x_sol[index] = selected_solver.get(index, "x")
    for index in range(N):
      self.u_sol[index] = selected_solver.get(index, "u")
    self.v_solution = self.x_sol[:, 1]
    self.a_solution = self.x_sol[:, 2]
    self.j_solution = self.u_sol[:, 0]
    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    if primary_status != 0 and self.solution_status == 0:
      # Keep the legacy primary as the first attempt next cycle, but don't
      # retain its failed QP memory. The fallback trajectory remains published.
      self.solver.reset()
      for index in range(N + 1):
        self.solver.set(index, "x", self.x0)
    elif self.solution_status != 0:
      now = time.monotonic()
      if now > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = now
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  from acados.acados_template import AcadosOcpSolver

  AcadosOcpSolver.generate(generate_legacy_ocp(), json_file=JSON_FILE)
  # Same OCP and runtime parameters, with less aggressive QP condensing. This
  # is a numerical recovery solver, not another longitudinal policy.
  AcadosOcpSolver.generate(
    generate_legacy_ocp(FALLBACK_MODEL_NAME, FALLBACK_EXPORT_DIR, N),
    json_file=FALLBACK_JSON_FILE,
  )
