from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource as LongitudinalPlanSource,
  STOP_DISTANCE as STOP_DISTANCE,
  T_IDXS as T_IDXS,
  get_T_FOLLOW as get_T_FOLLOW,
  get_stopped_equivalence_factor as get_stopped_equivalence_factor,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.c_generated_code.acados_ocp_solver_pyx import (
  AcadosOcpSolverCython as PrimaryAcadosSolver,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.c_generated_code_fallback.acados_ocp_solver_pyx import (
  AcadosOcpSolverCython as FallbackAcadosSolver,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.long_mpc import LegacyCruiseLongitudinalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.long_mpc_sp import LongitudinalMpcSP


class LongitudinalMpc(LegacyCruiseLongitudinalMpc, LongitudinalMpcSP):
  """Final rs408 TN MPC with the retained acceleration-controller hooks."""

  def __init__(self, dt):
    LongitudinalMpcSP.__init__(self)
    LegacyCruiseLongitudinalMpc.__init__(self, PrimaryAcadosSolver, FallbackAcadosSolver, dt)

  def _scale_legacy_jerk_cost(self, jerk_cost: float) -> float:
    return self.scale_jerk_cost(jerk_cost)

  def _apply_legacy_backend_params(self) -> None:
    self.apply_accel_limits()

  def _legacy_cruise_accel_max(self, stock_accel_max: float) -> float:
    return self.cruise_accel_max(stock_accel_max)

  def _save_backend_solution_status(self) -> None:
    self.save_solution_status()
