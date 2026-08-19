from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource as LongitudinalPlanSource,
  T_IDXS as T_IDXS,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.c_generated_code.acados_ocp_solver_pyx import (
  AcadosOcpSolverCython as PrimaryAcadosSolver,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.c_generated_code_fallback.acados_ocp_solver_pyx import (
  AcadosOcpSolverCython as FallbackAcadosSolver,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.long_mpc import LegacyCruiseLongitudinalMpc


class LongitudinalMpc(LegacyCruiseLongitudinalMpc):
  """Final rs408 Experimental MPC behind the current backend registry."""

  def __init__(self, dt):
    super().__init__(PrimaryAcadosSolver, FallbackAcadosSolver, dt)
