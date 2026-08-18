from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc as UpstreamLongitudinalMpc,
  LongitudinalPlanSource as LongitudinalPlanSource,
  STOP_DISTANCE as STOP_DISTANCE,
  T_IDXS as T_IDXS,
  get_T_FOLLOW as get_T_FOLLOW,
  get_stopped_equivalence_factor as get_stopped_equivalence_factor,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.long_mpc_sp import LongitudinalMpcSP


class LongitudinalMpc(UpstreamLongitudinalMpc, LongitudinalMpcSP):
  """TN policy hooks on the current upstream solver and model."""

  def __init__(self, dt):
    LongitudinalMpcSP.__init__(self)
    UpstreamLongitudinalMpc.__init__(self, dt=dt)

  def _scale_backend_jerk_factor(self, jerk_factor: float) -> float:
    return self.scale_jerk_cost(jerk_factor)

  def _apply_backend_params(self) -> None:
    self.apply_accel_limits()

  def _save_backend_solution_status(self) -> None:
    self.save_solution_status()
