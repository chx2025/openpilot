from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner as UpstreamLongitudinalPlanner
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.long_mpc import LongitudinalMpc


class LongitudinalPlanner(UpstreamLongitudinalPlanner):
  """Experimental provider kept behind the longitudinal backend registry."""

  def __init__(self, CP, CP_SP, **kwargs):
    super().__init__(CP, CP_SP, mpc_factory=LongitudinalMpc, **kwargs)

  def _update_mpc(self, sm, v_cruise: float) -> None:
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)
