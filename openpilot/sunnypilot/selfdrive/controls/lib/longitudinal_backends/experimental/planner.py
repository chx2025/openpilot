from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner as UpstreamLongitudinalPlanner
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.long_mpc import LongitudinalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.model_policy import ModelPolicy, model_e2e_enabled


class LongitudinalPlanner(UpstreamLongitudinalPlanner):
  """Experimental provider kept behind the longitudinal backend registry."""

  def __init__(self, CP, CP_SP, **kwargs):
    super().__init__(CP, CP_SP, mpc_factory=LongitudinalMpc, **kwargs)
    self._model_policy = ModelPolicy.ACC
    self._experimental_mode = False

  def set_model_policy(self, policy: ModelPolicy) -> None:
    self._model_policy = ModelPolicy(policy)

  def is_e2e(self, sm) -> bool:
    self._experimental_mode = bool(sm['selfdriveState'].experimentalMode)
    return model_e2e_enabled(
      self._model_policy, experimental_mode=self._experimental_mode, dec_mode=self.dec.mode(),
    )

  def _publish_backend_state(self, longitudinal_plan_sp) -> None:
    super()._publish_backend_state(longitudinal_plan_sp)
    longitudinal_plan_sp.dec.enabled = self._model_policy == ModelPolicy.DYNAMIC
    longitudinal_plan_sp.dec.active = bool(longitudinal_plan_sp.dec.enabled and self._experimental_mode)

  def _update_mpc(self, sm, v_cruise: float) -> None:
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)
