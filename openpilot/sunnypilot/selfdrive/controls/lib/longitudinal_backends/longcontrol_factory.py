from importlib import import_module

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendSpec
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.session import latch_active_backend


def _load_stopping_policy(spec: BackendSpec):
  if spec.stopping_policy is None:
    return None
  module_name, class_name = spec.stopping_policy.split(":", 1)
  return getattr(import_module(module_name), class_name)()


def create_long_control(CP, CP_SP, *, params=None) -> LongControl:
  """Attach only the selected backend's stopping policy to upstream control."""
  params = Params() if params is None else params
  spec = latch_active_backend(params)
  return LongControl(CP, CP_SP, stopping_policy=_load_stopping_policy(spec))
