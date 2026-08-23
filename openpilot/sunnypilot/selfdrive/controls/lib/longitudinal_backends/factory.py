from importlib import import_module
from typing import Any, Protocol

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendSpec
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.session import latch_active_backend


class PlannerBackend(Protocol):
  sla: Any

  def update(self, sm) -> None: ...
  def publish(self, sm, pm) -> None: ...


def _load_provider(spec: BackendSpec):
  module_name, class_name = spec.provider.split(":", 1)
  return getattr(import_module(module_name), class_name)


def create_longitudinal_planner(CP, CP_SP, *, params=None) -> PlannerBackend:
  """Construct the session-latched provider behind the planner seam."""
  params = Params() if params is None else params
  spec = latch_active_backend(params)
  planner = _load_provider(spec)(CP, CP_SP)
  planner.active_backend_id = spec.id
  planner.mpc.configure_runtime_tuning(params, spec)
  return planner
