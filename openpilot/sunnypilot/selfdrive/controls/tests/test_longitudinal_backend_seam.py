from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import (
  BACKENDS, BackendId, get_backend, validate_registry,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.session import (
  ACTIVE_BACKEND_PARAM, latch_active_backend,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}
    self.puts = []

  def get(self, key, return_default=False):
    del return_default
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value
    self.puts.append((key, value, block))


def test_registry_starts_with_current_upstream_planner_only():
  validate_registry()
  assert set(BACKENDS) == {BackendId.OFFICIAL}
  assert get_backend(BackendId.OFFICIAL).provider.endswith("longitudinal_planner:LongitudinalPlanner")


def test_unknown_and_not_yet_installed_backends_fail_closed_to_official():
  assert get_backend(None).id == BackendId.OFFICIAL
  assert get_backend("invalid").id == BackendId.OFFICIAL
  assert get_backend(BackendId.EXPERIMENTAL).id == BackendId.OFFICIAL
  assert get_backend(BackendId.TN_NO_DEC).id == BackendId.OFFICIAL


def test_backend_is_latched_across_process_restarts():
  params = FakeParams({"LongitudinalPlannerMode": int(BackendId.OFFICIAL)})
  assert latch_active_backend(params).id == BackendId.OFFICIAL
  assert params.puts == [(ACTIVE_BACKEND_PARAM, int(BackendId.OFFICIAL), True)]

  params.values["LongitudinalPlannerMode"] = int(BackendId.TN_NO_DEC)
  assert latch_active_backend(params).id == BackendId.OFFICIAL
  assert len(params.puts) == 1


def test_uninstalled_backend_selection_latches_safe_provider():
  params = FakeParams({"LongitudinalPlannerMode": int(BackendId.TN_NO_DEC)})
  assert latch_active_backend(params).id == BackendId.OFFICIAL
  assert params.values[ACTIVE_BACKEND_PARAM] == int(BackendId.OFFICIAL)
