from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import (
  BACKENDS, BackendId, get_backend, ordered_backends, validate_registry,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.session import (
  ACTIVE_BACKEND_PARAM, latch_active_backend,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.longcontrol_factory import _load_stopping_policy
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.longcontrol_policy import TNStoppingPolicy


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


def test_registry_keeps_upstream_and_custom_providers_separate():
  validate_registry()
  assert set(BACKENDS) == {BackendId.OFFICIAL, BackendId.EXPERIMENTAL, BackendId.TN_NO_DEC}
  assert get_backend(BackendId.OFFICIAL).provider.endswith("longitudinal_planner:LongitudinalPlanner")
  assert ".experimental.planner:" in get_backend(BackendId.EXPERIMENTAL).provider
  assert ".tn_no_dec.planner:" in get_backend(BackendId.TN_NO_DEC).provider
  assert [backend.id for backend in ordered_backends()] == [BackendId.OFFICIAL, BackendId.EXPERIMENTAL, BackendId.TN_NO_DEC]


def test_unknown_backends_fail_closed_to_official():
  assert get_backend(None).id == BackendId.OFFICIAL
  assert get_backend("invalid").id == BackendId.OFFICIAL
  assert get_backend(BackendId.EXPERIMENTAL).id == BackendId.EXPERIMENTAL
  assert get_backend(BackendId.TN_NO_DEC).id == BackendId.TN_NO_DEC


def test_experimental_provider_is_installed_and_isolated_from_official():
  spec = BACKENDS[BackendId.EXPERIMENTAL]
  module_name, class_name = spec.provider.split(":", 1)
  module_path = Path(__file__).parents[5] / f"{module_name.replace('.', '/')}.py"

  assert module_path.is_file()
  source = module_path.read_text()
  assert f"class {class_name}(UpstreamLongitudinalPlanner)" in source


def test_backend_is_latched_across_process_restarts():
  params = FakeParams({"LongitudinalPlannerMode": int(BackendId.OFFICIAL)})
  assert latch_active_backend(params).id == BackendId.OFFICIAL
  assert params.puts == [(ACTIVE_BACKEND_PARAM, int(BackendId.OFFICIAL), True)]

  params.values["LongitudinalPlannerMode"] = int(BackendId.TN_NO_DEC)
  assert latch_active_backend(params).id == BackendId.OFFICIAL
  assert len(params.puts) == 1


def test_custom_backend_selection_is_latched():
  params = FakeParams({"LongitudinalPlannerMode": int(BackendId.TN_NO_DEC)})
  assert latch_active_backend(params).id == BackendId.TN_NO_DEC
  assert params.values[ACTIVE_BACKEND_PARAM] == int(BackendId.TN_NO_DEC)


def test_default_backend_hooks_preserve_upstream_dec_behavior():
  class FakeDec:
    def __init__(self):
      self.updated = False

    def update(self, sm):
      self.updated = sm

    def mode(self):
      return "blended"

    def enabled(self):
      return True

    def active(self):
      return True

  class State:
    pass

  planner = object.__new__(LongitudinalPlannerSP)
  planner.dec = FakeDec()
  planner._update_backend("sm")
  assert planner.dec.updated == "sm"

  plan = State()
  plan.dec = State()
  planner._publish_backend_state(plan)
  assert plan.dec.enabled and plan.dec.active


def test_tn_backend_does_not_depend_on_dynamic_experimental_control():
  root = Path(__file__).parents[1] / "lib" / "longitudinal_backends" / "tn_no_dec"
  source = "\n".join(path.read_text() for path in root.rglob("*.py"))
  assert "DynamicExperimental" not in source
  assert "self.dec" not in source
  assert "dynamic_experimental_control" not in source.lower()
  assert "enable_dec=False" in source


def test_tn_reuses_upstream_solver_instead_of_forking_generated_code():
  root = Path(__file__).parents[1] / "lib" / "longitudinal_backends" / "tn_no_dec"
  source = (root / "long_mpc.py").read_text()
  assert "UpstreamLongitudinalMpc" in source
  assert "gen_long_ocp" not in source
  assert not (root / "SConscript").exists()


def test_tn_stopping_policy_fails_safe_on_invalid_inputs():
  policy = TNStoppingPolicy()
  cs = SimpleNamespace(vEgo=float("nan"), aEgo=0.0, standstill=False)
  assert policy.stopping_decel_rate(cs, -0.5, -0.2) == 1.0


def test_stopping_policy_is_attached_only_to_tn_backend():
  assert _load_stopping_policy(BACKENDS[BackendId.OFFICIAL]) is None
  assert isinstance(_load_stopping_policy(BACKENDS[BackendId.TN_NO_DEC]), TNStoppingPolicy)
