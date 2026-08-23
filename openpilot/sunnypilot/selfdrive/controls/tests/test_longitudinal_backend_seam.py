from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import (
  BACKENDS, BackendId, get_backend, ordered_backends, validate_registry,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends import factory
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


def test_custom_backends_advertise_the_solver_they_actually_use():
  assert BACKENDS[BackendId.OFFICIAL].capabilities == frozenset({"upstream"})
  assert "legacy_cruise_mpc" in BACKENDS[BackendId.EXPERIMENTAL].capabilities
  assert "legacy_cruise_mpc" in BACKENDS[BackendId.TN_NO_DEC].capabilities
  assert "upstream_mpc" not in BACKENDS[BackendId.EXPERIMENTAL].capabilities
  assert "upstream_mpc" not in BACKENDS[BackendId.TN_NO_DEC].capabilities


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
  assert "def is_e2e" not in source  # Preserve the shared legacy DEC/Experimental Mode behavior.


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


def test_factory_returns_the_selected_backend_without_a_wrapper(monkeypatch):
  configured = []
  planner = SimpleNamespace(
    mpc=SimpleNamespace(configure_runtime_tuning=lambda params, spec: configured.append((params, spec))),
  )
  spec = SimpleNamespace(id=BackendId.OFFICIAL)
  params = FakeParams({})
  monkeypatch.setattr(factory, "latch_active_backend", lambda _: spec)
  monkeypatch.setattr(factory, "_load_provider", lambda _: lambda CP, CP_SP: planner)

  result = factory.create_longitudinal_planner(
    SimpleNamespace(brand="tesla"), SimpleNamespace(), params=params,
  )

  assert result is planner
  assert planner.active_backend_id == BackendId.OFFICIAL
  assert configured == [(params, spec)]


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
  assert "return sm['selfdriveState'].experimentalMode" in source


def test_custom_backends_share_one_generated_legacy_solver_contract():
  root = Path(__file__).parents[1] / "lib" / "longitudinal_backends"
  experimental_source = (root / "experimental" / "long_mpc.py").read_text()
  tn_source = (root / "tn_no_dec" / "long_mpc.py").read_text()

  assert "LegacyCruiseLongitudinalMpc" in experimental_source
  assert "LegacyCruiseLongitudinalMpc" in tn_source
  assert ".legacy_mpc.c_generated_code." in experimental_source
  assert ".legacy_mpc.c_generated_code." in tn_source
  assert not (root / "experimental" / "SConscript").exists()
  assert not (root / "tn_no_dec" / "SConscript").exists()
  assert (root / "legacy_mpc" / "SConscript").is_file()
  ignored = (root / "legacy_mpc" / ".gitignore").read_text().splitlines()
  assert "/c_generated_code/" in ignored
  assert "/c_generated_code_fallback/" in ignored


def test_custom_planners_only_enable_solver_recovery_while_longitudinal_is_active():
  root = Path(__file__).parents[1] / "lib" / "longitudinal_backends"
  for backend in ("experimental", "tn_no_dec"):
    source = (root / backend / "planner.py").read_text()
    assert "set_recovery_enabled(sm['carControl'].longActive)" in source


def test_tn_stopping_policy_fails_safe_on_invalid_inputs():
  policy = TNStoppingPolicy()
  cs = SimpleNamespace(vEgo=float("nan"), aEgo=0.0, standstill=False)
  assert policy.stopping_decel_rate(cs, -0.5, -0.2) == 1.0


def test_stopping_policy_is_attached_only_to_tn_backend():
  assert _load_stopping_policy(BACKENDS[BackendId.OFFICIAL]) is None
  assert isinstance(_load_stopping_policy(BACKENDS[BackendId.TN_NO_DEC]), TNStoppingPolicy)
