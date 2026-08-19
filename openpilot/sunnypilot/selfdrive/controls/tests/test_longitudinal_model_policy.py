import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.model_policy import (
  ModelPolicy, default_model_policy, load_model_policy, model_e2e_enabled, save_model_policy,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, get_backend


def test_acc_policy_rejects_model_braking_while_experimental_mode_is_enabled():
  assert not model_e2e_enabled(
    ModelPolicy.ACC, experimental_mode=True, dec_mode="blended",
  )


def test_dynamic_policy_follows_the_decision_controller():
  assert not model_e2e_enabled(ModelPolicy.DYNAMIC, experimental_mode=True, dec_mode="acc")
  assert model_e2e_enabled(ModelPolicy.DYNAMIC, experimental_mode=True, dec_mode="blended")


def test_e2e_policy_respects_the_global_experimental_mode_switch():
  assert model_e2e_enabled(ModelPolicy.E2E, experimental_mode=True)
  assert not model_e2e_enabled(ModelPolicy.E2E, experimental_mode=False)


def test_non_official_backends_default_to_acc_policy():
  assert default_model_policy(get_backend(BackendId.EXPERIMENTAL)) == ModelPolicy.ACC
  assert default_model_policy(get_backend(BackendId.TN_NO_DEC)) == ModelPolicy.ACC


class FakeParams:
  def __init__(self):
    self.values = {}

  def get(self, key, return_default=False):
    del return_default
    return self.values.get(key)

  def put(self, key, value, block=False):
    del block
    self.values[key] = value


def test_each_backend_saves_its_model_policy_independently():
  params = FakeParams()
  experimental = get_backend(BackendId.EXPERIMENTAL)
  tn = get_backend(BackendId.TN_NO_DEC)

  save_model_policy(params, experimental, ModelPolicy.DYNAMIC)
  save_model_policy(params, tn, ModelPolicy.E2E)

  assert load_model_policy(params, experimental) == ModelPolicy.DYNAMIC
  assert load_model_policy(params, tn) == ModelPolicy.E2E


def test_tn_rejects_dynamic_policy_because_it_has_no_dec():
  with pytest.raises(ValueError, match="unsupported"):
    save_model_policy(FakeParams(), get_backend(BackendId.TN_NO_DEC), ModelPolicy.DYNAMIC)
