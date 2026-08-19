from enum import IntEnum

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, BackendSpec


class ModelPolicy(IntEnum):
  ACC = 0
  DYNAMIC = 1
  E2E = 2


POLICY_PARAMS = {
  BackendId.EXPERIMENTAL: "ExperimentalLongitudinalModelPolicy",
  BackendId.TN_NO_DEC: "TNLongitudinalModelPolicy",
}


def default_model_policy(backend: BackendSpec) -> ModelPolicy | None:
  return ModelPolicy.ACC if backend.id in (BackendId.EXPERIMENTAL, BackendId.TN_NO_DEC) else None


def supported_model_policies(backend: BackendSpec) -> tuple[ModelPolicy, ...]:
  if backend.id == BackendId.EXPERIMENTAL:
    return ModelPolicy.ACC, ModelPolicy.DYNAMIC, ModelPolicy.E2E
  if backend.id == BackendId.TN_NO_DEC:
    return ModelPolicy.ACC, ModelPolicy.E2E
  return ()


def load_model_policy(params, backend: BackendSpec) -> ModelPolicy | None:
  default = default_model_policy(backend)
  if default is None:
    return None
  try:
    policy = ModelPolicy(int(params.get(POLICY_PARAMS[backend.id], return_default=True)))
    return policy if policy in supported_model_policies(backend) else default
  except (KeyError, TypeError, ValueError):
    return default


def save_model_policy(params, backend: BackendSpec, policy: ModelPolicy) -> None:
  if policy not in supported_model_policies(backend):
    raise ValueError(f"unsupported model policy for {backend.slug}: {policy}")
  params.put(POLICY_PARAMS[backend.id], int(policy), block=True)


def model_e2e_enabled(policy: ModelPolicy, *, experimental_mode: bool, dec_mode: str | None = None) -> bool:
  if not experimental_mode or policy == ModelPolicy.ACC:
    return False
  if policy == ModelPolicy.DYNAMIC:
    return dec_mode == "blended"
  return policy == ModelPolicy.E2E
