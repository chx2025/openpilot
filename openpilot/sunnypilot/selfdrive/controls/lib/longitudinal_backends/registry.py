from dataclasses import dataclass
from enum import IntEnum


class BackendId(IntEnum):
  OFFICIAL = 0
  EXPERIMENTAL = 1
  TN_NO_DEC = 2


@dataclass(frozen=True)
class BackendSpec:
  id: BackendId
  label: str
  provider: str
  capabilities: frozenset[str] = frozenset()


# The official provider always points at the current upstream planner. It is
# deliberately not copied into this module, so an upstream planner update is
# picked up without maintaining a second implementation.
OFFICIAL_BACKEND = BackendSpec(
  id=BackendId.OFFICIAL,
  label="Official",
  provider="openpilot.selfdrive.controls.lib.longitudinal_planner:LongitudinalPlanner",
  capabilities=frozenset({"upstream"}),
)


BACKENDS: dict[BackendId, BackendSpec] = {
  BackendId.OFFICIAL: OFFICIAL_BACKEND,
}


def get_backend(value: object) -> BackendSpec:
  try:
    backend_id = BackendId(int(value))
  except (TypeError, ValueError):
    return OFFICIAL_BACKEND
  return BACKENDS.get(backend_id, OFFICIAL_BACKEND)


def validate_registry() -> None:
  if OFFICIAL_BACKEND.id not in BACKENDS:
    raise ValueError("official longitudinal backend is required")
  if len({backend.provider for backend in BACKENDS.values()}) != len(BACKENDS):
    raise ValueError("duplicate longitudinal backend provider")
  for backend_id, backend in BACKENDS.items():
    if backend_id != backend.id or ":" not in backend.provider:
      raise ValueError(f"invalid longitudinal backend registration: {backend_id}")


validate_registry()
