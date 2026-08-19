from dataclasses import dataclass
from enum import IntEnum


class BackendId(IntEnum):
  OFFICIAL = 0
  EXPERIMENTAL = 1
  TN_NO_DEC = 2


@dataclass(frozen=True)
class BackendSpec:
  id: BackendId
  slug: str
  label: str
  provider: str
  capabilities: frozenset[str] = frozenset()
  stopping_policy: str | None = None


# The official provider always points at the current upstream planner. It is
# deliberately not copied into this module, so an upstream planner update is
# picked up without maintaining a second implementation.
OFFICIAL_BACKEND = BackendSpec(
  id=BackendId.OFFICIAL,
  slug="official",
  label="Official",
  provider="openpilot.selfdrive.controls.lib.longitudinal_planner:LongitudinalPlanner",
  capabilities=frozenset({"upstream"}),
)

EXPERIMENTAL_BACKEND = BackendSpec(
  id=BackendId.EXPERIMENTAL,
  slug="experimental",
  label="Experimental",
  provider="openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.planner:LongitudinalPlanner",
  capabilities=frozenset({"legacy_cruise_mpc", "cruise_obstacle", "dynamic_experimental_control"}),
)

TN_NO_DEC_BACKEND = BackendSpec(
  id=BackendId.TN_NO_DEC,
  slug="tn_no_dec",
  label="TN-NoDEC",
  provider="openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.planner:LongitudinalPlanner",
  capabilities=frozenset({"legacy_cruise_mpc", "no_dynamic_experimental_control", "accel_controller"}),
  stopping_policy="openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.longcontrol_policy:TNStoppingPolicy",
)


BACKENDS: dict[BackendId, BackendSpec] = {
  BackendId.OFFICIAL: OFFICIAL_BACKEND,
  BackendId.EXPERIMENTAL: EXPERIMENTAL_BACKEND,
  BackendId.TN_NO_DEC: TN_NO_DEC_BACKEND,
}


def get_backend(value: object) -> BackendSpec:
  try:
    backend_id = BackendId(int(value))
  except (TypeError, ValueError):
    return OFFICIAL_BACKEND
  return BACKENDS.get(backend_id, OFFICIAL_BACKEND)


def ordered_backends() -> tuple[BackendSpec, ...]:
  return tuple(BACKENDS[backend_id] for backend_id in sorted(BACKENDS))


def validate_registry() -> None:
  if OFFICIAL_BACKEND.id not in BACKENDS:
    raise ValueError("official longitudinal backend is required")
  if len({backend.provider for backend in BACKENDS.values()}) != len(BACKENDS):
    raise ValueError("duplicate longitudinal backend provider")
  if len({backend.slug for backend in BACKENDS.values()}) != len(BACKENDS):
    raise ValueError("duplicate longitudinal backend slug")
  for backend_id, backend in BACKENDS.items():
    if backend_id != backend.id or ":" not in backend.provider:
      raise ValueError(f"invalid longitudinal backend registration: {backend_id}")


validate_registry()
