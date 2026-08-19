import math
from dataclasses import asdict, dataclass, fields, replace
from typing import Any

from openpilot.common.params import UnknownKeyName
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendSpec


CONFIG_PARAM = "LongitudinalTuningConfig"
SCHEMA_VERSION = 1


@dataclass(frozen=True)
class LongitudinalTuning:
  t_follow_relaxed: float = 1.75
  t_follow_standard: float = 1.45
  t_follow_aggressive: float = 1.25
  x_ego_obstacle_cost: float = 3.0
  j_ego_cost: float = 5.0
  a_change_cost: float = 200.0
  danger_zone_cost: float = 100.0
  lead_danger_factor: float = 0.75
  comfort_brake: float = 2.5
  stop_distance: float = 6.0
  jerk_factor_relaxed: float = 1.0

  def as_dict(self) -> dict[str, float]:
    return asdict(self)


DEFAULT_VALUES = LongitudinalTuning().as_dict()
CRAZYMAX_VALUES = {
  **DEFAULT_VALUES,
  "x_ego_obstacle_cost": 5.0,
  "j_ego_cost": 3.0,
  "a_change_cost": 100.0,
  "danger_zone_cost": 80.0,
  "lead_danger_factor": 0.35,
  "comfort_brake": 2.7,
  "stop_distance": 4.5,
  "jerk_factor_relaxed": 0.8,
  "t_follow_relaxed": 1.65,
  "t_follow_standard": 1.35,
  "t_follow_aggressive": 1.0,
}

# (minimum, maximum, quantization step, maximum change per second). All values
# are the old C3XL controls expressed in native units instead of hundredths.
VALUE_SPECS = {
  "t_follow_relaxed": (0.50, 4.00, 0.01, 0.20),
  "t_follow_standard": (0.50, 4.00, 0.01, 0.20),
  "t_follow_aggressive": (0.50, 4.00, 0.01, 0.20),
  "x_ego_obstacle_cost": (0.01, 10.0, 0.01, 2.0),
  "j_ego_cost": (0.01, 10.0, 0.01, 2.0),
  "a_change_cost": (0.01, 500.0, 0.01, 100.0),
  "danger_zone_cost": (0.01, 500.0, 0.01, 100.0),
  "lead_danger_factor": (0.01, 5.0, 0.01, 1.0),
  "comfort_brake": (0.50, 5.0, 0.01, 0.25),
  "stop_distance": (1.0, 12.0, 0.01, 0.50),
  "jerk_factor_relaxed": (0.01, 3.0, 0.01, 1.0),
}


def _validated_values(raw: object) -> LongitudinalTuning:
  if not isinstance(raw, dict) or set(raw) != set(DEFAULT_VALUES):
    raise ValueError("longitudinal tuning values are incomplete")
  values: dict[str, float] = {}
  for key, value in raw.items():
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
      raise ValueError(f"invalid longitudinal tuning value: {key}")
    minimum, maximum, step, _ = VALUE_SPECS[key]
    number = float(value)
    if not minimum <= number <= maximum or not math.isclose(round(number / step) * step, number, abs_tol=1e-8):
      raise ValueError(f"longitudinal tuning value outside bounds: {key}")
    values[key] = number
  if not values["t_follow_aggressive"] <= values["t_follow_standard"] <= values["t_follow_relaxed"]:
    raise ValueError("following times must satisfy aggressive <= standard <= relaxed")
  return LongitudinalTuning(**values)


def _config(params: Any) -> dict[str, Any]:
  raw = params.get(CONFIG_PARAM)
  if raw is None:
    return {"schemaVersion": SCHEMA_VERSION, "revision": 0, "backends": {}}
  if not isinstance(raw, dict) or raw.get("schemaVersion") != SCHEMA_VERSION:
    raise ValueError("invalid longitudinal tuning config")
  revision = raw.get("revision")
  backends = raw.get("backends")
  if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0 or not isinstance(backends, dict):
    raise ValueError("invalid longitudinal tuning config")
  return raw


def backend_values(params: Any, backend: BackendSpec) -> LongitudinalTuning:
  config = _config(params)
  backend_config = config["backends"].get(backend.slug)
  if backend_config is None:
    return LongitudinalTuning()
  if not isinstance(backend_config, dict) or backend_config.get("profile") not in (0, 1, 2):
    raise ValueError(f"invalid tuning config for {backend.slug}")
  return _validated_values(backend_config.get("values"))


def backend_profile(params: Any, backend: BackendSpec) -> int:
  try:
    backend_config = _config(params)["backends"].get(backend.slug, {})
  except ValueError:
    return 0
  profile = backend_config.get("profile", 0) if isinstance(backend_config, dict) else 0
  return profile if profile in (0, 1, 2) else 0


def save_backend_values(params: Any, backend: BackendSpec, values: dict[str, float], profile: int) -> None:
  if profile not in (0, 1, 2):
    raise ValueError("invalid longitudinal tuning profile")
  validated = _validated_values(values)
  try:
    config = _config(params)
  except ValueError:
    config = {"schemaVersion": SCHEMA_VERSION, "revision": 0, "backends": {}}
  backends = dict(config["backends"])
  previous = backends.get(backend.slug, {})
  custom_values = validated.as_dict() if profile == 2 else previous.get("customValues", DEFAULT_VALUES)
  backends[backend.slug] = {"profile": profile, "values": validated.as_dict(), "customValues": custom_values}
  params.put(CONFIG_PARAM, {
    "schemaVersion": SCHEMA_VERSION,
    "revision": config["revision"] + 1,
    "backends": backends,
  }, block=True)


def apply_backend_profile(params: Any, backend: BackendSpec, profile: int) -> LongitudinalTuning:
  if profile == 0:
    values = DEFAULT_VALUES
  elif profile == 1:
    values = CRAZYMAX_VALUES
  elif profile == 2:
    try:
      config = _config(params)
      previous = config["backends"].get(backend.slug, {})
      values = previous.get("customValues", DEFAULT_VALUES) if isinstance(previous, dict) else DEFAULT_VALUES
    except ValueError:
      values = DEFAULT_VALUES
  else:
    raise ValueError("invalid longitudinal tuning profile")
  save_backend_values(params, backend, dict(values), profile)
  return _validated_values(values)


def adjusted_obstacle(raw_upstream_obstacle: float, v_lead: float, v_ego: float,
                      tuning: LongitudinalTuning, t_follow: float) -> float:
  """Translate an obstacle for the unchanged upstream 6-parameter solver."""
  default = LongitudinalTuning()
  lead_equivalence_delta = v_lead ** 2 / (2 * tuning.comfort_brake) - v_lead ** 2 / (2 * default.comfort_brake)
  default_safe = v_ego ** 2 / (2 * default.comfort_brake) + t_follow * v_ego + default.stop_distance
  tuned_safe = v_ego ** 2 / (2 * tuning.comfort_brake) + t_follow * v_ego + tuning.stop_distance
  return raw_upstream_obstacle + lead_equivalence_delta + default_safe - tuned_safe


def _ramp(current: LongitudinalTuning, target: LongitudinalTuning, dt: float) -> LongitudinalTuning:
  if dt <= 0:
    return current
  changes = {}
  for field in fields(current):
    old = getattr(current, field.name)
    new = getattr(target, field.name)
    rate = VALUE_SPECS[field.name][3]
    changes[field.name] = old + max(-rate * dt, min(rate * dt, new - old))
  return replace(current, **changes)


class TuningController:
  """Poll a validated backend snapshot and retain the last known-good revision."""
  def __init__(self, params: Any, backend: BackendSpec, poll_interval: float = 1.0):
    self.params = params
    self.backend = backend
    self.poll_interval = poll_interval
    self.poll_elapsed = poll_interval
    self.current = LongitudinalTuning()
    self.target = self.current
    self.initialized = False

  def update(self, dt: float) -> LongitudinalTuning:
    self.poll_elapsed += max(dt, 0.0)
    if self.poll_elapsed >= self.poll_interval:
      self.poll_elapsed = 0.0
      try:
        target = backend_values(self.params, self.backend)
      except (ValueError, UnknownKeyName):
        target = None
      if target is not None:
        self.target = target
        if not self.initialized:
          self.current = target
          self.initialized = True
    self.current = _ramp(self.current, self.target, dt)
    return self.current
