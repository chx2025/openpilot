import copy
import fcntl
import math
from contextlib import contextmanager
from dataclasses import asdict, dataclass, fields, replace
from typing import Any

from openpilot.cereal import log
from openpilot.common.params import UnknownKeyName
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendSpec, ordered_backends


CONFIG_PARAM = "LongitudinalTuningConfig"
SCHEMA_VERSION = 2
CONFIG_FORMAT = "backendProfiles"
CONFIG_LOCK_PATH = "/tmp/longitudinal_tuning_config.lock"


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

RS408_SEMANTIC_TO_NATIVE = {
  "following.time.relaxed_s": "t_follow_relaxed",
  "following.time.standard_s": "t_follow_standard",
  "following.time.aggressive_s": "t_follow_aggressive",
  "mpc.obstacle_cost": "x_ego_obstacle_cost",
  "mpc.jerk_cost": "j_ego_cost",
  "mpc.accel_change_cost": "a_change_cost",
  "mpc.danger_zone_cost": "danger_zone_cost",
  "mpc.lead_danger_factor": "lead_danger_factor",
  "mpc.obstacle_comfort_brake_mps2": "comfort_brake",
  "mpc.obstacle_stop_distance_m": "stop_distance",
  "mpc.jerk_factor.relaxed": "jerk_factor_relaxed",
}
RS408_SHARED_IDS = frozenset(key for key in RS408_SEMANTIC_TO_NATIVE if key.startswith("following."))
RS408_FAMILY_IDS = frozenset(RS408_SEMANTIC_TO_NATIVE) - RS408_SHARED_IDS
RS408_SLUGS = {
  "sp_upstream_tunable": "official",
  "local": "experimental",
  "tn_no_dec": "tn_no_dec",
}
RS408_TN_NATIVE_IDS = frozenset({"tn.accel_personality.enabled", "tn.accel_personality.profile"})


def follow_distance_for_personality(personality, tuning: LongitudinalTuning) -> float:
  # Cap'n Proto message fields are DynamicEnum objects. They compare equal to
  # these integer constants but have different hashes, so a dict keyed by the
  # constants raises KeyError for the real message value.
  if personality == log.LongitudinalPersonality.relaxed:
    return tuning.t_follow_relaxed
  elif personality == log.LongitudinalPersonality.standard:
    return tuning.t_follow_standard
  elif personality == log.LongitudinalPersonality.aggressive:
    return tuning.t_follow_aggressive
  else:
    raise NotImplementedError("Longitudinal personality not supported")


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


def _default_backend_config() -> dict[str, Any]:
  return {"profile": 0, "values": dict(DEFAULT_VALUES), "customValues": dict(DEFAULT_VALUES)}


def _default_config() -> dict[str, Any]:
  return {
    "schemaVersion": SCHEMA_VERSION,
    "format": CONFIG_FORMAT,
    "revision": 0,
    "backends": {backend.slug: _default_backend_config() for backend in ordered_backends()},
  }


def _validate_revision_and_backends(raw: dict[str, Any]) -> dict[str, Any]:
  revision = raw.get("revision")
  backends = raw.get("backends")
  if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0 or not isinstance(backends, dict):
    raise ValueError("invalid longitudinal tuning config")
  known_slugs = {backend.slug for backend in ordered_backends()}
  if set(backends) - known_slugs:
    raise ValueError("invalid longitudinal tuning config")
  return raw


def _validate_backend_config(raw: object, slug: str) -> dict[str, Any]:
  if not isinstance(raw, dict) or set(raw) != {"profile", "values", "customValues"}:
    raise ValueError(f"invalid tuning config for {slug}")
  profile = raw.get("profile")
  if isinstance(profile, bool) or profile not in (0, 1, 2):
    raise ValueError(f"invalid tuning config for {slug}")
  _validated_values(raw.get("values"))
  _validated_values(raw.get("customValues"))
  return raw


def _parse_v2(raw: object) -> dict[str, Any]:
  if not isinstance(raw, dict) or raw.get("schemaVersion") != SCHEMA_VERSION or raw.get("format") != CONFIG_FORMAT:
    raise ValueError("invalid longitudinal tuning config")
  if set(raw) - {"schemaVersion", "format", "revision", "backends", "sourceBackup"}:
    raise ValueError("invalid longitudinal tuning config")
  config = _validate_revision_and_backends(raw)
  for slug, backend_config in config["backends"].items():
    _validate_backend_config(backend_config, slug)
  if "sourceBackup" in config and not isinstance(config["sourceBackup"], dict):
    raise ValueError("invalid longitudinal tuning config")
  return config


def _migrate_current_v1(raw: object) -> dict[str, Any]:
  if not isinstance(raw, dict) or raw.get("schemaVersion") != 1:
    raise ValueError("invalid longitudinal tuning config")
  if set(raw) != {"schemaVersion", "revision", "backends"}:
    raise ValueError("invalid longitudinal tuning config")
  source = _validate_revision_and_backends(raw)
  migrated = _default_config()
  migrated["revision"] = source["revision"] + 1
  for slug, backend_config in source["backends"].items():
    migrated["backends"][slug] = copy.deepcopy(_validate_backend_config(backend_config, slug))
  migrated["sourceBackup"] = {"format": "current-v1", "config": copy.deepcopy(raw)}
  return _parse_v2(migrated)


def _validated_rs408_layer(raw: object, allowed: frozenset[str], label: str) -> dict[str, float]:
  if not isinstance(raw, dict) or set(raw) - allowed:
    raise ValueError(f"invalid rs408 {label} tuning")
  values: dict[str, float] = {}
  for semantic_id, value in raw.items():
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
      raise ValueError(f"invalid rs408 tuning value: {semantic_id}")
    values[semantic_id] = float(value)
  return values


def _validated_rs408_backend_overrides(raw: object, old_slug: str) -> dict[str, float]:
  allowed = frozenset(RS408_SEMANTIC_TO_NATIVE) | (RS408_TN_NATIVE_IDS if old_slug == "tn_no_dec" else frozenset())
  if not isinstance(raw, dict) or set(raw) - allowed:
    raise ValueError(f"invalid rs408 backend tuning for {old_slug}")
  enabled = raw.get("tn.accel_personality.enabled")
  profile = raw.get("tn.accel_personality.profile")
  if enabled is not None and not isinstance(enabled, bool):
    raise ValueError("invalid rs408 TN accel personality enabled value")
  if profile is not None and (isinstance(profile, bool) or not isinstance(profile, int) or profile not in (0, 1, 2)):
    raise ValueError("invalid rs408 TN accel personality profile")
  return _validated_rs408_layer(
    {key: value for key, value in raw.items() if key in RS408_SEMANTIC_TO_NATIVE},
    frozenset(RS408_SEMANTIC_TO_NATIVE),
    "backend",
  )


def _migrate_rs408_v1(raw: object) -> dict[str, Any]:
  if not isinstance(raw, dict) or raw.get("schemaVersion") != 1:
    raise ValueError("invalid longitudinal tuning config")
  if set(raw) - {"schemaVersion", "revision", "shared", "families", "backends"}:
    raise ValueError("invalid longitudinal tuning config")
  revision = raw.get("revision")
  backends = raw.get("backends", {})
  if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0 or not isinstance(backends, dict):
    raise ValueError("invalid longitudinal tuning config")
  if set(backends) - set(RS408_SLUGS):
    raise ValueError("invalid longitudinal tuning config")

  shared = _validated_rs408_layer(raw.get("shared", {}), RS408_SHARED_IDS, "shared")
  families = raw.get("families", {})
  if not isinstance(families, dict) or set(families) - {"acados_long_v1"}:
    raise ValueError("invalid rs408 tuning families")
  family = _validated_rs408_layer(families.get("acados_long_v1", {}), RS408_FAMILY_IDS, "family")

  migrated = _default_config()
  migrated["revision"] = revision + 1
  for old_slug, new_slug in RS408_SLUGS.items():
    backend_raw = backends.get(old_slug, {})
    if not isinstance(backend_raw, dict) or set(backend_raw) - {"profileId", "overrides"}:
      raise ValueError(f"invalid rs408 tuning config for {old_slug}")
    profile = backend_raw.get("profileId", 0)
    if isinstance(profile, bool) or profile not in (0, 1, 2):
      raise ValueError(f"invalid rs408 tuning profile for {old_slug}")
    overrides = _validated_rs408_backend_overrides(backend_raw.get("overrides", {}), old_slug)
    semantic_values = {
      **{semantic_id: DEFAULT_VALUES[native_id] for semantic_id, native_id in RS408_SEMANTIC_TO_NATIVE.items()},
      **shared,
      **family,
      **overrides,
    }
    values = _validated_values({
      native_id: semantic_values[semantic_id] for semantic_id, native_id in RS408_SEMANTIC_TO_NATIVE.items()
    }).as_dict()
    migrated["backends"][new_slug] = {
      "profile": profile,
      "values": values,
      "customValues": dict(values) if profile == 2 else dict(DEFAULT_VALUES),
    }
  migrated["sourceBackup"] = {"format": "rs408-semantic-v1", "config": copy.deepcopy(raw)}
  return _parse_v2(migrated)


def _migrate_v1(raw: object) -> dict[str, Any]:
  if not isinstance(raw, dict) or raw.get("schemaVersion") != 1:
    raise ValueError("invalid longitudinal tuning config")
  backends = raw.get("backends")
  if not isinstance(backends, dict):
    raise ValueError("invalid longitudinal tuning config")
  if set(raw) == {"schemaVersion", "revision", "backends"}:
    if all(
      isinstance(config, dict) and set(config) == {"profile", "values", "customValues"}
      for config in backends.values()
    ):
      return _migrate_current_v1(raw)
  if ({"shared", "families"} & set(raw)) or any(
    isinstance(config, dict) and ({"profileId", "overrides"} & set(config))
    for config in backends.values()
  ):
    return _migrate_rs408_v1(raw)
  raise ValueError("unknown or mixed longitudinal tuning v1 config")


def _rs408_tn_param_updates(raw: object) -> dict[str, bool | int]:
  if not isinstance(raw, dict):
    return {}
  backend = raw.get("backends", {}).get("tn_no_dec", {})
  overrides = backend.get("overrides", {}) if isinstance(backend, dict) else {}
  updates: dict[str, bool | int] = {}
  if "tn.accel_personality.enabled" in overrides:
    updates["AccelPersonalityEnabled"] = overrides["tn.accel_personality.enabled"]
  if "tn.accel_personality.profile" in overrides:
    updates["AccelPersonality"] = overrides["tn.accel_personality.profile"]
  return updates


@contextmanager
def _config_lock():
  with open(CONFIG_LOCK_PATH, "a") as lock_file:
    fcntl.flock(lock_file, fcntl.LOCK_EX)
    try:
      yield
    finally:
      fcntl.flock(lock_file, fcntl.LOCK_UN)


def _config_locked(params: Any) -> dict[str, Any]:
  raw = params.get(CONFIG_PARAM)
  if raw is None:
    return _default_config()
  if isinstance(raw, dict) and raw.get("schemaVersion") == SCHEMA_VERSION:
    return _parse_v2(raw)
  migrated = _migrate_v1(raw)
  if migrated.get("sourceBackup", {}).get("format") == "rs408-semantic-v1":
    for key, value in _rs408_tn_param_updates(raw).items():
      params.put(key, value, block=True)
  params.put(CONFIG_PARAM, migrated, block=True)
  return migrated


def _config(params: Any) -> dict[str, Any]:
  raw = params.get(CONFIG_PARAM)
  if raw is None:
    return _default_config()
  if isinstance(raw, dict) and raw.get("schemaVersion") == SCHEMA_VERSION:
    return _parse_v2(raw)
  with _config_lock():
    return _config_locked(params)


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
  if isinstance(profile, bool) or profile not in (0, 1, 2):
    raise ValueError("invalid longitudinal tuning profile")
  validated = _validated_values(values)
  with _config_lock():
    config = _config_locked(params)
    backends = dict(config["backends"])
    previous = backends.get(backend.slug, {})
    custom_values = validated.as_dict() if profile == 2 else previous.get("customValues", DEFAULT_VALUES)
    backends[backend.slug] = {"profile": profile, "values": validated.as_dict(), "customValues": custom_values}
    params.put(CONFIG_PARAM, {
      **config,
      "revision": config["revision"] + 1,
      "backends": backends,
    }, block=True)


def apply_backend_profile(params: Any, backend: BackendSpec, profile: int) -> LongitudinalTuning:
  if isinstance(profile, bool):
    raise ValueError("invalid longitudinal tuning profile")
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
  if tuning == default:
    return raw_upstream_obstacle
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
