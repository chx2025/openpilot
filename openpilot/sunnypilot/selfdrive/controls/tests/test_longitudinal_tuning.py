import copy
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log, messaging
from openpilot.common.params import UnknownKeyName
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, get_backend
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tuning import (
  CONFIG_PARAM, DEFAULT_VALUES, LongitudinalTuning, TuningController, adjusted_obstacle, backend_profile, backend_values,
  follow_distance_for_personality, save_backend_values,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key, return_default=False):
    del return_default
    return self.values.get(key)

  def put(self, key, value, block=False):
    del block
    self.values[key] = value


def test_each_planner_keeps_an_independent_custom_profile():
  params = FakeParams()
  official = get_backend(BackendId.OFFICIAL)
  experimental = get_backend(BackendId.EXPERIMENTAL)

  save_backend_values(params, official, {**DEFAULT_VALUES, "j_ego_cost": 6.0}, profile=2)
  save_backend_values(params, experimental, {**DEFAULT_VALUES, "j_ego_cost": 3.0}, profile=2)

  assert backend_values(params, official).j_ego_cost == 6.0
  assert backend_values(params, experimental).j_ego_cost == 3.0
  assert backend_values(params, get_backend(BackendId.TN_NO_DEC)).j_ego_cost == DEFAULT_VALUES["j_ego_cost"]


def test_current_v1_config_migrates_without_numeric_drift_and_keeps_source_backup():
  source = {
    "schemaVersion": 1,
    "revision": 7,
    "backends": {
      "official": {
        "profile": 2,
        "values": {**DEFAULT_VALUES, "j_ego_cost": 6.25},
        "customValues": {**DEFAULT_VALUES, "j_ego_cost": 6.25},
      },
    },
  }
  params = FakeParams({CONFIG_PARAM: source})

  assert backend_values(params, get_backend(BackendId.OFFICIAL)).j_ego_cost == 6.25
  migrated = params.values[CONFIG_PARAM]
  assert migrated["schemaVersion"] == 2
  assert migrated["format"] == "backendProfiles"
  assert migrated["revision"] == 8
  assert migrated["sourceBackup"] == {"format": "current-v1", "config": source}


def test_rs408_semantic_v1_resolves_layers_and_renames_backends():
  source = {
    "schemaVersion": 1,
    "revision": 4,
    "shared": {"following.time.standard_s": 1.55},
    "families": {"acados_long_v1": {"mpc.jerk_cost": 6.25}},
    "backends": {
      "sp_upstream_tunable": {"profileId": 0, "overrides": {"following.time.relaxed_s": 1.85}},
      "local": {"profileId": 2, "overrides": {"mpc.jerk_cost": 4.25}},
      "tn_no_dec": {"profileId": 1, "overrides": {"mpc.obstacle_comfort_brake_mps2": 2.8}},
    },
  }
  params = FakeParams({CONFIG_PARAM: source})
  official = get_backend(BackendId.OFFICIAL)
  experimental = get_backend(BackendId.EXPERIMENTAL)
  tn = get_backend(BackendId.TN_NO_DEC)

  assert backend_values(params, official).t_follow_relaxed == 1.85
  assert backend_values(params, official).t_follow_standard == 1.55
  assert backend_values(params, official).j_ego_cost == 6.25
  assert backend_values(params, experimental).j_ego_cost == 4.25
  assert backend_values(params, tn).comfort_brake == 2.8
  assert [backend_profile(params, backend) for backend in (official, experimental, tn)] == [0, 2, 1]
  migrated = params.values[CONFIG_PARAM]
  assert migrated["revision"] == 5
  assert migrated["sourceBackup"] == {"format": "rs408-semantic-v1", "config": source}


def test_mixed_v1_config_is_rejected_without_overwriting_source():
  source = {
    "schemaVersion": 1,
    "revision": 3,
    "shared": {"following.time.standard_s": 1.55},
    "backends": {
      "official": {
        "profile": 2,
        "values": dict(DEFAULT_VALUES),
        "customValues": dict(DEFAULT_VALUES),
      },
    },
  }

  class TrackingParams(FakeParams):
    def __init__(self, values):
      super().__init__(values)
      self.puts = []

    def put(self, key, value, block=False):
      self.puts.append((key, value, block))
      super().put(key, value, block)

  params = TrackingParams({CONFIG_PARAM: copy.deepcopy(source)})
  with pytest.raises(ValueError, match="mixed|invalid"):
    backend_values(params, get_backend(BackendId.OFFICIAL))
  with pytest.raises(ValueError, match="mixed|invalid"):
    save_backend_values(params, get_backend(BackendId.OFFICIAL), dict(DEFAULT_VALUES), profile=0)

  assert params.values[CONFIG_PARAM] == source
  assert params.puts == []


def test_concurrent_backend_saves_do_not_lose_either_update():
  class SlowParams(FakeParams):
    def get(self, key, return_default=False):
      value = super().get(key, return_default)
      if key == CONFIG_PARAM:
        time.sleep(0.02)
      return value

  params = SlowParams()
  official = get_backend(BackendId.OFFICIAL)
  experimental = get_backend(BackendId.EXPERIMENTAL)
  start = threading.Barrier(3)

  def save(backend, jerk_cost):
    start.wait()
    save_backend_values(params, backend, {**DEFAULT_VALUES, "j_ego_cost": jerk_cost}, profile=2)

  threads = [
    threading.Thread(target=save, args=(official, 6.0)),
    threading.Thread(target=save, args=(experimental, 3.0)),
  ]
  for thread in threads:
    thread.start()
  start.wait()
  for thread in threads:
    thread.join(timeout=2.0)
    assert not thread.is_alive()

  assert backend_values(params, official).j_ego_cost == 6.0
  assert backend_values(params, experimental).j_ego_cost == 3.0
  assert params.values[CONFIG_PARAM]["revision"] == 2


def test_completed_v2_migration_is_idempotent():
  source = {
    "schemaVersion": 1,
    "revision": 2,
    "backends": {
      "experimental": {
        "profile": 2,
        "values": {**DEFAULT_VALUES, "j_ego_cost": 4.0},
        "customValues": {**DEFAULT_VALUES, "j_ego_cost": 4.0},
      },
    },
  }

  class TrackingParams(FakeParams):
    def __init__(self, values):
      super().__init__(values)
      self.puts = 0

    def put(self, key, value, block=False):
      self.puts += 1
      super().put(key, value, block)

  params = TrackingParams({CONFIG_PARAM: source})
  backend = get_backend(BackendId.EXPERIMENTAL)
  assert backend_values(params, backend).j_ego_cost == 4.0
  first = copy.deepcopy(params.values[CONFIG_PARAM])
  assert params.puts == 1

  assert backend_values(params, backend).j_ego_cost == 4.0
  assert params.values[CONFIG_PARAM] == first
  assert params.puts == 1


def test_failed_migration_write_leaves_v1_source_available_for_retry():
  source = {"schemaVersion": 1, "revision": 1, "backends": {}}

  class FailingParams(FakeParams):
    def put(self, key, value, block=False):
      raise OSError("simulated atomic Params write failure")

  params = FailingParams({CONFIG_PARAM: copy.deepcopy(source)})
  with pytest.raises(OSError, match="simulated"):
    backend_values(params, get_backend(BackendId.OFFICIAL))

  assert params.values[CONFIG_PARAM] == source


def test_rs408_tn_native_overrides_do_not_block_mpc_tuning_migration():
  source = {
    "schemaVersion": 1,
    "revision": 5,
    "shared": {},
    "families": {"acados_long_v1": {}},
    "backends": {
      "tn_no_dec": {
        "profileId": 2,
        "overrides": {
          "mpc.jerk_cost": 4.5,
          "tn.accel_personality.enabled": True,
          "tn.accel_personality.profile": 2,
        },
      },
    },
  }
  params = FakeParams({
    CONFIG_PARAM: source,
    "AccelPersonalityEnabled": False,
    "AccelPersonality": 0,
  })

  assert backend_values(params, get_backend(BackendId.TN_NO_DEC)).j_ego_cost == 4.5
  assert params.values["AccelPersonalityEnabled"] is True
  assert params.values["AccelPersonality"] == 2
  assert params.values[CONFIG_PARAM]["sourceBackup"]["config"] == source


def test_rs408_missing_tn_native_overrides_preserve_current_params():
  source = {
    "schemaVersion": 1,
    "revision": 5,
    "shared": {},
    "families": {"acados_long_v1": {}},
    "backends": {"tn_no_dec": {"profileId": 0, "overrides": {"mpc.jerk_cost": 4.5}}},
  }
  params = FakeParams({
    CONFIG_PARAM: source,
    "AccelPersonalityEnabled": True,
    "AccelPersonality": 2,
  })

  assert backend_values(params, get_backend(BackendId.TN_NO_DEC)).j_ego_cost == 4.5
  assert params.values["AccelPersonalityEnabled"] is True
  assert params.values["AccelPersonality"] == 2


def test_tesla_settings_display_invalid_config_without_repairing_it(monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands import tesla_planner

  source = {
    "schemaVersion": 1,
    "revision": 3,
    "shared": {"following.time.standard_s": 1.55},
    "backends": {
      "official": {
        "profile": 2,
        "values": dict(DEFAULT_VALUES),
        "customValues": dict(DEFAULT_VALUES),
      },
    },
  }
  params = FakeParams({CONFIG_PARAM: copy.deepcopy(source), "LongitudinalPlannerMode": 0})
  monkeypatch.setattr(tesla_planner.ui_state, "params", params)
  displayed = []
  layout = tesla_planner.TeslaPlannerSettingsLayout.__new__(tesla_planner.TeslaPlannerSettingsLayout)
  layout.backends = (get_backend(BackendId.OFFICIAL),)
  layout.planner = SimpleNamespace(action_item=SimpleNamespace(set_selected_button=lambda _value: None))
  layout.profile = SimpleNamespace(action_item=SimpleNamespace(set_selected_button=lambda _value: None))
  layout._show_values = displayed.append
  layout._update_visibility = lambda: None

  layout._load_selected_backend()

  assert displayed[0].as_dict() == DEFAULT_VALUES
  assert params.values[CONFIG_PARAM] == source


def test_default_profile_is_numerically_identical_to_upstream():
  params = FakeParams()
  tuning = backend_values(params, get_backend(BackendId.OFFICIAL))

  assert tuning.as_dict() == DEFAULT_VALUES
  assert adjusted_obstacle(42.0, 15.0, 15.0, tuning, 1.45) == 42.0


def test_default_tuning_returns_the_original_obstacle_without_float_math():
  raw = np.array([42.0, 42.00000000000001, 1.0e8], dtype=np.float64)

  adjusted = adjusted_obstacle(raw, 15.0, 15.0, LongitudinalTuning(), 1.45)

  assert adjusted is raw
  assert np.array_equal(adjusted, raw)


def test_comfort_brake_and_stop_distance_adjust_the_solver_obstacle():
  params = FakeParams()
  backend = get_backend(BackendId.EXPERIMENTAL)
  tuned = {**DEFAULT_VALUES, "comfort_brake": 2.7, "stop_distance": 4.5}
  save_backend_values(params, backend, tuned, profile=2)
  tuning = backend_values(params, backend)

  # The compiled solver keeps upstream's 2.5 m/s² and 6 m. The adapter moves
  # the obstacle so the resulting residual is equivalent to the requested values.
  assert adjusted_obstacle(42.0, 15.0, 15.0, tuning, 1.45) != 42.0


def test_hot_ramped_values_move_gradually_and_hot_values_apply_immediately():
  params = FakeParams()
  backend = get_backend(BackendId.OFFICIAL)
  controller = TuningController(params, backend, poll_interval=0.0)
  controller.update(0.0)
  save_backend_values(params, backend, {**DEFAULT_VALUES, "t_follow_standard": 1.65, "j_ego_cost": 8.0}, profile=2)

  first = controller.update(0.5)
  assert first.t_follow_standard == 1.55
  assert first.j_ego_cost == 6.0
  assert controller.target.t_follow_standard == 1.65


def test_invalid_revision_keeps_last_known_good_values():
  params = FakeParams()
  backend = get_backend(BackendId.OFFICIAL)
  save_backend_values(params, backend, {**DEFAULT_VALUES, "j_ego_cost": 6.0}, profile=2)
  controller = TuningController(params, backend, poll_interval=0.0)
  good = controller.update(0.0)
  params.values["LongitudinalTuningConfig"] = {"schemaVersion": 1, "revision": 99, "backends": {"official": {"values": {"j_ego_cost": -1}}}}

  assert controller.update(1.0) == good


def test_old_prebuilt_without_new_param_key_falls_back_to_upstream_defaults():
  class OldPrebuiltParams(FakeParams):
    def get(self, key, return_default=False):
      del return_default
      if key == "LongitudinalTuningConfig":
        raise UnknownKeyName(key.encode())
      return self.values.get(key)

  controller = TuningController(OldPrebuiltParams(), get_backend(BackendId.OFFICIAL), poll_interval=0.0)
  assert controller.update(0.05).as_dict() == DEFAULT_VALUES


def test_capnp_personality_enum_selects_runtime_follow_distance():
  msg = messaging.new_message("selfdriveState")
  msg.selfdriveState.personality = log.LongitudinalPersonality.aggressive
  personality = msg.as_reader().selfdriveState.personality

  assert follow_distance_for_personality(personality, LongitudinalTuning()) == 1.25
