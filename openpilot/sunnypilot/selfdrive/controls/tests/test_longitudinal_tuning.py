from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, get_backend
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tuning import (
  DEFAULT_VALUES, TuningController, adjusted_obstacle, backend_values, save_backend_values,
)
from openpilot.common.params import UnknownKeyName


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


def test_default_profile_is_numerically_identical_to_upstream():
  params = FakeParams()
  tuning = backend_values(params, get_backend(BackendId.OFFICIAL))

  assert tuning.as_dict() == DEFAULT_VALUES
  assert adjusted_obstacle(42.0, 15.0, 15.0, tuning, 1.45) == 42.0


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
