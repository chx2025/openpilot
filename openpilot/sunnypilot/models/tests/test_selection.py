from openpilot.cereal import custom
from openpilot.sunnypilot.models.default_model import get_default_model
from openpilot.sunnypilot.models.helpers import select_default_model


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


def test_select_default_is_atomic_from_ui_perspective():
  params = FakeParams({
    "ModelManager_DownloadIndex": 0,
    "ModelManager_ActiveBundle": {"internalName": "LM"},
    "ModelRunnerTypeCache": int(custom.ModelManagerSP.Runner.tinygrad),
  })

  select_default_model(params)

  assert params.get("ModelManager_DownloadIndex") is None
  assert params.get("ModelManager_ActiveBundle") is None
  assert params.get("ModelRunnerTypeCache") == int(custom.ModelManagerSP.Runner.stock)


def test_default_model_name_matches_connected_hardware():
  assert get_default_model(connected=False) == "CD210"
  assert get_default_model(connected=True) == "Lebowski"
