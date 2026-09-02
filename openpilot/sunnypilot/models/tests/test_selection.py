from typing import cast

from openpilot.common.params import Params
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION, get_active_bundle


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)

  def remove(self, key):
    self.values.pop(key, None)


def test_connected_chestnut_selects_big_slot_and_disconnected_selects_qcom():
  params = FakeParams({
    "ModelManager_ActiveBundle": {
      "internalName": "QCOM",
      "minimumSelectorVersion": REQUIRED_JSON_VERSION,
    },
    "ModelManager_ActiveBundleChestnut": {
      "internalName": "LM",
      "minimumSelectorVersion": REQUIRED_JSON_VERSION,
    },
  })

  typed_params = cast(Params, params)
  assert get_active_bundle(typed_params, chestnut=True).internalName == "LM"
  assert get_active_bundle(typed_params, chestnut=False).internalName == "QCOM"
