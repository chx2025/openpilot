"""Model-manager parameter migrations for the local dual-slot policy."""

from openpilot.sunnypilot.system.params_migration import _migrate_model_bundle_slots


class FakeParams:
  def __init__(self):
    self.values = {}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put(self, key, value, block=False):
    self.values[key] = value

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)

  def remove(self, key):
    self.values.pop(key, None)


class TestModelBundleSlotMigration:
  def test_legacy_big_model_moves_to_usbgpu_slot_and_remains_selected(self):
    params = FakeParams()
    bundle = {
      "ref": "lm-usbgpu",
      "minimumSelectorVersion": 18,
      "overrides": [{"key": "model_platform", "value": "usbgpu"}],
    }
    params.put("ModelManager_ActiveBundle", bundle, block=True)
    params.put_bool("ModelManager_ActiveBundleRequiresUsbGpu", True, block=True)

    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ActiveBundleUSBGPU") == bundle
    assert params.get("ModelManager_ActiveBundle") is None
    assert params.get("ModelManager_ActiveSource") == "usbgpu"

  def test_legacy_qcom_model_stays_in_qcom_slot_and_remains_selected(self):
    params = FakeParams()
    bundle = {"ref": "cd210", "minimumSelectorVersion": 18}
    params.put("ModelManager_ActiveBundle", bundle, block=True)

    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ActiveBundle") == bundle
    assert params.get("ModelManager_ActiveBundleUSBGPU") is None
    assert params.get("ModelManager_ActiveSource") == "qcom"
