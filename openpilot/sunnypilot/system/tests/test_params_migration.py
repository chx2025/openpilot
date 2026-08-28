"""Model-manager parameter migrations into the official Chestnut policy."""

from openpilot.sunnypilot.system.params_migration import _migrate_model_bundle_slots, run_migration


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key, **kwargs):
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
  def test_usbgpu_bundle_and_catalog_cache_seed_official_chestnut_slot(self):
    bundle = {"ref": "lm", "minimumSelectorVersion": 18}
    cache = {"bundles": [{"ref": "lm", "is_big": True, "minimum_selector_version": "18"}]}
    params = FakeParams({
      "OnroadScreenOffBrightnessMigrated": "1.0",
      "OnroadScreenOffTimerMigrated": "1.0",
      "ModelManager_ActiveBundleUSBGPU": bundle,
      "ModelManager_ModelsCache_USBGPU": cache,
      "ModelManager_ActiveSource": "usbgpu",
      "ModelManager_ActiveBundleRequiresUsbGpu": True,
    })

    run_migration(params)

    assert params.get("ModelManager_ActiveBundleChestnut") == bundle
    assert params.get("ModelManager_ModelsCache_Chestnut") == cache
    assert params.get("ModelManager_LastSyncTime_Chestnut") == 0
    assert params.get("ModelManager_ActiveSource") is None
    assert params.get("ModelManager_ActiveBundleRequiresUsbGpu") is None

  def test_pre_split_bundle_seeds_chestnut_without_removing_qcom_slot(self):
    bundle = {"ref": "candidate", "minimumSelectorVersion": 18}
    params = FakeParams({"ModelManager_ActiveBundle": bundle})

    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ActiveBundle") == bundle
    assert params.get("ModelManager_ActiveBundleChestnut") == bundle

  def test_existing_chestnut_values_are_not_overwritten(self):
    params = FakeParams({
      "ModelManager_ActiveBundleUSBGPU": {"ref": "old"},
      "ModelManager_ActiveBundleChestnut": {"ref": "new"},
      "ModelManager_ModelsCache_USBGPU": {"bundles": [{"ref": "old"}]},
      "ModelManager_ModelsCache_Chestnut": {"bundles": [{"ref": "new"}]},
    })

    _migrate_model_bundle_slots(params)
    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ActiveBundleChestnut") == {"ref": "new"}
    assert params.get("ModelManager_ModelsCache_Chestnut") == {"bundles": [{"ref": "new"}]}

  def test_qcom_shaped_legacy_cache_is_not_seeded_as_chestnut(self):
    params = FakeParams({
      "ModelManager_ModelsCache_USBGPU": {"bundles": [{"ref": "small", "is_big": False}]},
    })

    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ModelsCache_Chestnut") is None

  def test_no_selection_is_a_noop(self):
    params = FakeParams()

    _migrate_model_bundle_slots(params)

    assert params.get("ModelManager_ActiveBundleChestnut") is None
