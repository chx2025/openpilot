import hashlib

from openpilot.cereal import custom
from openpilot.common.hardware.hw import Paths
from openpilot.sunnypilot.models.default_model import get_default_model, get_stock_default_model
from openpilot.sunnypilot.models.helpers import get_active_bundle, get_active_model_runner, select_default_model, usbgpu_model_ready, validate_active_bundle


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


def test_user_selected_model_source_wins_over_usbgpu_presence():
  params = FakeParams({
    "ModelManager_ActiveSource": "qcom",
    "ModelManager_ActiveBundle": {
      "internalName": "QCOM",
      "minimumSelectorVersion": 18,
    },
    "ModelManager_ActiveBundleUSBGPU": {
      "internalName": "LM",
      "minimumSelectorVersion": 18,
    },
  })

  assert get_active_bundle(params, usbgpu=True).internalName == "QCOM"

  params.put("ModelManager_ActiveSource", "usbgpu")

  assert get_active_bundle(params, usbgpu=False).internalName == "LM"


def test_select_default_is_atomic_from_ui_perspective():
  params = FakeParams({
    "ModelManager_DownloadIndex": 0,
    "ModelManager_ActiveBundle": {"internalName": "LM"},
    "ModelManager_ActiveBundleRequiresUsbGpu": True,
    "ModelRunnerTypeCache": int(custom.ModelManagerSP.Runner.tinygrad),
  })

  select_default_model(params)

  assert params.get("ModelManager_DownloadIndex") is None
  assert params.get("ModelManager_ActiveBundle") is None
  assert params.get("ModelManager_ActiveBundleRequiresUsbGpu") is None
  assert params.get("ModelRunnerTypeCache") == int(custom.ModelManagerSP.Runner.stock)


def test_default_clears_only_the_user_selected_slot():
  qcom = {"internalName": "QCOM", "minimumSelectorVersion": 18}
  usbgpu = {"internalName": "LM", "minimumSelectorVersion": 18}
  params = FakeParams({
    "ModelManager_ActiveSource": "usbgpu",
    "ModelManager_ActiveBundle": qcom,
    "ModelManager_ActiveBundleUSBGPU": usbgpu,
    "ModelManager_DownloadRef": "other-big-model",
  })

  select_default_model(params)

  assert params.get("ModelManager_DownloadRef") is None
  assert params.get("ModelManager_ActiveBundleUSBGPU") is None
  assert params.get("ModelManager_ActiveBundle") == qcom
  assert params.get("ModelManager_ActiveSource") == "usbgpu"


def test_default_model_name_matches_connected_hardware():
  assert get_default_model(connected=False) == "CD210"
  assert get_default_model(connected=True) == "Lebowski"
  assert get_stock_default_model() == "CD210"


def test_valid_active_model_survives_temporary_catalog_switch(monkeypatch, tmp_path):
  model_data = b"compiled USBGPU model"
  model_name = "driving_lebowski_tinygrad.pkl"
  (tmp_path / model_name).write_bytes(model_data)
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))

  active_bundle = {
    "index": 0,
    "internalName": "LM",
    "displayName": "Lebowski",
    "models": [{
      "type": "chunked",
      "artifact": {
        "fileName": model_name,
        "downloadUri": {
          "uri": "https://example.invalid/lebowski.pkl",
          "sha256": hashlib.sha256(model_data).hexdigest(),
        },
      },
    }],
    "generation": 12,
    "environment": "development",
    "runner": "tinygrad",
    "is20hz": True,
    "ref": "lebowski-usbgpu",
    "minimumSelectorVersion": 17,
  }
  ordinary_catalog_bundle = custom.ModelManagerSP.ModelBundle(**{
    **active_bundle,
    "internalName": "CD210",
    "displayName": "CD210",
    "ref": "cd210-stock",
  })
  params = FakeParams({
    "ModelManager_ActiveBundle": active_bundle,
    "ModelRunnerTypeCache": int(custom.ModelManagerSP.Runner.tinygrad),
  })

  validate_active_bundle(params, [ordinary_catalog_bundle])

  assert params.get("ModelManager_ActiveBundle") == active_bundle
  assert params.get("ModelRunnerTypeCache") == int(custom.ModelManagerSP.Runner.tinygrad)


def test_usbgpu_model_uses_stock_runner_while_hardware_is_absent():
  params = FakeParams({
    "ModelManager_ActiveBundle": {
      "internalName": "LM",
      "runner": "tinygrad",
      "minimumSelectorVersion": 17,
    },
    "ModelManager_ActiveBundleRequiresUsbGpu": True,
    "ModelRunnerTypeCache": int(custom.ModelManagerSP.Runner.tinygrad),
  })

  runner = get_active_model_runner(params, force_check=True, usbgpu_connected=False)

  assert runner == custom.ModelManagerSP.Runner.stock
  assert params.get("ModelRunnerTypeCache") == int(custom.ModelManagerSP.Runner.stock)


def test_usbgpu_model_uses_selected_runner_when_hardware_returns():
  params = FakeParams({
    "ModelManager_ActiveBundle": {
      "internalName": "LM",
      "runner": "tinygrad",
      "minimumSelectorVersion": 17,
    },
    "ModelManager_ActiveBundleRequiresUsbGpu": True,
    "ModelRunnerTypeCache": int(custom.ModelManagerSP.Runner.stock),
  })

  runner = get_active_model_runner(params, force_check=True, usbgpu_connected=True)

  assert runner == custom.ModelManagerSP.Runner.tinygrad
  assert params.get("ModelRunnerTypeCache") == int(custom.ModelManagerSP.Runner.tinygrad)


def test_downloaded_usbgpu_bundle_satisfies_model_readiness(monkeypatch, tmp_path):
  model_data = b"compiled USBGPU model"
  model_name = "driving_lebowski_tinygrad.pkl"
  (tmp_path / model_name).write_bytes(model_data)
  monkeypatch.setattr(Paths, "model_root", staticmethod(lambda: str(tmp_path)))
  monkeypatch.setattr("openpilot.sunnypilot.models.helpers.usbgpu_compiled", lambda: False)
  params = FakeParams({
    "ModelManager_ActiveBundle": {
      "internalName": "LM",
      "overrides": [{"key": "model_platform", "value": "usbgpu"}],
      "models": [{
        "artifact": {
          "fileName": model_name,
          "downloadUri": {"sha256": hashlib.sha256(model_data).hexdigest()},
        },
      }],
      "runner": "tinygrad",
      "minimumSelectorVersion": 17,
    },
  })

  assert usbgpu_model_ready(params)
