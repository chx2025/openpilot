from types import SimpleNamespace as ns
from types import ModuleType
import sys

from openpilot.cereal import custom

fake_ui_state_module = ModuleType("openpilot.selfdrive.ui.ui_state")
fake_ui_state_module.ui_state = None
sys.modules["openpilot.selfdrive.ui.ui_state"] = fake_ui_state_module

from openpilot.selfdrive.ui.sunnypilot import model_info
from openpilot.sunnypilot.models.tests.test_selection import FakeParams


def _bundle(name: str, source: str):
  return custom.ModelManagerSP.ModelBundle(
    internalName=name,
    displayName=f"{name} display",
    ref=f"{name}-ref",
    minimumSelectorVersion=18,
    overrides=[{"key": "model_platform", "value": source}],
  )


def _state(params, bundles):
  return ns(
    params=params,
    sm={"modelManagerSP": ns(availableBundles=bundles)},
    started=False,
    usbgpu=True,
    usbgpu_active=True,
    usbgpu_loading=False,
    usbgpu_compiled=True,
    model_runner_tinygrad=True,
    big_model_failed=False,
  )


def test_model_info_uses_explicit_user_source_with_egpu_connected(monkeypatch):
  qcom, usbgpu = _bundle("QCOM", "qcom"), _bundle("BIG", "usbgpu")
  params = FakeParams({
    "ModelManager_ActiveSource": "qcom",
    "ModelManager_ActiveBundle": qcom.to_dict(),
    "ModelManager_ActiveBundleUSBGPU": usbgpu.to_dict(),
  })
  monkeypatch.setattr(model_info, "ui_state", _state(params, [qcom, usbgpu]))

  assert model_info.active_source() == "qcom"
  assert [bundle.ref for bundle in model_info.bundles_for_source("qcom")] == ["QCOM-ref"]
  assert [bundle.ref for bundle in model_info.bundles_for_source("usbgpu")] == ["BIG-ref"]
  assert model_info.model_info() == ("qcom", "QCOM display", "BIG display")
  assert model_info.carrying_model() == ("qcom", "QCOM", "QCOM display")


def test_failed_user_selected_big_model_reports_stock_small_fallback(monkeypatch):
  usbgpu = _bundle("BIG", "usbgpu")
  params = FakeParams({
    "ModelManager_ActiveSource": "usbgpu",
    "ModelManager_ActiveBundleUSBGPU": usbgpu.to_dict(),
  })
  state = _state(params, [usbgpu])
  state.started = True
  state.big_model_failed = True
  monkeypatch.setattr(model_info, "ui_state", state)

  source, internal_name, display_name = model_info.carrying_model()

  assert source == "qcom"
  assert "CD210" in internal_name
  assert display_name == internal_name
