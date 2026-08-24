from openpilot.sunnypilot.modeld_v2.modeld import should_use_usbgpu
from openpilot.sunnypilot.models.tests.test_selection import FakeParams


def test_big_bundle_uses_usbgpu_when_hardware_is_present():
  params = FakeParams({"ModelManager_ActiveBundleRequiresUsbGpu": True})

  assert should_use_usbgpu(params, hardware_present=True)


def test_small_bundle_stays_on_qcom_while_usbgpu_is_physically_connected():
  params = FakeParams({"ModelManager_ActiveBundleRequiresUsbGpu": False})

  assert not should_use_usbgpu(params, hardware_present=True)


def test_big_bundle_does_not_request_amd_when_hardware_is_absent():
  params = FakeParams({"ModelManager_ActiveBundleRequiresUsbGpu": True})

  assert not should_use_usbgpu(params, hardware_present=False)
