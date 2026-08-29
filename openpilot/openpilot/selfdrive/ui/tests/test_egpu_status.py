from openpilot.selfdrive.ui.egpu_status import chestnut_usb_speed_mbps, classify_egpu_link_state, describe_egpu_status


class FakeUsbDevice:
  def __init__(self, vendor_id: int, product_id: int, speed_mbps: int):
    self.vendorId, self.productId, self.speedMbps = vendor_id, product_id, speed_mbps


class FakeUsbState:
  def __init__(self, devices):
    self.devices = devices


class FakeDeviceState:
  def __init__(self, devices):
    self.usbState = FakeUsbState(devices)


def test_chestnut_usb_speed_ignores_unrelated_devices():
  device_state = FakeDeviceState([
    FakeUsbDevice(0xADD1, 0x0002, 5000),
    FakeUsbDevice(0x1234, 0x5678, 10000),
  ])
  assert chestnut_usb_speed_mbps(device_state) == 5000


def test_chestnut_usb_speed_absent_is_zero():
  assert chestnut_usb_speed_mbps(FakeDeviceState([])) == 0


def test_link_state_progression():
  assert classify_egpu_link_state(present=False, usb_speed_mbps=0, telemetry_alive=False, telemetry_valid=False, pcie_ltssm=0) == "disconnected"
  assert classify_egpu_link_state(present=True, usb_speed_mbps=480, telemetry_alive=False, telemetry_valid=False, pcie_ltssm=0) == "usb_degraded"
  assert classify_egpu_link_state(present=True, usb_speed_mbps=5000, telemetry_alive=False, telemetry_valid=False, pcie_ltssm=0) == "unchecked"
  assert classify_egpu_link_state(present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=False, pcie_ltssm=0) == "check_error"
  assert classify_egpu_link_state(present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x11) == "pcie_down"
  assert classify_egpu_link_state(present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x78) == "ready"


def test_describe_status_not_compiled_is_no_model():
  label, detail = describe_egpu_status(compiled=False, link_state="ready", usb_speed_mbps=5000, pcie_ltssm=0x78, loading=False, active=None)
  assert label == "無模型"
  assert "尚未下載" in detail


def test_describe_status_loading():
  label, _ = describe_egpu_status(compiled=True, link_state="ready", usb_speed_mbps=5000, pcie_ltssm=0x78, loading=True, active=None)
  assert label == "載入中"


def test_describe_status_active():
  label, detail = describe_egpu_status(compiled=True, link_state="ready", usb_speed_mbps=5000, pcie_ltssm=0x78, loading=False, active=True)
  assert label == "運作中"
  assert "5000" in detail


def test_describe_status_ready_but_idle():
  label, _ = describe_egpu_status(compiled=True, link_state="ready", usb_speed_mbps=5000, pcie_ltssm=0x78, loading=False, active=None)
  assert label == "就緒"


def test_describe_status_model_failed():
  label, _ = describe_egpu_status(compiled=True, link_state="ready", usb_speed_mbps=5000, pcie_ltssm=0x78, loading=False, active=False)
  assert label == "模型錯誤"


def test_describe_status_pcie_down_includes_ltssm_hex():
  _, detail = describe_egpu_status(compiled=False, link_state="pcie_down", usb_speed_mbps=5000, pcie_ltssm=0x11, loading=False, active=None)
  assert "0x11" in detail


def test_describe_status_usb_degraded():
  label, detail = describe_egpu_status(compiled=False, link_state="usb_degraded", usb_speed_mbps=480, pcie_ltssm=None, loading=False, active=None)
  assert label == "USB 480"
  assert "480" in detail
