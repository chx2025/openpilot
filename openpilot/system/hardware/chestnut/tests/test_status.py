from openpilot.system.hardware.chestnut.status import (
  PCIE_L0,
  ChestnutLinkState,
  classify_chestnut_link,
)
from openpilot.system.hardware.chestnut.statusd import PcieLinkProbe


def chestnut(speed_mbps: int) -> list[dict]:
  return [{"vendorId": 0x3801, "productId": 0x0001, "speedMbps": speed_mbps}]


def dual(speed_mbps: int) -> list[dict]:
  return [{"vendorId": 0xADD1, "productId": 0x0002, "speedMbps": speed_mbps}]


def test_link_classification_distinguishes_usb_and_pcie_faults():
  assert classify_chestnut_link([], None).state == ChestnutLinkState.disconnected
  assert classify_chestnut_link(chestnut(480), None).state == ChestnutLinkState.usb_degraded
  assert classify_chestnut_link(chestnut(5000), 0x00).state == ChestnutLinkState.pcie_down
  assert classify_chestnut_link(chestnut(5000), PCIE_L0).state == ChestnutLinkState.ready
  assert classify_chestnut_link(dual(5000), PCIE_L0).state == ChestnutLinkState.ready


def test_status_daemon_probe_publishes_passive_pcie_result():
  reads = []
  probe = PcieLinkProbe(read_ltssm=lambda: reads.append(True) or PCIE_L0, monotonic=lambda: 10.0)

  valid, ltssm = probe.update(connected=True, usb_speed_mbps=5000)
  assert reads == [True]
  assert valid
  assert ltssm == PCIE_L0


def test_status_daemon_probe_does_not_touch_pcie_on_degraded_usb():
  reads = []
  probe = PcieLinkProbe(read_ltssm=lambda: reads.append(True) or PCIE_L0, monotonic=lambda: 10.0)

  valid, ltssm = probe.update(connected=True, usb_speed_mbps=480)

  assert reads == []
  assert valid
  assert ltssm == 0
