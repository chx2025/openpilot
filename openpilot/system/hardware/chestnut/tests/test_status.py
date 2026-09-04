from openpilot.system.hardware.chestnut.status import (
  PCIE_L0,
  ChestnutLinkState,
  classify_chestnut_link,
  read_runtime_asm_telemetry,
)
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.system.hardware.chestnut.statusd import PcieLinkProbe, _chestnut_usb


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


def test_status_daemon_only_accepts_runtime_chestnut_identity():
  def state(product):
    device = SimpleNamespace(vendorId=0xADD1, productId=0x0002, manufacturer="tiny",
                             product=product, speedMbps=5000)
    return SimpleNamespace(usbState=SimpleNamespace(devices=[device]))

  assert _chestnut_usb(state("custom d1377a01-UT3G-DUAL")) == (True, 5000)
  assert _chestnut_usb(state("custom d1377a01-UT3G-DUAL-DIRTY")) == (False, 0)


def test_supply_telemetry_failure_does_not_invalidate_pcie_link():
  class Usb:
    def control_read(self, *_args, **_kwargs):
      raise RuntimeError("new firmware has no legacy supply telemetry")

  asm = SimpleNamespace(read=lambda address, length: bytes([PCIE_L0]), usb=Usb())

  telemetry = read_runtime_asm_telemetry(asm)

  assert telemetry.link_valid
  assert telemetry.pcie_ltssm == PCIE_L0
  assert not telemetry.supply_valid
  assert telemetry.supply_voltage_mv == 0
  assert telemetry.supply_current_ma == 0


def test_ltssm_failure_still_invalidates_link():
  asm = SimpleNamespace(read=lambda *_args: (_ for _ in ()).throw(RuntimeError("E4 failed")))

  telemetry = read_runtime_asm_telemetry(asm)

  assert not telemetry.link_valid
  assert telemetry.pcie_ltssm == 0


def test_chestnut_message_exposes_independent_validity_domains():
  msg = messaging.new_message("chestnutState", valid=True)
  msg.chestnutState.metricsValid = True
  msg.chestnutState.supplyValid = False

  assert msg.valid
  assert msg.chestnutState.metricsValid
  assert not msg.chestnutState.supplyValid
