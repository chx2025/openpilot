"""Read-only Chestnut USB/PCIe link monitoring for the offroad UI."""
from __future__ import annotations

import ctypes
import fcntl
import os
import struct
from dataclasses import dataclass
from enum import StrEnum

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS
from openpilot.system.hardware.chestnut.flash import Ctrl, USBDEVFS_CONTROL, find_runtime_chestnut, open_device


MIN_USB_SPEED_MBPS = 5000
PCIE_L0 = 0x78
PCIE_LTSSM_ADDRESS = 0xB450


class ChestnutLinkState(StrEnum):
  disconnected = "disconnected"
  usb_degraded = "usb_degraded"
  unchecked = "unchecked"
  check_error = "check_error"
  pcie_down = "pcie_down"
  ready = "ready"


@dataclass(frozen=True)
class ChestnutLinkStatus:
  state: ChestnutLinkState
  usb_speed_mbps: int = 0
  pcie_ltssm: int | None = None


@dataclass(frozen=True)
class ChestnutAsmTelemetry:
  link_valid: bool
  pcie_ltssm: int = 0
  supply_valid: bool = False
  supply_voltage_mv: int = 0
  supply_current_ma: int = 0


def _usb_speed_mbps(usb_state: list[dict]) -> int:
  return max((int(device.get("speedMbps", 0)) for device in usb_state
              if (int(device.get("vendorId", 0)), int(device.get("productId", 0))) in CHESTNUT_USB_IDS), default=0)


def classify_chestnut_link(usb_state: list[dict], pcie_ltssm: int | None, *, read_error: bool = False) -> ChestnutLinkStatus:
  speed = _usb_speed_mbps(usb_state)
  if speed == 0:
    return ChestnutLinkStatus(ChestnutLinkState.disconnected)
  if speed < MIN_USB_SPEED_MBPS:
    return ChestnutLinkStatus(ChestnutLinkState.usb_degraded, speed)
  if read_error:
    return ChestnutLinkStatus(ChestnutLinkState.check_error, speed)
  if pcie_ltssm is None:
    return ChestnutLinkStatus(ChestnutLinkState.unchecked, speed)
  if pcie_ltssm != PCIE_L0:
    return ChestnutLinkStatus(ChestnutLinkState.pcie_down, speed, pcie_ltssm)
  return ChestnutLinkStatus(ChestnutLinkState.ready, speed, pcie_ltssm)


def read_runtime_asm_telemetry(asm) -> ChestnutAsmTelemetry:
  """Read independent runtime link and optional legacy supply telemetry.

  B450 is the authoritative PCIe link signal. Some UT3G firmware revisions do
  not implement the legacy 0xC0/5 supply request; that must not invalidate an
  already successful LTSSM read or the independent GPU SMU metrics.
  """
  try:
    pcie_ltssm = int(asm.read(PCIE_LTSSM_ADDRESS, 1)[0])
  except Exception:
    return ChestnutAsmTelemetry(False)

  try:
    supply = bytes(asm.usb.control_read(0xC0, 5))
    supply_voltage_mv, supply_current_ma = struct.unpack('<Hh', supply[:4])
  except Exception:
    return ChestnutAsmTelemetry(True, pcie_ltssm)
  return ChestnutAsmTelemetry(True, pcie_ltssm, True, supply_voltage_mv, supply_current_ma)


def read_pcie_ltssm() -> int:
  """Read LTSSM over EP0 without changing PCIe power or link state."""
  path, _, _ = find_runtime_chestnut()
  if path is None:
    raise RuntimeError("eGPU is not connected")

  fd = open_device(path)
  try:
    buf = (ctypes.c_ubyte * 1)()
    fcntl.ioctl(fd, USBDEVFS_CONTROL,
                Ctrl(0xC0, 0xE4, PCIE_LTSSM_ADDRESS, 0, 1, 1000, ctypes.cast(buf, ctypes.c_void_p)))
    return int(buf[0])
  finally:
    os.close(fd)
