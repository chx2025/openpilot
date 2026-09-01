"""Pure status-text helpers for the eGPU sidebar icon.

Kept dependency-free (no rl/gui_app imports) so the classification logic can
be unit tested without the rendering stack.
"""
from __future__ import annotations

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS

MIN_USB_SPEED_MBPS = 5000
PCIE_L0 = 0x78


def chestnut_usb_speed_mbps(device_state) -> int:
  speeds = [int(device.speedMbps) for device in device_state.usbState.devices
            if (int(device.vendorId), int(device.productId)) in CHESTNUT_USB_IDS]
  return max(speeds, default=0)


def classify_egpu_link_state(*, present: bool, usb_speed_mbps: int, telemetry_alive: bool,
                             telemetry_valid: bool, pcie_ltssm: int) -> str:
  if not present:
    return "disconnected"
  if usb_speed_mbps < MIN_USB_SPEED_MBPS:
    return "usb_degraded"
  if not telemetry_alive:
    return "unchecked"
  if not telemetry_valid:
    return "check_error"
  if pcie_ltssm != PCIE_L0:
    return "pcie_down"
  return "ready"


def describe_egpu_status(*, compiled: bool, link_state: str, usb_speed_mbps: int,
                         pcie_ltssm: int | None, loading: bool, active: bool | None,
                         model_failed: bool = False) -> tuple[str, str]:
  """Return (short_label, detail_text) for the current eGPU state, in Traditional Chinese.

  Assumes the caller already confirmed the device is present (chestnutPresent);
  this only distinguishes the states beyond "connected".
  """
  if link_state == "usb_degraded":
    return f"USB {usb_speed_mbps}", f"USB 連線速度低於 5000 Mbps（目前 {usb_speed_mbps} Mbps）"
  if link_state == "pcie_down":
    ltssm = f"0x{pcie_ltssm:02X}" if pcie_ltssm is not None else "未知"
    return "PCIE 異常", f"USB 正常，但 PCIe 未進入 L0（LTSSM {ltssm}）"
  if link_state == "check_error":
    return "連結錯誤", "無法讀取 PCIe 連結狀態"
  if link_state != "ready":
    return "檢查中", f"USB {usb_speed_mbps or '?'} Mbps，正在確認 PCIe 狀態"
  if not compiled:
    return "無模型", "USB/PCIe 正常，但尚未下載/編譯 eGPU 大模型"
  if loading:
    return "載入中", "USB/PCIe 正常，大模型載入中"
  if model_failed or active is False:
    return "模型錯誤", "連結正常，但大模型載入或執行失敗"
  if active is True:
    return "運作中", f"eGPU 大模型運作中 · USB {usb_speed_mbps} Mbps · PCIe L0"
  return "就緒", f"eGPU 已就緒 · USB {usb_speed_mbps} Mbps · PCIe L0"
