#!/usr/bin/env python3
"""Safely quiesce the C3XL USBGPU before cutting UT3G external power."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time


VID, PID = 0x3801, 0x0001
PRODUCT = "custom ed4e39b7-CLEAN"
LTSSM = 0xB450
PCIE_L0 = 0x78
IS_OFFROAD = Path("/data/params/d/IsOffroad")


class SafePowerOffError(RuntimeError):
  pass


def conflicting_processes(proc_root: Path = Path("/proc")) -> list[dict]:
  conflicts = []
  for entry in proc_root.iterdir():
    if not entry.name.isdigit() or int(entry.name) == os.getpid():
      continue
    try:
      command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
    except OSError:
      continue
    if any(marker in command for marker in ("modeld", "ModelState", "DEV=USB+AMD")):
      conflicts.append({"pid": int(entry.name), "command": command[:300]})
  return sorted(conflicts, key=lambda item: item["pid"])


def safe_power_off(usb, sleeper=time.sleep) -> dict:
  if usb.product != PRODUCT:
    raise SafePowerOffError(f"unexpected product {usb.product!r}")
  before = bytes(usb.control_read(0xE4, 1, value=LTSSM, timeout=2000))[0]
  if before != PCIE_L0:
    return {
      "schema": "ut3g-safe-f3-poweroff-v1",
      "state": "already-not-l0",
      "ltssm_before": before,
      "ltssm_after": before,
      "f3_writes": 0,
      "persistent_writes": 0,
      "safe_to_cut_external_power": True,
    }
  usb.control_write(0xF3, value=0, timeout=10_000)
  samples = []
  for index in range(4):
    samples.append(bytes(usb.control_read(0xE4, 1, value=LTSSM, timeout=2000))[0])
    if index != 3:
      sleeper(1.0)
  if PCIE_L0 in samples:
    raise SafePowerOffError(f"PCIe returned to L0 during verification: {samples!r}")
  after = samples[-1]
  return {
    "schema": "ut3g-safe-f3-poweroff-v1",
    "state": "f3-powered-off",
    "ltssm_before": before,
    "ltssm_after": after,
    "f3_writes": 1,
    "verification_seconds": 3,
    "ltssm_samples": samples,
    "persistent_writes": 0,
    "safe_to_cut_external_power": True,
  }


def main() -> int:
  if os.geteuid() != 0:
    raise SafePowerOffError("run as root so libusb can claim the device")
  if not IS_OFFROAD.exists() or IS_OFFROAD.read_bytes().replace(b"\0", b"").strip() != b"1":
    raise SafePowerOffError("C3XL is not confirmed offroad")
  conflicts = conflicting_processes()
  if conflicts:
    raise SafePowerOffError(f"GPU users are still running: {conflicts!r}")

  from tinygrad.runtime.support.usb import USB3
  devices = USB3.list_devices(VID, PID)
  if len(devices) != 1:
    raise SafePowerOffError(f"expected one {VID:04x}:{PID:04x} device, found {len(devices)}")
  usb = USB3(devices[0][0])
  print(json.dumps(safe_power_off(usb), sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
