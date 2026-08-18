#!/usr/bin/env python3
"""Safely detach a Chestnut-connected eGPU while the device is offroad."""
import argparse
import os
import time
from pathlib import Path

from openpilot.system.hardware.chestnut.flash import VBUS_PATH, claim_interface, find_chestnut


DETACH_TIMEOUT = 5.0


def _wait_disconnected(timeout: float = DETACH_TIMEOUT) -> bool:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    path, _, _ = find_chestnut()
    if path is None:
      return True
    time.sleep(0.1)
  return False


def safe_eject() -> bool:
  """Exclusively claim Chestnut, then power it down or remove it from the USB bus.

  Returns whether VBUS was switched off. A False return still means the device is
  safe to unplug: it was removed from the host bus, but remains externally powered.
  """
  path, _, _ = find_chestnut()
  if path is None:
    raise RuntimeError("eGPU is not connected")

  # Claiming the interface is the safety barrier. It fails with EBUSY if modeld,
  # a compiler, or a diagnostic process is still using the bridge.
  fd = claim_interface(path)
  os.close(fd)

  vbus = Path(VBUS_PATH)
  powered_off = vbus.exists()
  detach_path = vbus if powered_off else Path(path) / "remove"
  detach_path.write_text("0\n" if powered_off else "1\n")

  if not _wait_disconnected():
    raise RuntimeError("eGPU did not disconnect from the host")
  return powered_off


def main() -> int:
  parser = argparse.ArgumentParser(description="safely detach the Chestnut eGPU")
  parser.parse_args()
  powered_off = safe_eject()
  print("powered-off" if powered_off else "host-detached", flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
