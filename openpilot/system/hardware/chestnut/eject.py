#!/usr/bin/env python3
"""Prepare a Chestnut-connected eGPU for physical removal while offroad."""
import argparse
import os
import sys
import time
from pathlib import Path

from openpilot.system.hardware.chestnut.flash import VBUS_PATH, claim_interface, find_runtime_chestnut


DETACH_TIMEOUT = 20.0
DETACH_PENDING_EXIT_CODE = 75  # EX_TEMPFAIL: the accepted USB remove is still converging


class DetachPendingError(RuntimeError):
  pass


def _wait_disconnected(timeout: float = DETACH_TIMEOUT) -> bool:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    path, _, _ = find_runtime_chestnut()
    if path is None:
      return True
    time.sleep(0.1)
  return False


def safe_eject() -> bool:
  """Exclusively claim Chestnut, then power it down when VBUS is controllable.

  Returns whether VBUS was switched off. On externally powered C3XL adapters a
  False return means the bridge is idle and safe for the user to unplug. Do not
  write the device's sysfs ``remove`` node: an ASM2464 that remains externally
  powered may not re-enumerate after that host-only detach until its power is
  physically cycled.
  """
  path, _, _ = find_runtime_chestnut()
  if path is None:
    raise RuntimeError("eGPU is not connected")

  # Claiming the interface is the safety barrier. It fails with EBUSY if modeld,
  # a compiler, or a diagnostic process is still using the bridge.
  fd = claim_interface(path)
  os.close(fd)

  vbus = Path(VBUS_PATH)
  powered_off = vbus.exists()
  if powered_off:
    vbus.write_text("0\n")
    if not _wait_disconnected():
      raise DetachPendingError("eGPU detach is still pending")
  return powered_off


def main() -> int:
  parser = argparse.ArgumentParser(description="safely detach the Chestnut eGPU")
  parser.parse_args()
  try:
    powered_off = safe_eject()
  except DetachPendingError as e:
    print(e, file=sys.stderr, flush=True)
    return DETACH_PENDING_EXIT_CODE
  print("powered-off" if powered_off else "safe-to-unplug", flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
