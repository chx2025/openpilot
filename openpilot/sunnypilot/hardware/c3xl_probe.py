#!/usr/bin/env python3
import json
from pathlib import Path
import subprocess
from typing import Any

from panda import Panda

from openpilot.sunnypilot.hardware.profile import get_hardware_profile


def read_text(path: str) -> str | None:
  try:
    return Path(path).read_text().strip("\x00\n")
  except OSError:
    return None


def run_output(command: list[str]) -> str | None:
  try:
    return subprocess.check_output(command, encoding="utf-8", stderr=subprocess.STDOUT).strip()
  except (OSError, subprocess.CalledProcessError):
    return None


def inspect_panda(serial: str) -> dict[str, Any]:
  result: dict[str, Any] = {"serial": serial}
  try:
    with Panda(serial) as panda:
      raw_type = panda.get_type()
      result.update({
        "transport": "spi",
        "bootstub": panda.bootstub,
        "raw_type_hex": raw_type.hex(),
        "raw_type_int": raw_type[0] if raw_type else None,
        "health_packet_version": panda.health_version,
        "can_packet_version": panda.can_version,
        "version": panda.get_version(),
      })
      if not panda.bootstub:
        result["signature"] = panda.get_signature().hex()
        result["health"] = panda.health()
  except Exception as exc:
    result["error"] = f"{type(exc).__name__}: {exc}"
  return result


def collect_probe() -> dict[str, Any]:
  try:
    spi_serials = Panda.spi_list()
    spi_error = None
  except Exception as exc:
    spi_serials = []
    spi_error = f"{type(exc).__name__}: {exc}"

  return {
    "hardware_profile": get_hardware_profile().value,
    "device_tree_model": read_text("/sys/firmware/devicetree/base/model"),
    "agnos_version": read_text("/VERSION"),
    "build": read_text("/BUILD"),
    "boot_slot": run_output(["abctl", "--boot_slot"]),
    "spidev0_0_exists": Path("/dev/spidev0.0").exists(),
    "spi_enumeration_error": spi_error,
    "pandas": [inspect_panda(serial) for serial in spi_serials],
  }


if __name__ == "__main__":
  print(json.dumps(collect_probe(), indent=2, sort_keys=True))
