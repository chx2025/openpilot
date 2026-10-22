"""dp - eGPU: the GPU's temperature and power, for the indicator under the onroad eGPU icon.

Why this is a kernel-sysfs read and not a device query: this dock's GPU has no driver. tinygrad
reaches it through the ASM2464 controller directly (tinygrad/runtime/support/usb.py builds the
PCIe config space and BARs in userspace), so the card never shows up on the PCI bus - there is no
amdgpu binding, no /dev/kfd and no hwmon on the units we run. The only numbers the kernel could
ever hand out are therefore the ones below, and they exist only if something does bind amdgpu
(a future kernel, or a different dock that tunnels PCIe as a real bus).

temp1_input is millidegrees, power1_average is microwatts. Both are None when the files are not
there, which is the normal case here; the caller falls back to the USB link speed.

The alternative - the SMU metrics table - is not readable from this process: it needs the device
open, and modeld owns it while the big model drives. If that is ever added, it belongs in
modeld_egpu.py, publishing a param the UI reads.
"""
from pathlib import Path

DRM_PATH = Path("/sys/class/drm")
AMD_VENDOR = "0x1002"


def _read(path: Path, divisor: float) -> float | None:
  try:
    return float(path.read_text()) / divisor
  except (OSError, ValueError):
    return None


def temp_power() -> tuple[float, float] | None:
  """(celsius, watts) of the AMD card, or None when no driver exposes it"""
  temp = power = None
  try:
    cards = list(DRM_PATH.glob("card[0-9]*"))
  except OSError:
    return None

  for card in cards:
    dev = card / "device"
    try:
      if (dev / "vendor").read_text().strip().lower() != AMD_VENDOR:
        continue
    except OSError:
      continue
    for hwmon in dev.glob("hwmon/hwmon*"):
      temp = temp if temp is not None else _read(hwmon / "temp1_input", 1000.0)
      power = power if power is not None else _read(hwmon / "power1_average", 1e6)

  if temp is None or power is None:
    return None
  return temp, power
