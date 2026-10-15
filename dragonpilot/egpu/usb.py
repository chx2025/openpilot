"""dp - eGPU presence detection via sysfs. Cheap enough to call once per second."""
from pathlib import Path

from dragonpilot.egpu.constants import EGPU_USB_IDS

USB_DEVICES_PATH = Path("/sys/bus/usb/devices")


def is_egpu_usb_id(vendor_id: int, product_id: int) -> bool:
  return (vendor_id, product_id) in EGPU_USB_IDS


def egpu_present() -> bool:
  try:
    devices = list(USB_DEVICES_PATH.glob("*"))
  except OSError:
    return False
  for d in devices:
    try:
      if is_egpu_usb_id(int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16)):
        return True
    except (OSError, ValueError):
      pass
  return False
