"""dp - eGPU presence detection via sysfs. Cheap enough to call once per second."""
from pathlib import Path

from dragonpilot.egpu.constants import EGPU_USB_IDS, EGPU_ROM_USB_IDS

USB_DEVICES_PATH = Path("/sys/bus/usb/devices")


def is_egpu_usb_id(vendor_id: int, product_id: int) -> bool:
  return (vendor_id, product_id) in EGPU_USB_IDS


def _ids():
  try:
    devices = list(USB_DEVICES_PATH.glob("*"))
  except OSError:
    return
  for d in devices:
    try:
      yield int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16)
    except (OSError, ValueError):
      pass


def egpu_present() -> bool:
  """a usable eGPU: official chestnut (ADD1/3801:0001) or UT3G V1.6 Dual (ADD1:0002)"""
  return any(is_egpu_usb_id(*i) for i in _ids())


def egpu_rom_present() -> bool:
  """an ASM2464 on the bus with factory / ROM firmware (174C:2464/2463): seen, but tinygrad cannot open it"""
  return any(i in EGPU_ROM_USB_IDS for i in _ids())
