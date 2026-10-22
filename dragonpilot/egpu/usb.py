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


def egpu_link_speed() -> float | None:
  """The dock's negotiated USB rate in Mbps (5000 for the 5 Gbps this tunnel wants), None when the
  device is absent or speed cannot be read. A drop to 480 (high speed) still enumerates and still
  drives, just starved - that is what the indicator paints red."""
  try:
    devices = list(USB_DEVICES_PATH.glob("*"))
  except OSError:
    return None
  for d in devices:
    try:
      if not is_egpu_usb_id(int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16)):
        continue
      return float((d / "speed").read_text())
    except (OSError, ValueError):
      continue
  return None


def egpu_rom_present() -> bool:
  """an ASM2464 on the bus with factory / ROM firmware (174C:2464/2463): seen, but tinygrad cannot open it"""
  return any(i in EGPU_ROM_USB_IDS for i in _ids())


# ---------------------------------------------------------------------------------------------
# PCIe link (dp - egpu v5.2). The ASM2464 enumerates on USB as soon as the USB-C side is up; the GPU behind it is
# only usable once its PCIe link reaches L0. zoompilot (system/hardware/chestnut/flash.py link_up, used by its
# SConscript before every chestnut build, "chestnut can enumerate before its PCIe link is up due to varying 12V
# power behavior across cars") waits up to 10 s for it. tinygrad itself (support/usb.py) reads the same LTSSM
# register once, powers PCIe on once (vendor request 0xF3, value 1) and raises if it is still not L0, with no
# wait in between. This does the same two requests, retried, before tinygrad opens the device: nothing that
# tinygrad does not already send on every open, and nothing persistent.
# ---------------------------------------------------------------------------------------------
import ctypes  # noqa: E402
import fcntl  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402

USBDEVFS_CONTROL = 0xC0185500
LTSSM_ADDR = 0xB450
LTSSM_L0 = 0x78


class _Ctrl(ctypes.Structure):
  _fields_ = [("request_type", ctypes.c_uint8), ("request", ctypes.c_uint8),
              ("value", ctypes.c_uint16), ("index", ctypes.c_uint16),
              ("length", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
              ("data", ctypes.c_void_p)]


def _egpu_dev_node() -> str | None:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      if is_egpu_usb_id(int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16)):
        return f"/dev/bus/usb/{int((d / 'busnum').read_text()):03d}/{int((d / 'devnum').read_text()):03d}"
    except (OSError, ValueError):
      pass
  return None


def pcie_ltssm(power_on: bool = True) -> int | None:
  """The LTSSM state byte (0x78 = L0, usable), or None when the eGPU cannot be asked (absent, busy, no access)."""
  node = _egpu_dev_node()
  if node is None:
    return None
  try:
    fd = os.open(node, os.O_RDWR)
  except OSError:
    return None
  try:
    if power_on:
      fcntl.ioctl(fd, USBDEVFS_CONTROL, _Ctrl(0x40, 0xF3, 1, 0, 0, 2000, None))
    buf = (ctypes.c_ubyte * 1)()
    fcntl.ioctl(fd, USBDEVFS_CONTROL, _Ctrl(0xC0, 0xE4, LTSSM_ADDR, 0, 1, 1000, ctypes.cast(buf, ctypes.c_void_p)))
    return int(buf[0])
  except OSError:
    return None
  finally:
    os.close(fd)


def wait_pcie_link(timeout: float = 10.0, should_stop=None) -> tuple[bool, int | None]:
  """(up, last LTSSM). Up when L0 is read; also returned as up when the state cannot be read at all (tinygrad then
  makes its own single check and raises its own error), so this never blocks a device it cannot talk to."""
  deadline = time.monotonic() + timeout
  state = None
  while True:
    state = pcie_ltssm()
    if state is None or state == LTSSM_L0:
      return True, state
    if time.monotonic() >= deadline or (should_stop is not None and should_stop()):
      return False, state
    time.sleep(1.0)
