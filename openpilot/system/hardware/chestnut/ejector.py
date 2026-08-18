import subprocess
import sys
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.chestnut.eject import DETACH_PENDING_EXIT_CODE


class ChestnutEjector:
  """Owns the asynchronous, offroad-only Chestnut detach request."""
  def __init__(self, params: Params):
    self.params = params
    self.thread: threading.Thread | None = None
    self.detached_seen = False
    self.detach_pending = False

  def eject(self) -> None:
    ret = subprocess.run(["sudo", sys.executable, "-m", "openpilot.system.hardware.chestnut.eject"], cwd=BASEDIR,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    output = ret.stdout.strip()
    self.detach_pending = ret.returncode == DETACH_PENDING_EXIT_CODE
    if ret.returncode == 0:
      self.params.put("UsbGpuEjectStatus", "safe")
      self.params.remove("UsbGpuEjectError")
    else:
      self.params.put("UsbGpuEjectStatus", "error")
      self.params.put("UsbGpuEjectError", output[-300:] or f"exit {ret.returncode}")
    cloudlog.event("chestnut eject done", returncode=ret.returncode, output=output[-1000:], error=ret.returncode != 0)

  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    detected = any((d["vendorId"], d["productId"]) in CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS for d in usb_state)
    ready = any((d["vendorId"], d["productId"]) in CHESTNUT_USB_IDS and d.get("speedMbps", 0) == 5000 for d in usb_state)
    status = self.params.get("UsbGpuEjectStatus")
    if self.detach_pending and status == "error" and not detected:
      self.params.put("UsbGpuEjectStatus", "safe")
      self.params.remove("UsbGpuEjectError")
      self.detach_pending = False
      self.detached_seen = True
      status = "safe"

    if status == "safe" and not detected:
      self.detached_seen = True
    elif ready and status == "safe" and self.detached_seen:
      self.params.remove("UsbGpuEjectStatus")
      self.detached_seen = False

    if not self.params.get_bool("UsbGpuEjectRequest"):
      return
    self.params.remove("UsbGpuEjectRequest")

    if not offroad:
      self.detach_pending = False
      self.params.put("UsbGpuEjectStatus", "error")
      self.params.put("UsbGpuEjectError", "eGPU can only be ejected while offroad")
      return
    if self.thread is not None and self.thread.is_alive():
      return

    self.detach_pending = False
    self.params.put("UsbGpuEjectStatus", "ejecting")
    self.params.remove("UsbGpuEjectError")
    self.thread = threading.Thread(target=self.eject, daemon=True)
    self.thread.start()
