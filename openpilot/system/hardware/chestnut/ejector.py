import json
import subprocess
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS, is_chestnut_runtime_device
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


def parse_safe_poweroff_report(output: str) -> dict:
  for line in reversed(output.splitlines()):
    try:
      report = json.loads(line)
    except json.JSONDecodeError:
      continue
    if report.get("schema") != "ut3g-safe-f3-poweroff-v1":
      raise ValueError("unexpected F3 poweroff report schema")
    if report.get("safe_to_cut_external_power") is not True:
      raise ValueError("safe_to_cut_external_power was not confirmed")
    if report.get("persistent_writes") != 0:
      raise ValueError("persistent_writes was not zero")
    expected_f3_writes = {"f3-powered-off": 1, "already-not-l0": 0}.get(report.get("state"))
    if expected_f3_writes is None:
      raise ValueError("unexpected F3 poweroff state")
    if report.get("f3_writes") != expected_f3_writes:
      raise ValueError("F3 write count does not match poweroff state")
    return report
  raise ValueError("F3 poweroff script did not return a JSON report")


class ChestnutEjector:
  """Owns the asynchronous, offroad-only Chestnut detach request."""
  def __init__(self, params: Params):
    self.params = params
    self.thread: threading.Thread | None = None
    self.detached_seen = False

  def eject(self) -> None:
    ret = subprocess.run(["sudo", "env", f"PYTHONPATH={BASEDIR}/tinygrad_repo", "/usr/local/venv/bin/python", "-u",
                          f"{BASEDIR}/tools/ut3g_safe_f3_poweroff.py"], cwd=BASEDIR,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    output = ret.stdout.strip()
    report_error = None
    if ret.returncode == 0:
      try:
        parse_safe_poweroff_report(output)
      except ValueError as exc:
        report_error = str(exc)

    if ret.returncode == 0 and report_error is None:
      self.params.put("UsbGpuEjectStatus", "safe")
      self.params.remove("UsbGpuEjectError")
    else:
      self.params.put("UsbGpuEjectStatus", "error")
      self.params.put("UsbGpuEjectError", report_error or output[-300:] or f"exit {ret.returncode}")
    cloudlog.event("chestnut eject done", returncode=ret.returncode, output=output[-1000:],
                   error=ret.returncode != 0 or report_error is not None)

  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    detected = any((d["vendorId"], d["productId"]) in CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS for d in usb_state)
    ready = any(is_chestnut_runtime_device(d) and d.get("speedMbps", 0) == 5000 for d in usb_state)
    status = self.params.get("UsbGpuEjectStatus")

    if status == "safe" and not detected:
      self.detached_seen = True
    elif ready and status == "safe" and self.detached_seen:
      self.params.remove("UsbGpuEjectStatus")
      self.detached_seen = False

    if not self.params.get_bool("UsbGpuEjectRequest"):
      return
    self.params.remove("UsbGpuEjectRequest")

    if not offroad:
      self.params.put("UsbGpuEjectStatus", "error")
      self.params.put("UsbGpuEjectError", "eGPU can only be ejected while offroad")
      return
    if self.thread is not None and self.thread.is_alive():
      return

    self.params.put("UsbGpuEjectStatus", "ejecting")
    self.params.remove("UsbGpuEjectError")
    self.thread = threading.Thread(target=self.eject, daemon=True)
    self.thread.start()
