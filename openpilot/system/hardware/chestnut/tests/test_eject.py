import os
import json
from types import SimpleNamespace

import pytest

from openpilot.system.hardware.chestnut import eject
from openpilot.system.hardware.chestnut.ejector import ChestnutEjector
from openpilot.common.basedir import BASEDIR


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


@pytest.fixture
def chestnut_path(tmp_path, monkeypatch):
  path = tmp_path / "4-1"
  path.mkdir()
  (path / "remove").write_text("")
  monkeypatch.setattr(eject, "VBUS_PATH", str(tmp_path / "missing-vbus"))
  monkeypatch.setattr(eject, "find_runtime_chestnut", lambda: (str(path), ("3801", "0001"), "custom test-CLEAN"))
  monkeypatch.setattr(eject, "_wait_disconnected", lambda timeout=eject.DETACH_TIMEOUT: True)
  return path


def test_safe_eject_leaves_externally_powered_device_for_physical_unplug(chestnut_path, monkeypatch):
  read_fd, write_fd = os.pipe()
  os.close(write_fd)
  monkeypatch.setattr(eject, "claim_interface", lambda path: read_fd)

  assert not eject.safe_eject()
  assert (chestnut_path / "remove").read_text() == ""
  with pytest.raises(OSError):
    os.fstat(read_fd)


def test_safe_eject_does_not_detach_when_claim_fails(chestnut_path, monkeypatch):
  def busy(_):
    raise RuntimeError("chestnut is in use")

  monkeypatch.setattr(eject, "claim_interface", busy)
  with pytest.raises(RuntimeError, match="in use"):
    eject.safe_eject()
  assert (chestnut_path / "remove").read_text() == ""


def test_safe_eject_requires_connected_device(monkeypatch):
  monkeypatch.setattr(eject, "find_chestnut", lambda: (None, None, None))
  with pytest.raises(RuntimeError, match="not connected"):
    eject.safe_eject()


def test_wait_disconnected_allows_slow_c3xl_teardown(monkeypatch):
  clock = [0.0]

  monkeypatch.setattr(eject.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(eject.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
  monkeypatch.setattr(eject, "find_chestnut", lambda: (("/sys/4-1", None, None) if clock[0] < 8.0 else (None, None, None)))

  assert eject._wait_disconnected()


def test_ejector_rejects_onroad_request():
  params = FakeParams({"UsbGpuEjectRequest": True})
  ejector = ChestnutEjector(params)

  ejector.update(False, [])

  assert params.get("UsbGpuEjectStatus") == "error"
  assert "offroad" in params.get("UsbGpuEjectError")
  assert not params.get_bool("UsbGpuEjectRequest")


def test_ejector_runs_project_f3_poweroff_script(monkeypatch):
  params = FakeParams()
  captured = {}
  report = {
    "schema": "ut3g-safe-f3-poweroff-v1",
    "state": "f3-powered-off",
    "f3_writes": 1,
    "persistent_writes": 0,
    "safe_to_cut_external_power": True,
  }

  def run(command, **kwargs):
    captured["command"] = command
    captured["kwargs"] = kwargs
    return SimpleNamespace(returncode=0, stdout=json.dumps(report))

  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", run)
  ChestnutEjector(params).eject()

  assert captured["command"] == [
    "sudo", "env", f"PYTHONPATH={BASEDIR}/tinygrad_repo", "/usr/local/venv/bin/python", "-u",
    f"{BASEDIR}/tools/ut3g_safe_f3_poweroff.py",
  ]
  assert params.get("UsbGpuEjectStatus") == "safe"
  assert params.get("UsbGpuEjectError") is None


def test_ejector_rejects_success_exit_without_safe_f3_report(monkeypatch):
  params = FakeParams()
  report = {
    "schema": "ut3g-safe-f3-poweroff-v1",
    "state": "f3-powered-off",
    "f3_writes": 1,
    "persistent_writes": 0,
    "safe_to_cut_external_power": False,
  }
  ret = SimpleNamespace(returncode=0, stdout=json.dumps(report))
  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", lambda *args, **kwargs: ret)

  ChestnutEjector(params).eject()

  assert params.get("UsbGpuEjectStatus") == "error"
  assert "safe_to_cut_external_power" in params.get("UsbGpuEjectError")


def test_ejector_rejects_report_with_persistent_writes(monkeypatch):
  params = FakeParams()
  report = {
    "schema": "ut3g-safe-f3-poweroff-v1",
    "state": "f3-powered-off",
    "f3_writes": 1,
    "persistent_writes": 1,
    "safe_to_cut_external_power": True,
  }
  ret = SimpleNamespace(returncode=0, stdout=json.dumps(report))
  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", lambda *args, **kwargs: ret)

  ChestnutEjector(params).eject()

  assert params.get("UsbGpuEjectStatus") == "error"
  assert "persistent_writes" in params.get("UsbGpuEjectError")


def test_ejector_rejects_unknown_poweroff_state(monkeypatch):
  params = FakeParams()
  report = {
    "schema": "ut3g-safe-f3-poweroff-v1",
    "state": "unknown",
    "f3_writes": 0,
    "persistent_writes": 0,
    "safe_to_cut_external_power": True,
  }
  ret = SimpleNamespace(returncode=0, stdout=json.dumps(report))
  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", lambda *args, **kwargs: ret)

  ChestnutEjector(params).eject()

  assert params.get("UsbGpuEjectStatus") == "error"
  assert "state" in params.get("UsbGpuEjectError")


def test_safe_status_clears_only_after_disconnect_and_5gbps_return():
  params = FakeParams({"UsbGpuEjectStatus": "safe"})
  ejector = ChestnutEjector(params)
  present = [{"vendorId": 0x3801, "productId": 0x0001, "speedMbps": 5000}]

  ejector.update(True, present)
  assert params.get("UsbGpuEjectStatus") == "safe"

  ejector.update(True, [])
  ejector.update(True, [{**present[0], "speedMbps": 480}])
  assert params.get("UsbGpuEjectStatus") == "safe"

  ejector.update(True, present)
  assert params.get("UsbGpuEjectStatus") is None


def test_disconnect_never_overrides_missing_safe_f3_report(monkeypatch):
  params = FakeParams()
  ejector = ChestnutEjector(params)
  ret = SimpleNamespace(returncode=eject.DETACH_PENDING_EXIT_CODE, stdout="eGPU detach is still pending")
  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", lambda *args, **kwargs: ret)

  ejector.eject()
  assert params.get("UsbGpuEjectStatus") == "error"

  ejector.update(True, [])
  assert params.get("UsbGpuEjectStatus") == "error"
  assert params.get("UsbGpuEjectError") == "eGPU detach is still pending"


def test_non_pending_error_does_not_converge_to_safe(monkeypatch):
  params = FakeParams()
  ejector = ChestnutEjector(params)
  ret = SimpleNamespace(returncode=1, stdout="permission denied")
  monkeypatch.setattr("openpilot.system.hardware.chestnut.ejector.subprocess.run", lambda *args, **kwargs: ret)

  ejector.eject()
  ejector.update(True, [])

  assert params.get("UsbGpuEjectStatus") == "error"
  assert params.get("UsbGpuEjectError") == "permission denied"


def test_cli_marks_slow_detach_as_temporary_failure(monkeypatch):
  monkeypatch.setattr(eject, "safe_eject", lambda: (_ for _ in ()).throw(eject.DetachPendingError("pending")))
  monkeypatch.setattr(eject.sys, "argv", ["eject.py"])

  assert eject.main() == eject.DETACH_PENDING_EXIT_CODE
