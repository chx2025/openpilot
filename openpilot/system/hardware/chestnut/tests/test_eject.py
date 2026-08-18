import os

import pytest

from openpilot.system.hardware.chestnut import eject
from openpilot.system.hardware.chestnut.ejector import ChestnutEjector


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
  monkeypatch.setattr(eject, "find_chestnut", lambda: (str(path), ("3801", "0001"), "custom test-CLEAN"))
  monkeypatch.setattr(eject, "_wait_disconnected", lambda timeout=eject.DETACH_TIMEOUT: True)
  return path


def test_safe_eject_removes_host_device(chestnut_path, monkeypatch):
  read_fd, write_fd = os.pipe()
  os.close(write_fd)
  monkeypatch.setattr(eject, "claim_interface", lambda path: read_fd)

  assert not eject.safe_eject()
  assert (chestnut_path / "remove").read_text() == "1\n"
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


def test_ejector_rejects_onroad_request():
  params = FakeParams({"UsbGpuEjectRequest": True})
  ejector = ChestnutEjector(params)

  ejector.update(False, [])

  assert params.get("UsbGpuEjectStatus") == "error"
  assert "offroad" in params.get("UsbGpuEjectError")
  assert not params.get_bool("UsbGpuEjectRequest")


def test_safe_status_survives_stale_usb_snapshot():
  params = FakeParams({"UsbGpuEjectStatus": "safe"})
  ejector = ChestnutEjector(params)
  present = [{"vendorId": 0x3801, "productId": 0x0001}]

  ejector.update(True, present)
  assert params.get("UsbGpuEjectStatus") == "safe"

  ejector.update(True, [])
  ejector.update(True, present)
  assert params.get("UsbGpuEjectStatus") is None
