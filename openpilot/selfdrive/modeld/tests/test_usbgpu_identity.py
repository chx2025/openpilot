from pathlib import Path

from openpilot.selfdrive.modeld import helpers


def make_usb(tmp_path: Path, vid: str, pid: str, manufacturer: str, product: str) -> Path:
  device = tmp_path / "4-1"
  device.mkdir()
  for name, value in (("idVendor", vid), ("idProduct", pid), ("manufacturer", manufacturer), ("product", product)):
    (device / name).write_text(value)
  return device


def test_usbgpu_present_accepts_official_and_dual(monkeypatch, tmp_path):
  monkeypatch.setattr(helpers, "USB_DEVICES_PATH", tmp_path)
  make_usb(tmp_path, "add1", "0002", "tiny", "custom d1377a01-UT3G-DUAL")
  assert helpers.usbgpu_present()


def test_usbgpu_present_rejects_dirty_dual(monkeypatch, tmp_path):
  monkeypatch.setattr(helpers, "USB_DEVICES_PATH", tmp_path)
  make_usb(tmp_path, "add1", "0002", "tiny", "custom d1377a01-UT3G-DUAL-DIRTY")
  assert not helpers.usbgpu_present()


def test_usbgpu_present_rejects_factory(monkeypatch, tmp_path):
  monkeypatch.setattr(helpers, "USB_DEVICES_PATH", tmp_path)
  make_usb(tmp_path, "2065", "2463", "ASMedia", "ASM246X series")
  assert not helpers.usbgpu_present()
