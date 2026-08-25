from pathlib import Path

from openpilot.system.hardware.chestnut import flash


def test_official_flasher_ids_exclude_dual_runtime_id():
  assert ("add1", "0002") not in flash.VID_PIDS
  assert ("add1", "0002") in flash.RUNTIME_VID_PIDS


def make_sysfs_device(tmp_path, product="custom d1377a01-UT3G-DUAL", manufacturer="tiny"):
  path = tmp_path / "4-1"
  path.mkdir()
  for name, value in (("idVendor", "add1"), ("idProduct", "0002"),
                      ("manufacturer", manufacturer), ("product", product)):
    (path / name).write_text(value)
  return path


def test_runtime_finder_sees_dual_while_official_flasher_finder_does_not(monkeypatch, tmp_path):
  path = make_sysfs_device(tmp_path)
  monkeypatch.setattr(flash.glob, "glob", lambda _pattern: [str(path)])
  assert flash.find_chestnut() == (None, None, None)
  assert flash.find_runtime_chestnut() == (str(path), ("add1", "0002"), "custom d1377a01-UT3G-DUAL")


def test_runtime_finder_rejects_dirty_dual(monkeypatch, tmp_path):
  path = make_sysfs_device(tmp_path, product="custom d1377a01-UT3G-DUAL-DIRTY")
  monkeypatch.setattr(flash.glob, "glob", lambda _pattern: [str(path)])
  assert flash.find_runtime_chestnut() == (None, None, None)


def test_hardwared_uses_official_ownership_predicate_not_runtime_union():
  source = (Path(__file__).resolve().parents[2] / "hardwared.py").read_text()
  update_body = source.split("def update(self, offroad: bool, usb_state: list[dict])", 1)[1].split("if not mismatch", 1)[0]
  assert "chestnut_official_flash_mismatch(usb_state)" in update_body
  assert "CHESTNUT_USB_IDS" not in update_body
