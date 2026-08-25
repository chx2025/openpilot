from openpilot.common.hardware.usb import (
  CHESTNUT_FW_VERSION,
  chestnut_official_flash_mismatch,
  is_chestnut_runtime_device,
)


def device(vid, pid, product, manufacturer="tiny"):
  return {"vendorId": vid, "productId": pid, "manufacturer": manufacturer, "product": product}


def test_runtime_and_official_flash_ownership_are_separate():
  official = device(0xADD1, 0x0001, f"custom {CHESTNUT_FW_VERSION}-CLEAN")
  assert is_chestnut_runtime_device(official)
  assert not chestnut_official_flash_mismatch([official])

  old_official = device(0x3801, 0x0001, "custom deadbeef-CLEAN")
  assert not is_chestnut_runtime_device(old_official)
  assert chestnut_official_flash_mismatch([old_official])

  dual = device(0xADD1, 0x0002, "custom d1377a01-UT3G-DUAL")
  assert is_chestnut_runtime_device(dual)
  assert not chestnut_official_flash_mismatch([dual])


def test_invalid_dual_is_neither_runnable_nor_owned_by_official_flasher():
  invalid = (
    device(0xADD1, 0x0002, "custom d1377a01-UT3G-DUAL-DIRTY"),
    device(0xADD1, 0x0002, "custom d1377a01-UT3G-DUAL", manufacturer="ASMedia"),
    device(0xADD1, 0x0002, "custom D1377A01-UT3G-DUAL"),
  )
  for entry in invalid:
    assert not is_chestnut_runtime_device(entry)
    assert not chestnut_official_flash_mismatch([entry])


def test_factory_is_ignored_and_official_rom_remains_owned():
  factory = device(0x2065, 0x2463, "ASM246X series", manufacturer="ASMedia")
  assert not is_chestnut_runtime_device(factory)
  assert not chestnut_official_flash_mismatch([factory])

  rom = device(0x174C, 0x2463, "USB 3.2 PCIe TinyEnclosure", manufacturer="ASMedia")
  assert not is_chestnut_runtime_device(rom)
  assert chestnut_official_flash_mismatch([rom])
