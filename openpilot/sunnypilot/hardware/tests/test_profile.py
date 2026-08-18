import pytest
from panda import Panda

from openpilot.sunnypilot.hardware.panda import InternalPanda
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile, has_driver_camera, resolve_internal_panda_type


def test_repository_profile_is_c3xl() -> None:
  assert get_hardware_profile() == HardwareProfile.C3XL


def test_explicit_standard_profile() -> None:
  assert get_hardware_profile("standard") == HardwareProfile.STANDARD


def test_driver_camera_capability_is_profile_scoped() -> None:
  assert has_driver_camera(HardwareProfile.STANDARD)
  assert not has_driver_camera(HardwareProfile.C3XL)


def test_unknown_profile_fails_closed() -> None:
  with pytest.raises(ValueError):
    get_hardware_profile("unknown")


def test_standard_profile_preserves_raw_panda_type() -> None:
  assert resolve_internal_panda_type(b"\x00", HardwareProfile.STANDARD) == b"\x00"
  assert resolve_internal_panda_type(b"\x07", HardwareProfile.STANDARD) == b"\x07"


def test_c3xl_profile_only_resolves_known_internal_types() -> None:
  assert resolve_internal_panda_type(b"\x00", HardwareProfile.C3XL) == b"\x09"
  assert resolve_internal_panda_type(b"\x09", HardwareProfile.C3XL) == b"\x09"
  with pytest.raises(ValueError):
    resolve_internal_panda_type(b"\x07", HardwareProfile.C3XL)


def test_internal_panda_adapter_keeps_raw_type_observable(monkeypatch) -> None:
  monkeypatch.setattr(Panda, "get_type", lambda _panda: b"\x00")
  panda = InternalPanda.__new__(InternalPanda)
  panda.hardware_profile = HardwareProfile.C3XL
  panda.last_raw_hw_type = None

  assert panda.get_type() == b"\x09"
  assert panda.last_raw_hw_type == b"\x00"
