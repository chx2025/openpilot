import pytest
from pathlib import Path
from panda import Panda

from openpilot.common.hardware.comma.hardware import HardwareComma
from openpilot.sunnypilot.hardware.panda import InternalPanda
from openpilot.sunnypilot.hardware.profile import (
  HardwareProfile, allows_automatic_power_down, get_hardware_profile, has_amplifier, has_driver_camera, power_down_requested,
  model_compile_cpu, resolve_internal_panda_type,
)


def test_repository_without_device_override_defaults_standard() -> None:
  assert get_hardware_profile() == HardwareProfile.STANDARD


def test_device_profile_file_enables_c3xl(tmp_path, monkeypatch) -> None:
  from openpilot.sunnypilot.hardware import profile

  profile_file = tmp_path / "hardware_profile"
  profile_file.write_text("c3xl\n")
  monkeypatch.setattr(profile, "HARDWARE_PROFILE_FILE", profile_file)

  assert get_hardware_profile() == HardwareProfile.C3XL


def test_native_build_uses_device_local_profile() -> None:
  sconstruct = (Path(__file__).parents[4] / "SConstruct").read_text()

  assert '"/data/hardware_profile"' in sconstruct
  assert "SUNNYPILOT_HARDWARE_PROFILE" in sconstruct
  assert "Dir('#').abspath, 'hardware_profile'" not in sconstruct


def test_explicit_standard_profile() -> None:
  assert get_hardware_profile("standard") == HardwareProfile.STANDARD


def test_driver_camera_capability_is_profile_scoped() -> None:
  assert has_driver_camera(HardwareProfile.STANDARD)
  assert not has_driver_camera(HardwareProfile.C3XL)


def test_amplifier_capability_is_profile_scoped() -> None:
  assert has_amplifier(HardwareProfile.STANDARD)


def test_model_compile_cpu_never_exceeds_available_hardware() -> None:
  assert model_compile_cpu(8) == 7
  assert model_compile_cpu(4) == 3
  assert model_compile_cpu(1) == 0
  assert not has_amplifier(HardwareProfile.C3XL)


def test_automatic_power_down_is_disabled_only_for_c3xl() -> None:
  assert allows_automatic_power_down(HardwareProfile.STANDARD)
  assert not allows_automatic_power_down(HardwareProfile.C3XL)


def test_c3xl_ignores_automatic_power_down_but_keeps_manual_force() -> None:
  assert not power_down_requested(automatic=True, manual=False, profile=HardwareProfile.C3XL)
  assert power_down_requested(automatic=False, manual=True, profile=HardwareProfile.C3XL)
  assert power_down_requested(automatic=True, manual=False, profile=HardwareProfile.STANDARD)


def test_c3xl_tici_does_not_probe_absent_amplifier(tmp_path, monkeypatch) -> None:
  from openpilot.sunnypilot.hardware import profile

  profile_file = tmp_path / "hardware_profile"
  profile_file.write_text("c3xl\n")
  monkeypatch.setattr(profile, "HARDWARE_PROFILE_FILE", profile_file)
  hardware = HardwareComma()
  monkeypatch.setattr(hardware, "get_device_type", lambda: "tici")
  assert hardware.amplifier is None


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
