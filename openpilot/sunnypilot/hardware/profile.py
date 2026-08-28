from enum import StrEnum
from pathlib import Path


HARDWARE_PROFILE_FILE = Path(__file__).parents[3] / "hardware_profile"


class HardwareProfile(StrEnum):
  STANDARD = "standard"
  C3XL = "c3xl"


PANDA_TYPE_UNKNOWN = b"\x00"
PANDA_TYPE_TRES = b"\x09"


def get_hardware_profile(value: str | None = None) -> HardwareProfile:
  if value is not None:
    raw_value = value
  elif HARDWARE_PROFILE_FILE.is_file():
    raw_value = HARDWARE_PROFILE_FILE.read_text().strip()
  else:
    raw_value = HardwareProfile.STANDARD
  return HardwareProfile(raw_value)


def has_driver_camera(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def has_amplifier(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def allows_automatic_power_down(profile: HardwareProfile | None = None) -> bool:
  return (profile or get_hardware_profile()) != HardwareProfile.C3XL


def power_down_requested(*, automatic: bool, manual: bool,
                         profile: HardwareProfile | None = None) -> bool:
  return manual or (automatic and allows_automatic_power_down(profile))


def model_compile_cpu(cpu_count: int) -> int:
  """Return the upstream isolated CPU when present, otherwise the highest available CPU."""
  return min(7, max(0, cpu_count - 1))


def resolve_internal_panda_type(raw_type: bytes, profile: HardwareProfile | None = None) -> bytes:
  """Resolve the effective type for an already-identified internal Panda."""
  selected_profile = profile or get_hardware_profile()
  if selected_profile != HardwareProfile.C3XL:
    return raw_type
  if raw_type in (PANDA_TYPE_UNKNOWN, PANDA_TYPE_TRES):
    return PANDA_TYPE_TRES
  raise ValueError(f"C3XL internal SPI Panda reported unexpected raw type {raw_type.hex()!r}")
