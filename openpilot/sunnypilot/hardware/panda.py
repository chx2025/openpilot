"""Profile-scoped Adapter for the internal Panda Python flashing path."""

from panda import Panda

from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile, resolve_internal_panda_type


class InternalPanda(Panda):
  """Panda whose effective type is resolved without changing global discovery."""

  def __init__(self, *args, hardware_profile: HardwareProfile | None = None, **kwargs):
    self.hardware_profile = hardware_profile or get_hardware_profile()
    self.last_raw_hw_type: bytes | None = None
    super().__init__(*args, **kwargs)

  def get_raw_type(self) -> bytes:
    raw_type = super().get_type()
    self.last_raw_hw_type = raw_type
    return raw_type

  def get_type(self) -> bytes:
    return resolve_internal_panda_type(self.get_raw_type(), self.hardware_profile)
