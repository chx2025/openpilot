"""C3XL-specific sound selection kept outside upstream soundd."""
from __future__ import annotations

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile


def get_sound_overrides(params: Params | None = None) -> dict[int, tuple[str, int | None, float]]:
  params = params or Params()
  if get_hardware_profile() != HardwareProfile.C3XL or not params.get_bool("CustomAlertSounds"):
    return {}

  alert = log.SelfdriveState.AudibleAlert
  return {
    alert.engage: ("engage_tizi.wav", 1, 1.0),
    alert.disengage: ("disengage_tizi.wav", 1, 1.0),
  }
