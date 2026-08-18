from openpilot.cereal import log
from openpilot.sunnypilot.hardware.profile import HardwareProfile
from openpilot.sunnypilot.system.sound_profile import get_sound_overrides


class FakeParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get_bool(self, key):
    assert key == "CustomAlertSounds"
    return self.enabled


def test_custom_sounds_are_c3xl_profile_scoped(monkeypatch):
  monkeypatch.setattr("openpilot.sunnypilot.system.sound_profile.get_hardware_profile",
                      lambda: HardwareProfile.STANDARD)
  assert get_sound_overrides(FakeParams(True)) == {}

  monkeypatch.setattr("openpilot.sunnypilot.system.sound_profile.get_hardware_profile",
                      lambda: HardwareProfile.C3XL)
  assert get_sound_overrides(FakeParams(False)) == {}
  sounds = get_sound_overrides(FakeParams(True))
  assert sounds[log.SelfdriveState.AudibleAlert.engage][0] == "engage_tizi.wav"
  assert sounds[log.SelfdriveState.AudibleAlert.disengage][0] == "disengage_tizi.wav"
