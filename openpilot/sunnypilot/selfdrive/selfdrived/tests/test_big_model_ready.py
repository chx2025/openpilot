import pytest

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.ui.onroad.alert_localizer import localize_alert_text
import openpilot.sunnypilot.selfdrive.selfdrived.events as events_sp_module
from openpilot.sunnypilot.hardware.profile import HardwareProfile
from openpilot.sunnypilot.selfdrive.selfdrived.events import ET, EventsSP


class LoadingParams:
  def __init__(self):
    self.loading = False

  def get_bool(self, key):
    assert key == "ChestnutLoading"
    return self.loading

  def get(self, key):
    assert key == "ChestnutActive"
    raise StopAfterBigModelLoading


class StopAfterBigModelLoading(Exception):
  pass


class LateralControlState:
  @staticmethod
  def which():
    return "pidState"


class ControlsState:
  lateralControlState = LateralControlState()


class SelfdriveInputs:
  def __getitem__(self, key):
    assert key == "controlsState"
    return ControlsState()


def update_through_big_model_loading(selfdrive):
  with pytest.raises(StopAfterBigModelLoading):
    selfdrive.update_events(None)


def test_big_model_ready_event_serializes_as_permanent():
  event_name = custom.OnroadEventSP.EventName.bigModelReady
  events = EventsSP()

  events.add(event_name)

  [event] = events.to_msg()
  assert event.name == event_name
  assert event.permanent


def test_big_model_ready_fires_once_on_loading_completion():
  selfdrive = SelfdriveD.__new__(SelfdriveD)
  selfdrive.params = LoadingParams()
  selfdrive.events = Events()
  selfdrive.events_sp = EventsSP()
  selfdrive.sm = SelfdriveInputs()
  selfdrive.big_model_loading = False
  selfdrive.big_model_ready_t = 0.
  ready = custom.OnroadEventSP.EventName.bigModelReady

  update_through_big_model_loading(selfdrive)
  assert not selfdrive.events_sp.has(ready)

  selfdrive.params.loading = True
  update_through_big_model_loading(selfdrive)
  assert not selfdrive.events_sp.has(ready)

  selfdrive.params.loading = False
  update_through_big_model_loading(selfdrive)
  assert selfdrive.events_sp.has(ready)

  update_through_big_model_loading(selfdrive)
  assert not selfdrive.events_sp.has(ready)


@pytest.mark.parametrize("profile, expected_sound", (
  (HardwareProfile.STANDARD, car.CarControl.HUDControl.AudibleAlert.prompt),
  (HardwareProfile.C3XL, car.CarControl.HUDControl.AudibleAlert.promptRepeat),
))
def test_big_model_ready_uses_available_audio_output(monkeypatch, profile, expected_sound):
  monkeypatch.setattr(events_sp_module, "get_hardware_profile", lambda: profile)
  events = EventsSP()
  events.add(custom.OnroadEventSP.EventName.bigModelReady)

  [alert] = events.create_alerts([ET.PERMANENT], [None] * 6)

  assert alert.alert_text_1 == "Big Model Ready"
  assert alert.audible_alert == expected_sound


def test_big_model_ready_has_simplified_chinese_text():
  assert localize_alert_text(
    "bigModelReady/permanent", "Big Model Ready", "", "zh-CHS",
  ) == ("大模型已就绪", "")
