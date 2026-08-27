"""Offroad home-button click on the eGPU icon should drive the safe-eject flow."""
from openpilot.selfdrive.ui.sunnypilot.layouts.sidebar import HOME_BTN, SidebarSP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    self.values[key] = value

  def put_bool(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class FakeDeviceState:
  def __init__(self, chestnut_present: bool):
    self.chestnutPresent = chestnut_present


class FakeSM(dict):
  pass


def _inside_home_btn():
  return (HOME_BTN.x + 10, HOME_BTN.y + 10)


def _outside_home_btn():
  return (0, 0)


def _prepare(monkeypatch, *, started=False, chestnut_present=True, params=None):
  monkeypatch.setattr(ui_state, "started", started)
  fake_sm = FakeSM()
  fake_sm["deviceState"] = FakeDeviceState(chestnut_present)
  monkeypatch.setattr(ui_state, "sm", fake_sm)
  monkeypatch.setattr(ui_state, "params", params or FakeParams())
  pushed: list = []
  monkeypatch.setattr(gui_app, "push_widget", pushed.append)
  return pushed


def test_no_egpu_present_is_a_noop(monkeypatch):
  pushed = _prepare(monkeypatch, chestnut_present=False)
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is False
  assert pushed == []


def test_click_outside_home_button_is_ignored(monkeypatch):
  pushed = _prepare(monkeypatch)
  assert SidebarSP()._handle_egpu_click(_outside_home_btn()) is False
  assert pushed == []


def test_onroad_click_defers_to_flag_behavior(monkeypatch):
  pushed = _prepare(monkeypatch, started=True)
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is False
  assert pushed == []


def test_offroad_click_prompts_for_confirmation(monkeypatch):
  pushed = _prepare(monkeypatch)
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is True
  assert len(pushed) == 1
  assert isinstance(pushed[0], ConfirmDialog)


def test_confirming_sets_eject_request(monkeypatch):
  pushed = _prepare(monkeypatch)
  SidebarSP()._handle_egpu_click(_inside_home_btn())
  pushed[0].callback(DialogResult.CONFIRM)
  assert ui_state.params.get_bool("UsbGpuEjectRequest") is True


def test_cancelling_does_not_set_eject_request(monkeypatch):
  pushed = _prepare(monkeypatch)
  SidebarSP()._handle_egpu_click(_inside_home_btn())
  pushed[0].callback(DialogResult.CANCEL)
  assert not ui_state.params.get_bool("UsbGpuEjectRequest")


def test_ejecting_status_shows_alert_not_new_request(monkeypatch):
  pushed = _prepare(monkeypatch, params=FakeParams({"UsbGpuEjectStatus": "ejecting"}))
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is True
  assert not isinstance(pushed[0], ConfirmDialog)
  assert not ui_state.params.get_bool("UsbGpuEjectRequest")


def test_safe_status_shows_alert(monkeypatch):
  pushed = _prepare(monkeypatch, params=FakeParams({"UsbGpuEjectStatus": "safe"}))
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is True
  assert not isinstance(pushed[0], ConfirmDialog)


def test_error_status_surfaces_error_in_retry_dialog(monkeypatch):
  pushed = _prepare(monkeypatch, params=FakeParams({"UsbGpuEjectStatus": "error", "UsbGpuEjectError": "chestnut is in use"}))
  assert SidebarSP()._handle_egpu_click(_inside_home_btn()) is True
  assert isinstance(pushed[0], ConfirmDialog)
  assert "chestnut is in use" in pushed[0].text
