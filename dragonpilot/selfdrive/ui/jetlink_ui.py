"""
dp - jetlink: what the panels and the onroad/sidebar icons show of the link.

Ported from zoompilot's selfdrive/ui/sunnypilot/accelerator_link.py and the
jetlink parts of model_info.py / layouts/settings/models.py. Everything here
reads ui_state.jetlink (the snapshot the 5 Hz params pass takes) and
ui_state.jetlink_state, never the link itself.

The icons are comma's chestnut icons (commaai/openpilot
selfdrive/assets/icons_mici/chestnut*.png), shipped here as
dragonpilot/selfdrive/assets/icons/jetlink*.png: zoompilot uses the same
icons for the link.
"""
import math
from typing import Union

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, JetlinkState
from openpilot.system.ui.lib.application import gui_app
from dragonpilot.jetlink_adapter import KEYS, MODES

LINK_MODES = MODES
LINK_PARAM = KEYS.link
LINK_MODE_TITLES = {"off": "關閉", "usb": "USB", "ios": "iOS"}

ICON_DIR = "../../dragonpilot/selfdrive/assets/icons"

# the UI fonts are bitmaps holding ASCII, a few symbols and the characters of the
# translation files: a character outside them (an em dash, a middle dot) draws as "?"
NONE_TEXT = "無"

# one short word per state, for the sidebar card and the onroad badge
STATE_TEXT = {
  JetlinkState.DISCONNECTED: "未連線",
  JetlinkState.UNCOMPILED: "未就緒",
  JetlinkState.READY: "就緒",
  JetlinkState.LOADING: "連線中",
  JetlinkState.ACTIVE: "大模型",
  JetlinkState.FAILED: "失敗",
  JetlinkState.WAITING: "待切換",
}

GREEN = rl.Color(128, 216, 166, 255)
YELLOW = rl.Color(218, 202, 37, 255)
ORANGE = rl.Color(255, 140, 40, 255)
GREY = rl.Color(166, 166, 166, 255)

STATE_COLOR = {
  JetlinkState.DISCONNECTED: GREY,
  JetlinkState.UNCOMPILED: ORANGE,
  JetlinkState.READY: GREEN,
  JetlinkState.LOADING: YELLOW,
  JetlinkState.ACTIVE: GREEN,
  JetlinkState.FAILED: ORANGE,
  JetlinkState.WAITING: GREEN,
}


def link_mode() -> str:
  """The setting as stored, one of LINK_MODES: shown even when jetlink cannot
  run, so a link that will not start can be turned off."""
  try:
    index = int(ui_state.params.get(LINK_PARAM) or 0)
  except (TypeError, ValueError):
    index = 0
  return LINK_MODES[index] if 0 <= index < len(LINK_MODES) else "off"


def link_toggle_meaningful() -> bool:
  """Offered wherever jetlink is present, except beside a USB GPU, which runs
  the big model itself; and while the setting is on, whatever else."""
  if ui_state.usbgpu:
    return link_mode() != "off"
  return ui_state.jetlink is not None or link_mode() != "off"


def link_status() -> str:
  """One line: what is on the comma's USB-C port right now. jetlink knows a
  Jetson or a phone and the transport says which; below that only the CC pin
  speaks (a cable with a host behind it, not what the host is)."""
  jetlink = ui_state.jetlink
  if jetlink is None:
    return ""
  if jetlink.present:
    return f"Jetlink 已連線：{jetlink.transport}。"
  if jetlink.port is None:
    return ""
  return "USB 埠上沒有裝置。" if jetlink.port == "empty" else "USB 埠上有裝置（尚未回應 Jetlink）。"


def progress() -> tuple[str, float, str] | None:
  """(stage, 0..1, message) while jetlink is working, else None. The message
  names the cable once jetlink counts enough link drops to blame it."""
  jetlink = ui_state.jetlink
  p = jetlink.progress if jetlink is not None else None
  if not p:
    return None
  stage = str(p.get('stage', ''))
  if stage in ('', 'ready'):
    return None
  msg = str(p.get('msg', ''))
  if drops := p.get('drops'):
    hint = "請檢查線材或 App" if jetlink.mode == 'ios' else "請檢查線材"
    msg = f"{msg}，{hint}（斷線 {drops} 次）"
  return stage, float(p.get('frac', 0.0)), msg


def _alka_on() -> bool:
  try:
    return ui_state.params.get_bool("dp_lat_alka")
  except Exception:
    return False


def status_note(model_name: str | None = None) -> str:
  """The failover story (zoompilot's Model Status note, jetlink half). `model_name`:
  the model that will drive when the caller knows better than the snapshot (following
  the far end, it is the far end's loaded model, not the comma's default)."""
  view = ui_state.jetlink_view
  if view is None:
    return ""
  big_name = (model_name if model_name and model_name != NONE_TEXT else None) or view.model or "大模型"
  state = ui_state.jetlink_state
  if state in (JetlinkState.FAILED, JetlinkState.UNCOMPILED):
    if view.reason:
      return f"大模型無法使用：{view.reason}。由小模型駕駛。"
    return "大模型無法使用，由小模型駕駛。"
  if state == JetlinkState.WAITING:
    if _alka_on():
      return f"{big_name} 已就緒。ALKA 開啟中，取消巡航不會切換：關閉巡航主開關約 1 秒，或停車排入 P／N／R 檔即切換。"
    return f"{big_name} 已就緒。取消巡航即切換，約 1 秒後再啟用。"
  if state == JetlinkState.LOADING:
    return "大模型就緒前由小模型駕駛。"
  if not view.ready:
    if view.standin:
      return f"{view.standin} 先行駕駛，直到 {big_name} 準備完成。"
    return f"Jetlink 就緒後將由 {big_name} 駕駛。"
  return f"將由 {big_name} 駕駛；連線中斷時自動改用小模型，恢復後再切回。"


class JetlinkIcons:
  """The three icon textures at one size, and how a state draws."""

  def __init__(self, width: int, height: int):
    self.green = gui_app.texture(f"{ICON_DIR}/jetlink_green.png", width, height)
    self.default = gui_app.texture(f"{ICON_DIR}/jetlink.png", width, height)
    self.orange = gui_app.texture(f"{ICON_DIR}/jetlink_orange.png", width, height)

  def for_state(self, state: JetlinkState) -> tuple[Union[rl.Texture, None], float]:
    """(texture, opacity) for a state; None while disconnected. Loading pulses,
    waiting is a steady dim green, failed is orange, ready/active solid green."""
    if state == JetlinkState.DISCONNECTED:
      return None, 0.0
    if state == JetlinkState.LOADING:
      return self.default, 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    if state == JetlinkState.WAITING:
      return self.green, 0.5
    if state in (JetlinkState.UNCOMPILED, JetlinkState.FAILED):
      return self.orange, 1.0
    return self.green, 1.0


def tint(opacity: float) -> rl.Color:
  return rl.Color(255, 255, 255, int(255 * max(0.0, min(1.0, opacity))))
