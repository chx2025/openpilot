"""
dp - egpu: what the settings panel and the onroad / sidebar icons show of the eGPU big model.

Ported from sunnypilot's ChestnutState icon logic (selfdrive/ui/ui_state.py, layouts/sidebar.py,
mici/onroad/hud_renderer.py) and model_info.py. Reads ui_state.egpu_* (taken on the 5 Hz params pass).

The icons are comma's chestnut icons, the same files the jetlink port ships
(dragonpilot/selfdrive/assets/icons/jetlink*.png): sunnypilot draws the chestnut with them too.
Strings are Traditional Chinese written in place, as the jetlink panel does; every character used
is one the UI font already carries (selfdrive/ui/translations/app_zh-CHT.po), otherwise it draws "?".
"""
from openpilot.selfdrive.ui.ui_state import ui_state, EgpuState, JetlinkState
from dragonpilot.selfdrive.ui import jetlink_ui
from dragonpilot.egpu.bundles import pointer_for, onnx_present, model_compiled
from dragonpilot.egpu.constants import (ST_IDLE, ST_RESOLVING, ST_DOWNLOADING, ST_COMPILING_WARP, ST_COMPILING, ST_DONE,
                                        ST_FAILED, ST_CANCELLED, ST_WAITING_EGPU, BUSY_STATES)

NONE_TEXT = jetlink_ui.NONE_TEXT

STATE_TEXT = {
  EgpuState.DISCONNECTED: "未連線",
  EgpuState.NO_MODEL: "未使用",
  EgpuState.READY: "就緒",
  EgpuState.LOADING: "載入中",
  EgpuState.ACTIVE: "大模型",
  EgpuState.FAILED: "失敗",
}

STATE_COLOR = {
  EgpuState.DISCONNECTED: jetlink_ui.GREY,
  EgpuState.NO_MODEL: jetlink_ui.ORANGE,
  EgpuState.READY: jetlink_ui.GREEN,
  EgpuState.LOADING: jetlink_ui.YELLOW,
  EgpuState.ACTIVE: jetlink_ui.GREEN,
  EgpuState.FAILED: jetlink_ui.ORANGE,
}

# same drawing rules as sunnypilot's chestnut icon, through jetlink's JetlinkIcons
_AS_JETLINK = {
  EgpuState.DISCONNECTED: JetlinkState.DISCONNECTED,
  EgpuState.NO_MODEL: JetlinkState.UNCOMPILED,   # orange: plugged in but not used
  EgpuState.READY: JetlinkState.READY,
  EgpuState.LOADING: JetlinkState.LOADING,       # pulses
  EgpuState.ACTIVE: JetlinkState.ACTIVE,
  EgpuState.FAILED: JetlinkState.FAILED,
}


def icon_for(icons: "jetlink_ui.JetlinkIcons", state: EgpuState):
  return icons.for_state(_AS_JETLINK[state])


def badge_text(state: EgpuState) -> str:
  """the onroad badge's word: the model's short name while it drives"""
  if state == EgpuState.ACTIVE and ui_state.egpu_bundle:
    return str(ui_state.egpu_bundle.get("short_name", STATE_TEXT[state]))
  return STATE_TEXT[state]


def model_name() -> str:
  b = ui_state.egpu_bundle
  return b["display_name"] if b else "未選擇（由內建小模型駕駛）"


def model_state(ref: str) -> str:
  """'compiled' | 'downloaded' | 'none' for one catalog model, from the files on disk"""
  p = pointer_for(ui_state.params, ref)
  if p and model_compiled(p["oid"]):
    return "compiled"
  if onnx_present(p):
    return "downloaded"
  return "none"


def bundle_label(b: dict, state: str) -> str:
  active = ui_state.egpu_bundle
  if active and active.get("ref") == b["ref"]:
    tag = "（使用中）"
  else:
    tag = {"compiled": "（已編譯）", "downloaded": "（已下載）"}.get(state, "")
  return f"{b['display_name']}{tag}"


def status_note() -> str:
  """sunnypilot's Model Status failover note, for the eGPU"""
  state = ui_state.egpu_state
  name = (ui_state.egpu_bundle or {}).get("short_name", "大模型")
  if state == EgpuState.DISCONNECTED:
    return "未偵測到 eGPU。可先選擇並下載大模型，接上 eGPU 後會自動完成編譯。"
  if not ui_state.egpu_enabled:
    return "eGPU 大模型已關閉，由內建小模型駕駛。"
  if state == EgpuState.NO_MODEL:
    if ui_state.egpu_bundle is None:
      return "已偵測到 eGPU，但尚未有編譯好的大模型，由內建小模型駕駛。"
    return "目前由小模型駕駛，等待 eGPU 就緒。若本趟未啟用 eGPU，請停車後重新發動。"
  if state == EgpuState.FAILED:
    return f"{name} 無法使用，已改用小模型。解除 ACC 與全時置中後可重新嘗試；若失敗時已解除，請再次開啟並關閉巡航主開關。"
  if state == EgpuState.LOADING:
    return f"{name} 背景載入中，目前由小模型駕駛。"
  if state == EgpuState.ACTIVE:
    return f"{name} 正在 eGPU 上駕駛；失敗時改用小模型，解除控制後可重試。"
  if ui_state.started:
    return f"{name} 已就緒；解除 ACC 與全時置中後切入，目前仍由小模型駕駛。"
  return f"發動後先由小模型駕駛，再背景載入 {name}。"


def _mmss(seconds) -> str:
  s = int(seconds or 0)
  return f"{s // 60}:{s % 60:02d}"


def download_text() -> str:
  st = ui_state.egpu_status or {}
  state = st.get("state", ST_IDLE)
  name = st.get("name", "")
  if state == ST_RESOLVING:
    return f"{name} 查詢模型中"
  if state == ST_DOWNLOADING:
    return f"{name} 下載 onnx {st.get('progress', 0.0):.0f}%"
  if state == ST_COMPILING_WARP:
    return f"編譯 warp {_mmss(st.get('elapsed'))}"
  if state == ST_COMPILING:
    return f"{name} 編譯中 {_mmss(st.get('elapsed'))}"
  if state == ST_WAITING_EGPU:
    if st.get("error", "").startswith("PCIe"):
      return f"{name} 等待 eGPU 顯示卡連線（{st['error']}）"
    return f"{name} 等待 eGPU 接上以編譯"
  if state == ST_DONE:
    return f"{name} 已完成"
  if state == ST_FAILED:
    return f"{name} 失敗"
  if state == ST_CANCELLED:
    return f"{name} 已取消"
  return NONE_TEXT


def busy() -> bool:
  return (ui_state.egpu_status or {}).get("state") in (*BUSY_STATES, ST_WAITING_EGPU)
