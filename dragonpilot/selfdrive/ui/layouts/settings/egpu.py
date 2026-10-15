"""
dp - egpu settings panel (tici big UI, i.e. comma 3 / 3X).

Ported from sunnypilot's models panel (selfdrive/ui/sunnypilot/layouts/settings/models.py), the big
model ("chestnut") half: model selector, status with cancel, refresh model list, clear model cache, the
failover note.

dp divergences:
  * the model list is jetlink's (JetlinkCatalog, shared with the Jetlink panel), one flat list newest
    first, as the Jetlink panel's big-model menu; no folders (sunnypilot's TreeOptionDialog groups by folder)
  * models are downloaded as ONNX and compiled on the device (dragonpilot/egpu/compile.py)
  * a "delete model" row lists the models with files on disk and deletes one (its compiled pkl and ONNX)
"""
import os
import threading
import time

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.list_view import text_item, button_item, toggle_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller
from dragonpilot.egpu.bundles import cached_bundles, pointer_for
from dragonpilot.egpu.constants import (MODEL_SOURCE_LABEL, P_ENABLED, P_DOWNLOAD_REF, P_MANIFEST_SYNC, P_CLEAR_CACHE,
                                        P_ACTIVE_BUNDLE, P_DELETE_REF, P_POINTERS, model_root, onnx_path)
from dragonpilot import jetlink_adapter
from dragonpilot.selfdrive.ui import egpu_ui

DESCRIPTION = "".join((
  "在接於 comma USB 埠的 eGPU（chestnut / USB AMD GPU）上執行大模型。",
  "模型清單與 Jetlink 共用；選擇後會下載模型的 onnx，並在本機編譯一次：",
  "warp 在 comma 的 GPU（QCOM）上執行，只把裁切後的影像送過 USB，適合 comma 3X。",
  "只能在熄火（offroad）時選擇、下載、編譯與刪除；編譯需要 eGPU 接上。",
))

NO_BIG_MODEL = "不使用大模型"


def _cache_size_mb() -> float:
  total = 0
  try:
    for name in os.listdir(model_root()):
      try:
        total += os.path.getsize(os.path.join(model_root(), name))
      except OSError:
        pass
  except OSError:
    pass
  pointers = ui_state.params.get(P_POINTERS)
  for p in (pointers.values() if isinstance(pointers, dict) else []):
    try:
      total += onnx_path(p["oid"]).stat().st_size
    except (OSError, KeyError, TypeError):
      pass
  return total / (1024 ** 2)


class EgpuLayout(Widget):
  def __init__(self):
    super().__init__()
    self._bundles: list[dict] = []
    self._dialog: MultiOptionDialog | None = None
    self._refreshing = False
    self._refresh_result = ""
    self._n_on_disk = 0
    self._cache_mb = 0.0
    self._cache_at = 0.0

    self._state_item = text_item("eGPU 狀態", lambda: egpu_ui.STATE_TEXT[ui_state.egpu_state],
                                 description=egpu_ui.status_note)
    self._enabled_item = toggle_item("啟用 eGPU 大模型", DESCRIPTION, initial_state=ui_state.params.get_bool(P_ENABLED),
                                     callback=self._on_enabled, enabled=ui_state.is_offroad)
    self._model_item = button_item("大模型", "選擇", callback=self._open_model_dialog, enabled=ui_state.is_offroad)
    self._model_item.action_item.set_value(egpu_ui.model_name)
    self._status_item = button_item("處理狀態", "取消", callback=self._cancel,
                                    enabled=lambda: egpu_ui.busy() and ui_state.params.get(P_DOWNLOAD_REF) is not None)
    self._status_item.action_item.set_value(egpu_ui.download_text)
    self._delete_item = button_item("刪除模型", "刪除", "選一個已下載或已編譯的模型，刪除它的編譯檔與 onnx。",
                                    callback=self._open_delete_dialog,
                                    enabled=lambda: ui_state.is_offroad() and not egpu_ui.busy())
    self._delete_item.action_item.set_value(lambda: f"{self._n_on_disk} 個")
    # the same refresh as the Jetlink panel's (jetlink_adapter.refresh_catalog): one list for both
    self._refresh_item = button_item("重新整理模型清單", lambda: "讀取中" if self._refreshing else "更新",
                                     self._refresh_note, callback=self._refresh_models,
                                     enabled=lambda: not self._refreshing)
    self._refresh_item.action_item.set_value(lambda: f"{len(self._bundles)} 個模型")
    self._clear_item = button_item("清除模型快取", "清除", "刪除所有已下載的 onnx 與編譯檔，只保留目前使用中的模型。",
                                   callback=self._clear_cache,
                                   enabled=lambda: ui_state.is_offroad() and not egpu_ui.busy())
    self._clear_item.action_item.set_value(lambda: f"{self._cache_mb:.0f} MB")
    self._source_item = text_item("模型清單", MODEL_SOURCE_LABEL)

    self._scroller = Scroller([self._state_item, self._enabled_item, self._model_item, self._status_item,
                               self._delete_item, self._refresh_item, self._clear_item, self._source_item], line_separator=True, spacing=0)

  # ------------------------------------------------------------------ actions
  def _on_enabled(self, state: bool):
    ui_state.params.put_bool(P_ENABLED, state)

  def _cancel(self):
    ui_state.params.remove(P_DOWNLOAD_REF)

  def _refresh_note(self) -> str:
    return self._refresh_result or "與 Jetlink 頁面共用同一份清單；任一邊更新，兩邊都會看到。"

  def _refresh_models(self):
    # exactly the Jetlink panel's refresh (dragonpilot/selfdrive/ui/layouts/settings/jetlink.py _refresh_catalog)
    if self._refreshing:
      return
    self._refreshing = True
    self._refresh_result = "更新中…"

    def work():
      try:
        n = jetlink_adapter.refresh_catalog(ui_state.params)
        ui_state.params.put(P_MANIFEST_SYNC, int(time.monotonic_ns()))
        self._refresh_result = f"已更新：{n} 個大模型。"
      except Exception as e:
        self._refresh_result = f"更新失敗：{e}"
      finally:
        self._refreshing = False
        self._cache_at = 0.0

    threading.Thread(target=work, daemon=True).start()

  def _clear_cache(self):
    def _callback(result):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool(P_CLEAR_CACHE, True)
    gui_app.push_widget(ConfirmDialog("將刪除所有已下載的 onnx 與編譯檔，只保留目前使用中的模型。確定嗎？", "清除", callback=_callback))

  # ------------------------------------------------------------------ selector
  def _push(self, title: str, options: list[str], current: str, on_pick):
    def callback(result):
      dialog, self._dialog = self._dialog, None
      if result == DialogResult.CONFIRM and dialog is not None:
        on_pick(dialog.selection)
    # current="" so the highlighted entry can still be confirmed
    self._dialog = MultiOptionDialog(title, options, "", callback=callback)
    self._dialog.selection = current
    gui_app.push_widget(self._dialog)

  def _labels(self, bundles: list[dict]) -> dict[str, dict]:
    return {egpu_ui.bundle_label(b, egpu_ui.model_state(b["ref"])): b for b in bundles}

  def _open_model_dialog(self):
    """one flat list, newest first (the Jetlink panel's menu): picking a model uses it, after downloading
    and compiling it when needed (egpu_manager skips every step whose result is already on disk)"""
    self._bundles = cached_bundles(ui_state.params)
    by_label = self._labels(self._bundles)
    active = ui_state.egpu_bundle
    current = next((label for label, b in by_label.items() if active and b["ref"] == active["ref"]), NO_BIG_MODEL)

    def on_pick(selection):
      if selection == NO_BIG_MODEL:
        ui_state.params.remove(P_ACTIVE_BUNDLE)
        ui_state.params.remove(P_DOWNLOAD_REF)
      elif selection in by_label:
        ui_state.params.put(P_DOWNLOAD_REF, by_label[selection]["ref"])
    self._push("選擇大模型", [NO_BIG_MODEL] + list(by_label), current, on_pick)

  def _open_delete_dialog(self):
    on_disk = [b for b in cached_bundles(ui_state.params) if egpu_ui.model_state(b["ref"]) != "none"]
    if not on_disk:
      gui_app.push_widget(alert_dialog("目前沒有已下載或已編譯的模型。"))
      return
    by_label = self._labels(on_disk)
    self._push("刪除哪個模型？", list(by_label), next(iter(by_label)), lambda sel: sel in by_label and self._confirm_delete(by_label[sel]))

  def _confirm_delete(self, bundle: dict):
    active = ui_state.egpu_bundle
    is_active = bool(active and active.get("ref") == bundle["ref"])
    p = pointer_for(ui_state.params, bundle["ref"])
    size = f"（onnx 約 {int(p['size']) / 1e6:.0f} MB）" if p else ""
    note = "這是目前使用中的模型，刪除後改由內建小模型駕駛。" if is_active else ""
    msg = f"刪除 {bundle['display_name']} 的編譯檔與 onnx{size}？{note}之後要再使用需重新下載並編譯。"

    def _callback(result):
      if result == DialogResult.CONFIRM:
        ui_state.params.put(P_DELETE_REF, bundle["ref"])
        self._cache_at = 0.0
    gui_app.push_widget(ConfirmDialog(msg, "刪除", callback=_callback))

  # ------------------------------------------------------------------ frame
  def _update_state(self):
    now = time.monotonic()
    if now - self._cache_at > 2.0:
      self._cache_at = now
      self._cache_mb = _cache_size_mb()
      self._bundles = cached_bundles(ui_state.params)
      self._n_on_disk = sum(egpu_ui.model_state(b["ref"]) != "none" for b in self._bundles)
    self._enabled_item.action_item.set_state(ui_state.egpu_enabled)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._bundles = cached_bundles(ui_state.params)
    self._cache_at = 0.0

  def _render(self, rect):
    self._scroller.render(rect)
