"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from dataclasses import dataclass
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog


METRIC_HEIGHT = 126
METRIC_WIDTH = 240
METRIC_MARGIN = 30
METRIC_START_Y = 300
HOME_BTN = rl.Rectangle(60, 860, 180, 180)


# Color scheme
class Colors:
  WHITE = rl.WHITE
  WHITE_DIM = rl.Color(255, 255, 255, 85)
  GRAY = rl.Color(84, 84, 84, 255)

  # Status colors
  GOOD = rl.WHITE
  WARNING = rl.Color(218, 202, 37, 255)
  DANGER = rl.Color(201, 34, 49, 255)
  PROGRESS = rl.Color(0, 134, 233, 255)
  DISABLED = rl.Color(128, 128, 128, 255)

  # UI elements
  METRIC_BORDER = rl.Color(255, 255, 255, 85)
  BUTTON_NORMAL = rl.WHITE
  BUTTON_PRESSED = rl.Color(255, 255, 255, 166)


@dataclass(slots=True)
class MetricData:
  label: str
  value: str
  color: rl.Color
  icon: rl.Texture | None = None

  def update(self, label: str, value: str, color: rl.Color, icon: rl.Texture | None = None):
    self.label = label
    self.value = value
    self.color = color
    self.icon = icon


class SidebarSP:
  def __init__(self):
    self._egpu_icon = gui_app.texture("icons_mici/egpu_green.png", 60, 44)
    self._egpu_icon_gray = gui_app.texture("icons_mici/egpu_gray.png", 60, 44)
    self._egpu_status = MetricData("eGPU", "未连接", Colors.DISABLED, self._egpu_icon_gray)
    self._egpu_metric_rect = rl.Rectangle(0, 0, 0, 0)

  def _update_egpu_status(self):
    present = bool(ui_state.sm["deviceState"].chestnutPresent)
    eject_status = ui_state.params.get("UsbGpuEjectStatus")

    if eject_status == "ejecting":
      value, color, icon = "卸载中", Colors.PROGRESS, self._egpu_icon_gray
    elif eject_status == "safe":
      value, color, icon = "可拔出", Colors.GOOD, self._egpu_icon_gray
    elif eject_status == "error":
      value, color, icon = "卸载失败", Colors.DANGER, self._egpu_icon_gray
    elif present and ui_state.usbgpu_compiled:
      value, color, icon = "已连接", Colors.GOOD, self._egpu_icon
    elif present:
      value, color, icon = "未编译", Colors.WARNING, self._egpu_icon_gray
    else:
      value, color, icon = "未连接", Colors.DISABLED, self._egpu_icon_gray

    self._egpu_status.update("eGPU", value, color, icon)

  def _handle_egpu_click(self, mouse_pos) -> bool:
    if not rl.check_collision_point_rec(mouse_pos, self._egpu_metric_rect):
      return False

    if ui_state.started:
      gui_app.push_widget(ConfirmDialog("只能在停车后的 offroad 界面安全卸载 eGPU。", "确定", cancel_text=""))
      return True

    present = bool(ui_state.sm["deviceState"].chestnutPresent)
    status = ui_state.params.get("UsbGpuEjectStatus")
    if not present or status in ("ejecting", "safe"):
      return True

    def confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool("UsbGpuEjectRequest", True)

    error = ui_state.params.get("UsbGpuEjectError")
    if status == "error" and error:
      message = f"上次卸载失败：{error}\n是否重试安全卸载？"
      confirm_text = "重试卸载"
    else:
      message = "安全卸载 eGPU？\n请等界面显示“可拔出”后再断开连接。"
      confirm_text = "安全卸载"
    gui_app.push_widget(ConfirmDialog(message, confirm_text, callback=confirm))
    return True

  def _draw_metrics_sp(self, rect: rl.Rectangle, _temp, _panda, _connect):
    metrics = [_temp, _panda, _connect, self._egpu_status]
    start_y = int(rect.y) + METRIC_START_Y
    available_height = max(0, int(HOME_BTN.y) - METRIC_MARGIN - METRIC_HEIGHT - start_y)
    spacing = available_height / max(1, len(metrics) - 1)
    self._egpu_metric_rect = rl.Rectangle(rect.x + METRIC_MARGIN, start_y + (len(metrics) - 1) * spacing, METRIC_WIDTH, METRIC_HEIGHT)

    return metrics, start_y, spacing
