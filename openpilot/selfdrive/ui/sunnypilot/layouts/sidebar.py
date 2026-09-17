"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl
from dataclasses import dataclass
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr_noop
from openpilot.selfdrive.ui.egpu_status import (
  build_egpu_sidebar_status, chestnut_usb_speed_mbps, classify_egpu_link_state, resolve_egpu_connection,
)


METRIC_HEIGHT = 126
METRIC_MARGIN = 30
METRIC_START_Y = 300
HOME_BTN = rl.Rectangle(60, 860, 180, 180)

CHESTNUT_ICON_WIDTH = 180
CHESTNUT_ICON_HEIGHT = 133


# Color scheme
class Colors:
  WHITE = rl.WHITE
  WHITE_DIM = rl.Color(255, 255, 255, 85)
  GRAY = rl.Color(84, 84, 84, 255)

  # Status colors
  # GOOD 只作为 MetricData.color 使用，也就是第四格左侧那条粗竖线的颜色：
  # severity == "good" -> 绿色（其余 warning/danger/progress/disabled 保持原样，2026-09-17 改）
  GOOD = rl.Color(0, 230, 60, 255)
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

  def update(self, label: str, value: str, color: rl.Color):
    self.label = label
    self.value = value
    self.color = color


class SidebarSP:
  def __init__(self):
    # 第四格：原 SUNNYLINK，现改为 eGPU 状态（离线时显示「小模型」）
    self._egpu_status = MetricData("eGPU", "小模型", Colors.DISABLED)
    self._chestnut_green_img = gui_app.texture("icons_mici/chestnut_green.png", CHESTNUT_ICON_WIDTH, CHESTNUT_ICON_HEIGHT)
    self._chestnut_default_img = gui_app.texture("icons_mici/chestnut.png", CHESTNUT_ICON_WIDTH, CHESTNUT_ICON_HEIGHT)
    self._chestnut_orange_img = gui_app.texture("icons_mici/chestnut_orange.png", CHESTNUT_ICON_WIDTH, CHESTNUT_ICON_HEIGHT)
    self._egpu_offline_img = gui_app.texture("icons_mici/egpu_offline.png", CHESTNUT_ICON_WIDTH, CHESTNUT_ICON_HEIGHT)

  def _update_egpu_status(self):
    sm = ui_state.sm
    device_state = sm["deviceState"]

    present = resolve_egpu_connection(device_state)
    speed_mbps = chestnut_usb_speed_mbps(device_state)

    telemetry = sm["chestnutState"]
    telemetry_alive = bool(sm.alive["chestnutState"])
    # ⚠️ 不要用 sm.valid[...] 或 telemetry.metricsValid 判断遥测有效性（2026-09-17 修）：
    #   - sm.valid[...] 在本分支恒为 False（实测 20s / 3580 次采样 / 200 次收包 / 0 次 valid），
    #     会让遥测永远"无效" -> check_error -> 侧边栏误报 LINK ERR；
    #   - ChestnutState 结构里根本没有 metricsValid 字段（capnp 会抛 AttributeError），
    #     只是因为 sm.valid 恒 False 短路了才没炸。
    # alive=True 就说明 modeld/hardwared 在正常发布，字段可读即可用。
    telemetry_valid = telemetry_alive
    pcie_ltssm = int(telemetry.pcieLtssm) if telemetry_alive else 0

    link_state = classify_egpu_link_state(
      present=present,
      usb_speed_mbps=speed_mbps,
      telemetry_alive=telemetry_alive,
      telemetry_valid=telemetry_valid,
      pcie_ltssm=pcie_ltssm,
    )

    # 大模型"在跑"的双保险：参数 + ui_state 的状态机推断（后者含 modelV2.big 判据）
    active = bool(ui_state.chestnut_active) or ui_state.chestnut_state == ChestnutState.ACTIVE

    status = build_egpu_sidebar_status(
      present=present,
      compiled=ui_state.chestnut_compiled,
      link_state=link_state,
      usb_speed_mbps=speed_mbps,
      pcie_ltssm=pcie_ltssm if telemetry_alive else None,
      loading=ui_state.chestnut_loading,
      active=active,
      loading_progress=ui_state.chestnut_loading_progress,
      model_failed=ui_state.big_model_failed,
      power_w=float(telemetry.powerDrawW) if telemetry_valid else 0.0,
      gpu_usage_percent=int(telemetry.gpuUsagePercent) if telemetry_valid else 0,
      telemetry_valid=telemetry_valid,
    )

    color = {
      "good": Colors.GOOD,
      "warning": Colors.WARNING,
      "danger": Colors.DANGER,
      "progress": Colors.PROGRESS,
      "disabled": Colors.DISABLED,
    }[status.severity]

    self._egpu_status.update("eGPU", status.value, color)

  def _get_home_icon(self, default_img: rl.Texture) -> tuple[rl.Texture, rl.Vector2, float]:
    state = ui_state.chestnut_state
    if state == ChestnutState.DISCONNECTED:
      icon = self._egpu_offline_img
      x = HOME_BTN.x + (HOME_BTN.width - icon.width) / 2
      y = HOME_BTN.y + (HOME_BTN.height - icon.height) / 2
      return icon, rl.Vector2(x, y), 1.0

    if state == ChestnutState.LOADING:
      icon = self._chestnut_default_img
      opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED):
      icon, opacity = self._chestnut_orange_img, 1.0
    else:
      icon, opacity = self._chestnut_green_img, 1.0

    x = HOME_BTN.x + (HOME_BTN.width - icon.width) / 2
    y = HOME_BTN.y + (HOME_BTN.height - icon.height) / 2
    return icon, rl.Vector2(x, y), opacity

  def _draw_metrics_sp(self, rect: rl.Rectangle, _temp, _panda, _connect):
    metrics = [_temp, _panda, _connect, self._egpu_status]
    start_y = int(rect.y) + METRIC_START_Y
    available_height = max(0, int(HOME_BTN.y) - METRIC_MARGIN - METRIC_HEIGHT - start_y)
    spacing = available_height / max(1, len(metrics) - 1)

    return metrics, start_y, spacing
