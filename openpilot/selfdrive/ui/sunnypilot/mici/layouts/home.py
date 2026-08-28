"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets.icon_widget import IconWidget
from openpilot.system.ui.widgets.label import UnifiedLabel

EGPU_LOADING_COLOR = rl.Color(0, 134, 233, 255)


class MiciHomeLayoutSP(MiciHomeLayout):
  def __init__(self):
    super().__init__()
    self._openpilot_label = UnifiedLabel("sunnypilot", font_size=88, font_weight=FontWeight.AUDIOWIDE, max_width=480, wrap_text=False)
    self._egpu_icon_default = IconWidget("icons_mici/egpu.png", (50, 37))
    self._egpu_icon_default.set_visible(False)
    self._egpu_icon_orange = IconWidget("icons_mici/egpu_orange.png", (50, 37))
    self._egpu_icon_orange.set_visible(False)
    gray_idx = self._status_bar_layout.widgets.index(self._egpu_icon_gray)
    self._status_bar_layout.widgets.insert(gray_idx + 1, self._egpu_icon_default)
    self._status_bar_layout.widgets.insert(gray_idx + 2, self._egpu_icon_orange)
    self._egpu_loading_label = UnifiedLabel("", font_size=36, text_color=EGPU_LOADING_COLOR,
                                            font_weight=FontWeight.MEDIUM, max_width=480, wrap_text=False)

  def _set_egpu_visibility(self):
    chestnut = ui_state.sm["deviceState"].chestnutPresent
    if not chestnut:
      self._egpu_icon.set_visible(False)
      self._egpu_icon_default.set_visible(False)
      self._egpu_icon_orange.set_visible(False)
      self._egpu_icon_gray.set_visible(False)
      self._egpu_loading_label.set_text("")
      return

    big_model_selected = ui_state.usbgpu_compiled or ui_state.model_runner_tinygrad
    big_model_failed = ui_state.started and ui_state.big_model_failed
    loading = ui_state.usbgpu_loading or (big_model_selected and ui_state.started and ui_state.usbgpu_active is None)

    if loading:
      self._egpu_icon_default._opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
      self._egpu_icon_default.set_visible(True)
      self._egpu_icon.set_visible(False)
      self._egpu_icon_orange.set_visible(False)
      self._egpu_icon_gray.set_visible(False)
      if ui_state.usbgpu_loading:
        self._egpu_loading_label.set_text(f"eGPU {ui_state.usbgpu_loading_progress}%")
      else:
        self._egpu_loading_label.set_text("eGPU ...")
    else:
      self._egpu_icon_default.set_visible(False)
      self._egpu_icon.set_visible(big_model_selected and not big_model_failed)
      self._egpu_icon_orange.set_visible(big_model_selected and big_model_failed)
      self._egpu_icon_gray.set_visible(not big_model_selected)
      self._egpu_loading_label.set_text("")

  def _render(self, rect):
    super()._render(rect)
    if self._egpu_loading_label.text:
      self._egpu_loading_label.set_position(rect.x + 20, rect.y + rect.height - 110)
      self._egpu_loading_label.render()
