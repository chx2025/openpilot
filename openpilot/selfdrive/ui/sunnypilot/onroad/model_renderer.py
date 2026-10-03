"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath
from openpilot.selfdrive.ui.sunnypilot.ui_state import MADSState
from openpilot.system.ui.lib.application import gui_app


class ModelRendererSP:
  # ── 路径条宽度（半宽，单位：米）───────────────────────────────────────────
  #   ★ 上游 openpilot 原值是 0.9（= 全宽 1.8 m），并非"车宽"，而是一个固定的
  #     视觉设计值。2026-10-01 用户要求改成自己车的实际宽度：
  #       车宽 1855 mm；**含后视镜 2155 mm**（用户指定按含后视镜算）。
  #     ⇒ 半宽 = 2.155 / 2 = 1.0775 m
  #   ⚠ 纯 UI / 视觉改动，**不参与任何控制计算**（路径条只影响画面，不影响 MPC、
  #     不影响横向控制）。所以调它只有"看着顺不顺眼"的问题，没有安全后果。
  #   ⚠ 未激活时的 0.40 保留不动（那是"路径尚未接管"的视觉提示，与车宽无关）。
  PATH_HALF_WIDTH_ACTIVE = 1.0775   # 2155 mm / 2
  PATH_HALF_WIDTH_IDLE = 0.40       # 上游原值，未激活

  def __init__(self):
    self.rainbow_path = RainbowPath()
    self.chevron_metrics = ChevronMetrics()
    self._width_filter = FirstOrderFilter(self.PATH_HALF_WIDTH_ACTIVE, 0.1, 1 / gui_app.target_fps)

  @property
  def _lateral_active(self) -> bool:
    sm = ui_state.sm
    if sm.valid["selfdriveStateSP"]:
      mads = sm["selfdriveStateSP"].mads
      if mads.available:
        return mads.enabled and mads.state != MADSState.paused
    return ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)

  def _get_path_half_width(self) -> float:
    target = self.PATH_HALF_WIDTH_ACTIVE if self._lateral_active else self.PATH_HALF_WIDTH_IDLE
    return self._width_filter.update(target)
