"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

================================================================================
顶部居中的北京时间显示条
================================================================================

- 设备时区就是 UTC（`/etc/localtime -> /data/etc/localtime`，`date` 与 `date -u` 完全一致），
  所以这里**不依赖 TZ 环境变量**，直接 `utc + 8h` 再用 `gmtime()` 格式化。
- 常驻显示 `HH:MM:SS`。带上秒的原因：校时（`time_seed.sh`）成不成功一眼就能看出来
  —— 时间错的时候秒在走但整体偏；校时成功的那一刻数字会跳变。
- 位置：顶部居中，与 `road_name` 同一条带（`road_name` 若开启会自动下移避让，
  见 `road_name.py` 里的 `clock_reserved_height()`）。

想关掉：把 `CLOCK_ENABLED` 改成 False（road_name 的避让会自动失效）。
想换颜色：改 `CLOCK_TEXT_COLOR`。
想只显示 HH:MM：把 `CLOCK_TIME_FORMAT` 改成 "%H:%M"。
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import NUMBER_GREEN
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

# ---- 可调项 ----
CLOCK_ENABLED = True
CLOCK_TIME_FORMAT = "%H:%M:%S"          # 想省地方就改成 "%H:%M"
CLOCK_FONT_SIZE = 46
CLOCK_BAR_HEIGHT = 60
CLOCK_PADDING = 40                      # 背景左右各留的空白(px)
CLOCK_Y_OFFSET = 4                      # 距内容区顶部的内缩(px)，与 road_name 原值一致
CLOCK_BG_COLOR = rl.Color(0, 0, 0, 120)  # 半透明黑，与 road_name 同款
CLOCK_TEXT_COLOR = NUMBER_GREEN         # 全 UI 数字统一绿；想改白色用 rl.Color(255, 255, 255, 200)

BEIJING_OFFSET_S = 8 * 3600             # 北京时间 = UTC+8
REFRESH_INTERVAL_S = 0.1                # 重算字符串的最小间隔，避免每帧都格式化


def clock_reserved_height() -> int:
  """顶部被时钟条占掉的高度，供同在顶部居中的元素（road_name）避让。"""
  return (CLOCK_BAR_HEIGHT + CLOCK_Y_OFFSET) if CLOCK_ENABLED else 0


class ClockDisplayRenderer:
  def __init__(self):
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._text = ""
    self._last_time = 0.0

  def update(self) -> None:
    if not CLOCK_ENABLED:
      return

    # 按挂钟时间取整到秒触发刷新；比每帧 strftime 省，且跨秒即更新
    now = time.time()
    if self._text and (now - self._last_time) < REFRESH_INTERVAL_S:
      return
    self._last_time = now
    self._text = time.strftime(CLOCK_TIME_FORMAT, time.gmtime(now + BEIJING_OFFSET_S))

  def render(self, rect: rl.Rectangle) -> None:
    if not CLOCK_ENABLED or not self._text:
      return

    text_size = measure_text_cached(self._font, self._text, CLOCK_FONT_SIZE)
    bar_width = max(140.0, text_size.x + CLOCK_PADDING * 2)

    bar_rect = rl.Rectangle(
      rect.x + rect.width / 2 - bar_width / 2,
      rect.y + CLOCK_Y_OFFSET,
      bar_width,
      CLOCK_BAR_HEIGHT,
    )
    rl.draw_rectangle_rounded(bar_rect, 0.2, 10, CLOCK_BG_COLOR)

    origin = rl.Vector2(
      bar_rect.x + bar_rect.width / 2 - text_size.x / 2,
      bar_rect.y + bar_rect.height / 2 - text_size.y / 2,
    )
    rl.draw_text_ex(self._font, self._text, origin, CLOCK_FONT_SIZE, 0, CLOCK_TEXT_COLOR)
