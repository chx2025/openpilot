import math
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds

# --- 方向燈與盲區閃爍頻率設定 ---
DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)
DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)
DP_INDICATOR_COLOR_BSM = rl.Color(255, 204, 0, 220)      # 盲區黃色
DP_INDICATOR_COLOR_BLINKER = rl.Color(0, 255, 0, 220)    # 方向燈綠色


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    # --- 前車距離變數 ---
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # --- 邊緣閃爍狀態變數 ---
    self._dp_indicator_show_left = False
    self._dp_indicator_show_right = False
    self._dp_indicator_count_left = 0
    self._dp_indicator_count_right = 0
    self._dp_indicator_color_left = rl.Color(0, 0, 0, 0)
    self._dp_indicator_color_right = rl.Color(0, 0, 0, 0)

    self._can_draw_top_icons = True

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_dp_indicator_side_state(self, blinker_state, bsm_state, show_prev, count_prev):
    """處理單邊的閃爍與顏色邏輯"""
    show = show_prev
    count = count_prev
    color = rl.Color(0, 0, 0, 0)

    if not blinker_state and not bsm_state:
      show = False
      count = 0
    else:
      count += 1

    if bsm_state and blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_FAST == 0 else show
      color = DP_INDICATOR_COLOR_BSM
    elif blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_STD == 0 else show
      color = DP_INDICATOR_COLOR_BLINKER
    elif bsm_state:
      show = True
      color = DP_INDICATOR_COLOR_BSM
    else:
      show = False

    return show, count, color

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0

      self._dp_indicator_show_left = False
      self._dp_indicator_show_right = False
      return

    # --- 讀取雷達狀態（前車距離） ---
    radar_state = sm['radarState']
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    controls_state = sm['controlsState']
    car_state = sm['carState']

    # --- 更新兩側方向燈與盲區閃爍狀態 ---
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left = \
      self._update_dp_indicator_side_state(car_state.leftBlinker, car_state.leftBlindspot,
                                           self._dp_indicator_show_left, self._dp_indicator_count_left)

    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right = \
      self._update_dp_indicator_side_state(car_state.rightBlinker, car_state.rightBlindspot,
                                           self._dp_indicator_show_right, self._dp_indicator_count_right)

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    if self.is_cruise_set:
      self._draw_set_speed(rect)

    # 置中的動態立體倒三角形與前車距離
    self._draw_lead_info(rect)

    # 自帶雙閃爍頻率的方向燈與盲區邊條（放最後畫，確保圖層在最上方，不被裁切）
    self._draw_edge_warnings(rect)

  def _draw_edge_warnings(self, rect: rl.Rectangle) -> None:
    """繪製兩側方向燈與盲區警示（圓角效果並垂直置中）"""
    bar_width = 20
    bar_height = int(rect.height * 0.60)

    # Y 軸垂直置中，並向上微調 20px
    y_pos = int(rect.y + (rect.height - bar_height) / 2) - 20

    if self._dp_indicator_show_left:
      left_rect = rl.Rectangle(int(rect.x), y_pos, bar_width, bar_height)
      rl.draw_rectangle_rounded(left_rect, 0.75, 20, self._dp_indicator_color_left)

    if self._dp_indicator_show_right:
      right_rect = rl.Rectangle(int(rect.x + rect.width - bar_width), y_pos, bar_width, bar_height)
      rl.draw_rectangle_rounded(right_rect, 0.75, 20, self._dp_indicator_color_right)

  def _draw_lead_info(self, rect: rl.Rectangle) -> None:
    """繪製置中的圓角立體「倒三角形」與前車距離"""

    # 無鎖定前車時，不顯示圖示與數字
    if self.lead_dist == "-":
      return

    # 尺寸設定：高度 40，寬度 45
    bar_h = 40.0
    bar_w = 45.0

    # 垂直位置與水平置中
    pos_y = int(rect.y + rect.height - 39)
    bar_y = pos_y - bar_h / 2
    bar_x = rect.x + (rect.width - bar_w) / 2

    dist_text = self.lead_dist
    dist_font_size = 40
    dist_size = measure_text_cached(self._font_bold, dist_text, dist_font_size)

    # 距離數據顯示在圖形左側（保持 15px 間距）
    text_x = bar_x - dist_size.x - 15
    text_y = pos_y - dist_size.y / 2

    dist_color = rl.WHITE

    # 判斷是否處於警告狀態
    is_warning = (self.lead_dist_raw < 15.0)

    if is_warning:
      # 呼吸燈效果
      alpha = 150 + int(60 * math.sin(time.time() * 5))
      center_color = rl.Color(255, 100, 100, alpha)
      edge_color = rl.Color(180, 0, 0, alpha)
      dist_color = rl.Color(255, 100, 100, 255)
    else:
      # 安全狀態：綠色恆亮
      center_color = rl.Color(150, 255, 150, 255)
      edge_color = rl.Color(0, 180, 0, 255)
      dist_color = rl.Color(128, 216, 166, 255)

    # 繪製文字陰影與主體
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x + 2, text_y + 2), dist_font_size, 0, rl.Color(0, 0, 0, 150))
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x, text_y), dist_font_size, 0, dist_color)

    # --- 圓角三角形繪製演算法（避免半透明圖層重疊加深） ---
    def draw_rounded_triangle_poly(x, y, w, h, r, scale, color):
        cx = x + w / 2
        # 倒三角形，視覺形心偏上方
        cy = y + h / 3

        # 內縮後的原始角點中心（倒三角形：左上 -> 底部中心 -> 右上）
        # 確保順序為逆時針 (Counter-Clockwise)
        v1 = rl.Vector2(x + r, y + r)                  # 左上
        v2 = rl.Vector2(x + w / 2, y + h - r)          # 底部中心
        v3 = rl.Vector2(x + w - r, y + r)              # 右上

        # 依據 scale 比例推算實際點位
        c1 = rl.Vector2(cx + (v1.x - cx) * scale, cy + (v1.y - cy) * scale)
        c2 = rl.Vector2(cx + (v2.x - cx) * scale, cy + (v2.y - cy) * scale)
        c3 = rl.Vector2(cx + (v3.x - cx) * scale, cy + (v3.y - cy) * scale)
        scaled_r = r * scale

        # 計算兩點之間的垂直法向量
        def get_normal(pA, pB):
            dx = pB.x - pA.x
            dy = pB.y - pA.y
            length = math.hypot(dx, dy)
            return (-dy / length, dx / length) if length > 0 else (0, -1)

        n12 = get_normal(c1, c2)
        n23 = get_normal(c2, c3)
        n31 = get_normal(c3, c1)

        points = []

        # 計算弧線點的輪廓
        def add_arc(center, n_start, n_end):
            angle_start = math.atan2(n_start[1], n_start[0])
            angle_end = math.atan2(n_end[1], n_end[0])

            # 強制確保角度為逆時針方向 (Pyray 螢幕座標 Y 軸向下，角度應為遞減)
            while angle_end > angle_start:
                angle_end -= math.pi * 2

            steps = 6  # 圓角的平滑度 (分 6 段)
            for i in range(steps + 1):
                t = i / steps
                a = angle_start + (angle_end - angle_start) * t
                points.append(rl.Vector2(center.x + math.cos(a) * scaled_r, center.y + math.sin(a) * scaled_r))

        # 依序加入左上、底部中心、右下三個圓角的弧線
        add_arc(c1, n31, n12)
        add_arc(c2, n12, n23)
        add_arc(c3, n23, n31)

        # 透過中心點向輪廓畫出放射狀三角形 (Triangle Fan)，完美填充且不重疊
        center_pt = rl.Vector2(cx, cy)
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            rl.draw_triangle(center_pt, p1, p2, color)

    # --- 依序繪製底層、邊緣、中心 ---
    base_r = 6.0  # 控制三角形圓角平滑度的半徑參數

    # 背景黑底（放大約 1.15 倍形成外圍邊框）
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 1.15, rl.Color(0, 0, 0, 180))
    # 底部暗色邊緣（原始大小）
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 1.0, edge_color)
    # 頂部亮色中心（縮小 0.7 倍製造立體感）
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 0.7, center_color)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    # draw drop shadow
    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("MAX")
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
