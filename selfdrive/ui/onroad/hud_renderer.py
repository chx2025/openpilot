import math
import shutil
import threading
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
#from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

# DP Perf Constants (字體恢復為 50)
PERF_FONT_SIZE = 50
PERF_PADDING = 14
PERF_MARGIN_BOTTOM = 0   # 設定為 0 以貼齊底部
PERF_ITEM_GAP = 50
PERF_BG_COLOR = rl.Color(0, 0, 0, 160)


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    
    # 前車距離字串與數值變數
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # --- TDX 路況預警變數 ---
    self.tdx_speed: int = -1
    self.tdx_status: str = "UNKNOWN"
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    #self._torque_bar = TorqueBar(scale=4.0)

    # Lincoln perf overlay init
    self._perf_font = gui_app.font(FontWeight.MEDIUM)
    self._perf_stats: dict[str, str] = {"cpu_temp": "-", "mem_usage": "-", "disk_free": "-"}
    self._perf_lock = threading.Lock()
    self._perf_running = True
    self._perf_thread = threading.Thread(target=self._perf_update_loop, daemon=True)
    self._perf_thread.start()

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0
      
      # 重置 TDX 變數，避免殘留上次熄火前的資料
      self.tdx_speed = -1
      self.tdx_status = "UNKNOWN"
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']
    radar_state = sm['radarState']

    # 紀錄真實距離並格式化字串
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 (使用 try 保護，避免未訂閱時報錯) ---
    try:
      tdx = sm['tdx']
      self.tdx_speed = tdx.trafficStatus.speed
      self.tdx_status = str(tdx.trafficStatus.status)
      self.tdx_event_active = tdx.roadEvent.isActive
      self.tdx_event_desc = str(tdx.roadEvent.description)
    except Exception:
      pass # 如果系統還沒有發布 tdx 訊息，就靜默略過

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    # --- 繪製 TDX 雙行顯示資訊 ---
    self._draw_tdx_info(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    #if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
      #self._torque_bar.render(rect)

    # Draw performance info at bottom
    self._draw_performance_info(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _wrap_text(self, text: str, font: rl.Font, font_size: int, max_width: float) -> list[str]:
    """計算中英文自動換行，回傳字串陣列"""
    if not text:
      return []

    lines = []
    current_line = ""

    for char in text:
      test_line = current_line + char
      test_width = measure_text_cached(font, test_line, font_size).x
      
      # 如果加上這個字會超過最大寬度，且目前行已經有字，就斷行
      if test_width > max_width and current_line != "":
        lines.append(current_line)
        current_line = char
      else:
        current_line = test_line

    if current_line:
      lines.append(current_line)

    return lines

  # --------------------------------------------------------------------------
  # TDX 雙行顯示繪製邏輯 (支援事件自動換行)
  # --------------------------------------------------------------------------
  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """繪製高公局即時路況與事件"""
    if self.tdx_speed <= 0 and not self.tdx_event_active:
      return

    # 1. 決定車速的文字顏色
    if self.tdx_status == "GREEN":
      speed_color = rl.Color(128, 216, 166, 255)  # 順暢綠
    elif self.tdx_status == "YELLOW":
      speed_color = rl.Color(255, 204, 0, 255)    # 壅塞黃
    elif self.tdx_status == "RED":
      speed_color = rl.Color(255, 100, 100, 255)  # 塞車紅
    else:
      speed_color = rl.WHITE

    bg_padding_x = 45
    bg_padding_y = 20
    current_y = rect.y + UI_CONFIG.header_height + 25

    # ==========================================
    # 第一行: 車速 (Traffic Speed)
    # ==========================================
    if self.tdx_speed > 0:
      speed_text = f"前方車速: {self.tdx_speed} km/h"
      tdx_speed_font_size = 90
      speed_size = measure_text_cached(self._font_semi_bold, speed_text, tdx_speed_font_size)
      speed_x = rect.x + rect.width / 2 - speed_size.x / 2
      
      # 繪製半透明黑底
      bg_rect = rl.Rectangle(speed_x - bg_padding_x, current_y - bg_padding_y, speed_size.x + bg_padding_x * 2, speed_size.y + bg_padding_y * 2)
      rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 160))
      
      # 繪製文字
      rl.draw_text_ex(self._font_semi_bold, speed_text, rl.Vector2(speed_x, current_y), tdx_speed_font_size, 0, speed_color)
      
      current_y += speed_size.y + bg_padding_y * 2 + 25

    # ==========================================
    # 第二行: 事件 (Road Event - 支援自動換行)
    # ==========================================
    if self.tdx_event_active and self.tdx_event_desc:
      tdx_event_font_size = 75
      
      # 計算最大允許寬度 (螢幕寬度扣除左右各 100px 安全距離)
      max_text_width = rect.width - 200 
      
      # 將長字串切割成多行陣列
      lines = self._wrap_text(self.tdx_event_desc, self._font_semi_bold, tdx_event_font_size, max_text_width)
      
      if lines:
        # 計算單行高度與行距
        line_height = measure_text_cached(self._font_semi_bold, "測試", tdx_event_font_size).y
        line_spacing = 15
        
        # 計算多行文字的總高度與實際最大寬度
        total_text_height = len(lines) * line_height + (len(lines) - 1) * line_spacing
        actual_max_width = max([measure_text_cached(self._font_semi_bold, line, tdx_event_font_size).x for line in lines])
        
        event_x = rect.x + rect.width / 2 - actual_max_width / 2
        event_bg_rect = rl.Rectangle(
            event_x - bg_padding_x, 
            current_y - bg_padding_y, 
            actual_max_width + bg_padding_x * 2, 
            total_text_height + bg_padding_y * 2
        )
        
        # [特效] 呼吸燈閃爍警告背景
        alpha = 130 + int(50 * math.sin(time.time() * 5)) 
        rl.draw_rectangle_rounded(event_bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))
        
        # 逐行繪製置中文字
        draw_y = current_y
        for line in lines:
          line_width = measure_text_cached(self._font_semi_bold, line, tdx_event_font_size).x
          line_x = rect.x + rect.width / 2 - line_width / 2
          rl.draw_text_ex(self._font_semi_bold, line, rl.Vector2(line_x, draw_y), tdx_event_font_size, 0, rl.WHITE)
          draw_y += line_height + line_spacing

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
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

  # --------------------------------------------------------------------------
  # DP Performance Info Methods
  # --------------------------------------------------------------------------

  def _draw_performance_info(self, rect: rl.Rectangle) -> None:
    if rect.width <= 0 or rect.height <= 0:
      return

    stats = self._get_perf_stats()
    control_text = self._get_control_state_text()

    lead_dist = self.lead_dist
    cpu_temp = stats.get("cpu_temp", "-")
    mem_usage = stats.get("mem_usage", "-")
    disk_free = stats.get("disk_free", "-")

    # Order: Lead Dist -> CPU Temp -> Memory -> Disk Free -> Control
    items = [
      f"{tr('Lead Dist')} {lead_dist}",
      f"{tr('CPU Temp')} {cpu_temp}",
      f"{tr('Memory')} {mem_usage}",
      f"{tr('Disk Free')} {disk_free}",
      f"{control_text}",
    ]

    measurements = [measure_text_cached(self._perf_font, text, PERF_FONT_SIZE) for text in items]

    # 設定整個狀態列的總寬度 (稍微留點邊距)
    bar_width = max(rect.width - 20, 0)
    bar_height = PERF_FONT_SIZE + 2 * PERF_PADDING

    bar_x = rect.x + (rect.width - bar_width) / 2
    # Apply 0 margin to stick to bottom
    bar_y = rect.y + rect.height - bar_height - PERF_MARGIN_BOTTOM
    minimum_y = rect.y + PERF_MARGIN_BOTTOM
    if bar_y < minimum_y:
      bar_y = minimum_y

    rl.draw_rectangle_rounded(
      rl.Rectangle(bar_x, bar_y, bar_width, bar_height),
      0.2,
      8,
      PERF_BG_COLOR,
    )

    # 將總寬度均分為等寬的 5 個欄位 (slots)
    slot_width = bar_width / len(items)
    text_y = bar_y + PERF_PADDING

    for i, (text, measurement) in enumerate(zip(items, measurements)):
      # 精準置中演算法
      cursor_x = bar_x + (i * slot_width) + (slot_width - measurement.x) / 2

      text_color = rl.WHITE
      # 動態決定前車距離的顏色 (小於 15 公尺時顯示橘色)
      if i == 0 and self.lead_dist != "-" and self.lead_dist_raw < 15.0:
        text_color = rl.Color(255, 188, 0, 200)
      # 當控制狀態不為自動巡航時，文字改為黃色
      elif i == 4 and ui_state.status != UIStatus.ENGAGED:
        text_color = rl.YELLOW

      rl.draw_text_ex(self._perf_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, text_color)

  def _get_control_state_text(self) -> str:
    status = ui_state.status
    if status == UIStatus.ENGAGED:
      return tr("Auto control")
    return tr("Manual control")

  def _get_perf_stats(self) -> dict[str, str]:
    with self._perf_lock:
      return dict(self._perf_stats)

  def _perf_update_loop(self) -> None:
    time.sleep(10)
    while self._perf_running:
      stats = {
        "cpu_temp": self._read_cpu_temp(),
        "mem_usage": self._read_mem_usage(),
        "disk_free": self._read_disk_free(),
      }
      with self._perf_lock:
        self._perf_stats.update(stats)
      for _ in range(10):
        if not self._perf_running:
          return
        time.sleep(0.1)

  @staticmethod
  def _read_cpu_temp() -> str:
    path = "/sys/class/thermal/thermal_zone0/temp"
    try:
      with open(path) as f:
        temp_c = int(f.read().strip()) / 1000.0
        return f"{temp_c:.0f}°C"
    except Exception:
      return "-"

  @staticmethod
  def _read_mem_usage() -> str:
    try:
      total_kb = None
      available_kb = None
      with open("/proc/meminfo") as f:
        for line in f:
          if line.startswith("MemTotal:"):
            total_kb = float(line.split()[1])
          elif line.startswith("MemAvailable:"):
            available_kb = float(line.split()[1])
          if total_kb is not None and available_kb is not None:
            break
      if total_kb and available_kb:
        used_pct = (total_kb - available_kb) / total_kb * 100.0
        used_pct = min(max(used_pct, 0.0), 100.0)
        return f"{used_pct:.0f}%"
    except Exception:
      pass
    return "-"

  @staticmethod
  def _read_disk_free() -> str:
    try:
      usage = shutil.disk_usage("/data")
      free_gb = usage.free / (1024 ** 3)
      if free_gb >= 1.0:
        return f"{free_gb:.1f}G" # GB 改為 G
      free_mb = usage.free / (1024 ** 2)
      return f"{free_mb:.0f}MB"   #
    except Exception:
      return "-"
