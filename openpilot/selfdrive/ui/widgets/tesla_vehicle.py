"""Device home card backed by the same vehicle snapshot as the local console."""
from concurrent.futures import ThreadPoolExecutor
import time

import pyray as rl
import requests

from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label

URL = "http://127.0.0.1:8088"


def _request(path, payload=None):
  if payload is None:
    response = requests.get(URL + path, timeout=2)
  else:
    response = requests.post(URL + path, json=payload, timeout=10)
  data = response.json()
  if not response.ok:
    raise RuntimeError(data.get("message", "车辆服务不可用"))
  return data


class TeslaVehicleWidget(Widget):
  def __init__(self):
    super().__init__()
    self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vehicle-home")
    self.fetch_future = None
    self.test_future = None
    self.next_fetch = 0.0
    self.data = {}
    self.error = "正在读取车辆信息…"
    self.result = ""
    self.details_open = False
    self.active_side = None
    self.info_rect = rl.Rectangle(0, 0, 0, 0)
    self.buttons = {}

  def _update_state(self):
    if self.test_future is not None and self.test_future.done():
      try:
        result = self.test_future.result()
        self.result = {"sent": "测试结束 · 请观察灯带", "no_echo": "未收到回显", "rejected": "发送被拒绝",
                       "blocked": "测试已停止", "timeout": "测试超时"}.get(result.get("state"), "测试结束")
        self.result_detail = result.get("message", "未知发送结果")
      except Exception as error:
        self.result = "连接失败"
        self.result_detail = str(error)
      self.test_future = None
      self.active_side = None
    if self.fetch_future is not None and self.fetch_future.done():
      try:
        self.data = self.fetch_future.result()
        self.error = ""
      except Exception:
        self.data = {}
        self.error = "车辆服务未连接，等待 CAN 数据"
      self.fetch_future = None
    if self.fetch_future is None and self.test_future is None and time.monotonic() >= self.next_fetch:
      self.fetch_future = self.executor.submit(_request, "/api/vehicle")
      self.next_fetch = time.monotonic() + 1

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    if rl.check_collision_point_rec(mouse_pos, self.info_rect):
      self.details_open = not self.details_open
      return
    if self.details_open:
      self.details_open = False
      return
    if not self.data.get("ambient_test_ready") or self.test_future is not None:
      return
    for side, rect in self.buttons.items():
      if rl.check_collision_point_rec(mouse_pos, rect):
        self.result = "正在发送 · 3 秒"
        self.active_side = side
        self.test_future = self.executor.submit(_request, "/api/tesla/ambient", {"side": side})
        break

  def _render(self, rect):
    bg, panel = rl.Color(17, 20, 24, 255), rl.Color(26, 30, 35, 255)
    white, muted = rl.Color(238, 241, 243, 255), rl.Color(136, 147, 158, 255)
    green, red = rl.Color(110, 226, 180, 255), rl.Color(245, 97, 108, 255)
    scale = min(rect.width / 980, rect.height / 780)
    ox, oy = rect.x + (rect.width - 980 * scale) / 2, rect.y + (rect.height - 780 * scale) / 2
    def box(x, y, w, h):
      return rl.Rectangle(ox + x * scale, oy + y * scale, w * scale, h * scale)
    def point(x, y):
      return rl.Vector2(ox + x * scale, oy + y * scale)
    def rounded(x, y, w, h, color, radius=.15):
      rl.draw_rectangle_rounded(box(x, y, w, h), radius, 16, color)
    def line(x1, y1, x2, y2, color=muted, thickness=2):
      rl.draw_line_ex(point(x1, y1), point(x2, y2), thickness * scale, color)
    def text(value, x, y, w=400, size=30, color=white, align=rl.GuiTextAlignment.TEXT_ALIGN_LEFT, height=46):
      gui_label(box(x, y, w, height), value, size * scale, color=color, font_weight=FontWeight.NORMAL, alignment=align)
    def polygon(coords, color):
      for i in range(1, len(coords) - 1):
        a, b, c = coords[0], coords[i], coords[i + 1]
        if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0:
          b, c = c, b
        rl.draw_triangle(point(*a), point(*b), point(*c), color)
    def icon(kind, x, y, color=muted):
      if kind == "battery":
        rl.draw_rectangle_rounded_lines_ex(box(x, y + 4, 29, 18), .2, 8, 2 * scale, color)
        rounded(x + 31, y + 9, 3, 8, color)
        rounded(x + 4, y + 8, 17, 10, color)
      elif kind == "bolt":
        polygon([(x + 18, y), (x + 5, y + 17), (x + 16, y + 17)], color)
        polygon([(x + 13, y + 11), (x + 26, y + 11), (x + 10, y + 30)], color)
      elif kind == "road":
        line(x + 9, y, x + 4, y + 28, color)
        line(x + 24, y, x + 29, y + 28, color)
        line(x + 16, y + 1, x + 16, y + 9, color)
        line(x + 16, y + 19, x + 16, y + 27, color)
      elif kind == "lamp":
        rl.draw_circle_lines(int(ox + (x + 15) * scale), int(oy + (y + 11) * scale), 9 * scale, color)
        line(x + 10, y + 21, x + 20, y + 21, color)
        line(x + 12, y + 26, x + 18, y + 26, color)
      elif kind == "arrow":
        line(x, y + 12, x + 23, y + 12, color)
        line(x + 15, y + 4, x + 23, y + 12, color)
        line(x + 15, y + 20, x + 23, y + 12, color)

    rl.draw_rectangle_rounded(rect, .045, 18, bg)
    v = self.data.get("vehicle", {})
    text("TESLA", 36, 22, 200, 27)
    alive = not self.error and any(v.get(key) not in (None, "—") for key in ("soc", "odometer", "consumption"))
    rl.draw_circle_v(point(805, 46), 4 * scale, green if alive else muted)
    text("车辆信息" if alive else "等待车辆", 819, 23, 100, 19, muted)
    self.info_rect = box(928, 24, 30, 38)
    rl.draw_circle_lines(int(ox + 943 * scale), int(oy + 43 * scale), 12 * scale, muted)
    text("i", 930, 24, 26, 22, muted, rl.GuiTextAlignment.TEXT_ALIGN_CENTER, 38)

    icon("battery", 38, 100, green)
    text("电量", 84, 90, 150, 23, muted)
    soc = v.get("soc", "—").replace(" %", "")
    text(soc + (" %" if soc != "—" else ""), 34, 129, 490, 88, height=105)
    rounded(40, 251, 490, 5, rl.Color(45, 51, 58, 255))
    try:
      fraction = max(0, min(1, float(soc) / 100))
    except ValueError:
      fraction = 0
    if fraction:
      rounded(40, 251, 490 * fraction, 5, green)
    text("续航", 610, 103, 120, 23, muted)
    text("—", 607, 147, 190, 64, height=82)
    text("单位待核实" if v.get("range") == "待核实单位" else "等待数据", 610, 222, 250, 21, muted)

    # Top-down vehicle with four values positioned by their physical wheels.
    cx, cy = 286, 421
    rounded(cx - 63, cy - 130, 126, 255, rl.Color(70, 78, 86, 255), .8)
    rounded(cx - 58, cy - 126, 116, 247, rl.Color(178, 187, 196, 255), .8)
    polygon([(cx - 48, cy - 68), (cx + 48, cy - 68), (cx + 37, cy - 30), (cx - 37, cy - 30)], bg)
    rounded(cx - 37, cy - 28, 74, 82, rl.Color(38, 44, 51, 255), .18)
    polygon([(cx - 36, cy + 57), (cx + 36, cy + 57), (cx + 46, cy + 86), (cx - 46, cy + 86)], bg)
    line(cx - 40, cy - 104, cx + 40, cy - 104, rl.Color(221, 230, 235, 255), 3)
    line(cx - 40, cy + 106, cx - 14, cy + 108, red, 3)
    line(cx + 14, cy + 108, cx + 40, cy + 106, red, 3)
    for dx in (-67, 57):
      for dy in (-76, 64):
        rounded(cx + dx, cy + dy, 10, 39, rl.Color(5, 7, 9, 255), .45)
    for side, dx in (("left", -53), ("right", 53)):
      line(cx + dx, cy - 39, cx + dx, cy + 49, red if self.active_side == side else rl.Color(113, 129, 143, 255),
           5 if self.active_side == side else 2)
    wheels = v.get("pressure", [{"label": n, "text": "—"} for n in ("左前", "右前", "左后", "右后")])
    for i, wheel in enumerate(wheels):
      x, y = (40 if i % 2 == 0 else 405), (313 if i < 2 else 476)
      color = red if wheel.get("warning") else white
      text(wheel["label"], x, y - 10, 130, 19, muted)
      text(wheel["text"].replace(" bar", ""), x - 2, y + 24, 135, 40, color)
      text("bar", x, y + 64, 120, 19, muted)
    line(570, 307, 570, 562, rl.Color(46, 51, 58, 255), 1)
    for kind, label, value, unit, y in (
      ("road", "总里程", v.get("odometer", "—"), "km", 317),
      ("bolt", "观测电耗", v.get("consumption", "—"), "kWh/100 km", 447),
    ):
      icon(kind, 610, y, muted)
      text(label, 656, y - 11, 240, 23, muted)
      text(value.replace(" " + unit, ""), 607, y + 32, 330, 47, height=66)
      text(unit, 611, y + 88, 300, 20, muted)

    rounded(26, 604, 928, 147, panel, .18)
    icon("lamp", 45, 626, red)
    text("氛围灯", 88, 612, 220, 26)
    text("3 秒", 851, 613, 78, 21, muted, rl.GuiTextAlignment.TEXT_ALIGN_RIGHT)
    enabled = self.data.get("ambient_test_ready") and self.test_future is None
    self.buttons = {}
    for i, (side, title) in enumerate((("left", "左侧"), ("right", "右侧"))):
      x = 43 + i * 453
      button = box(x, 671, 437, 62)
      self.buttons[side] = button
      active = self.active_side == side
      rounded(x, 671, 437, 62, rl.Color(92, 36, 43, 255) if active else rl.Color(43, 36, 41, 255), .25)
      rl.draw_circle_v(point(x + 26, 702), 5 * scale, red if enabled or active else muted)
      text(title, x + 45, 679, 280, 29, white if enabled or active else muted)
      icon("arrow", x + 387, 689, red if enabled or active else muted)
    if self.result or self.error:
      text(self.result if self.result else "等待车辆连接", 36, 751, 900, 19, muted, height=26)
    elif not enabled:
      text("P 挡静止后可测试", 36, 751, 900, 19, muted, height=26)

    if self.details_open:
      rounded(26, 80, 928, 500, rl.Color(35, 40, 47, 255), .09)
      text("更多信息", 52, 102, 600, 34)
      lines = [v.get("range_note", "等待原车续航信号"),
               "累计放电  " + v.get("discharge", "—"), "累计充电  " + v.get("charge", "—"),
               "电耗：连续观测至少 1 km 后显示", "测试：P 挡静止 / 原车灯开启 / onroad",
               getattr(self, "result_detail", "回显不代表变色；请观察左右灯带")]
      for i, value in enumerate(lines):
        text(value, 54, 172 + i * 58, 860, 25, muted)
