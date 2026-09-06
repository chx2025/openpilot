"""Device home card backed by the same vehicle snapshot as the local console."""
from concurrent.futures import ThreadPoolExecutor
import time

import pyray as rl
import requests

from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label

URL = "http://127.0.0.1:8088"


def _request(path):
  response = requests.get(URL + path, timeout=2)
  data = response.json()
  if not response.ok:
    raise RuntimeError(data.get("message", "车辆服务不可用"))
  return data


class TeslaVehicleWidget(Widget):
  def __init__(self):
    super().__init__()
    self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vehicle-home")
    self.fetch_future = None
    self.next_fetch = 0.0
    self.data = {}
    self.error = "正在读取车辆信息…"
    self.details_open = False
    self.info_rect = rl.Rectangle(0, 0, 0, 0)

  def _update_state(self):
    if self.fetch_future is not None and self.fetch_future.done():
      try:
        self.data = self.fetch_future.result()
        self.error = ""
      except Exception:
        self.data = {}
        self.error = "车辆服务未连接，等待 CAN 数据"
      self.fetch_future = None
    if self.fetch_future is None and time.monotonic() >= self.next_fetch:
      self.fetch_future = self.executor.submit(_request, "/api/vehicle")
      self.next_fetch = time.monotonic() + 1

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    if rl.check_collision_point_rec(mouse_pos, self.info_rect):
      self.details_open = not self.details_open
      return
    if self.details_open:
      self.details_open = False

  def _render(self, rect):
    bg, panel = rl.Color(17, 20, 24, 255), rl.Color(26, 30, 35, 255)
    white, muted = rl.Color(238, 241, 243, 255), rl.Color(136, 147, 158, 255)
    green, red = rl.Color(110, 226, 180, 255), rl.Color(245, 97, 108, 255)
    sx, sy = rect.width / 1740, rect.height / 900
    scale = min(sx, sy)
    ox, oy = rect.x, rect.y
    def box(x, y, w, h):
      return rl.Rectangle(ox + x * sx, oy + y * sy, w * sx, h * sy)
    def point(x, y):
      return rl.Vector2(ox + x * sx, oy + y * sy)
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

    rl.draw_rectangle_rounded(rect, .045, 18, bg)
    v = self.data.get("vehicle", {})
    text("TESLA", 46, 24, 260, 38, height=60)
    text("IP " + (self.data.get("device_ip") or "—"), 760, 26, 635, 32, muted, rl.GuiTextAlignment.TEXT_ALIGN_RIGHT, 54)
    alive = not self.error and any(v.get(key) not in (None, "—") for key in ("soc", "odometer"))
    rl.draw_circle_v(point(1460, 52), 6 * scale, green if alive else muted)
    text("车辆信息" if alive else "等待车辆", 1480, 26, 160, 27, muted, height=54)
    self.info_rect = box(1648, 20, 64, 64)
    rl.draw_circle_lines(int(ox + 1680 * sx), int(oy + 52 * sy), 20 * scale, muted)
    text("i", 1650, 21, 60, 34, muted, rl.GuiTextAlignment.TEXT_ALIGN_CENTER, 62)

    icon("battery", 53, 126, green)
    text("电量", 101, 111, 170, 34, muted, height=60)
    soc = v.get("soc", "—").replace(" %", "")
    text(soc + (" %" if soc != "—" else ""), 46, 175, 450, 132, height=156)
    rounded(54, 356, 414, 8, rl.Color(45, 51, 58, 255))
    try:
      fraction = max(0, min(1, float(soc) / 100))
    except ValueError:
      fraction = 0
    if fraction:
      rounded(54, 356, 414 * fraction, 8, green)
    text("续航", 54, 390, 220, 31, muted, height=54)
    text("—", 48, 432, 160, 72, height=88)
    text("单位待核实" if v.get("range") == "待核实单位" else "等待数据", 180, 450, 280, 26, muted, height=48)
    high_voltage = v.get("high_voltage", {})
    text("高压电池", 54, 528, 240, 31, muted, height=54)
    for label, value, x in (("电压", high_voltage.get("voltage", "—"), 54),
                            ("电流", high_voltage.get("current", "—"), 202),
                            ("功率", high_voltage.get("power", "—"), 350)):
      text(label, x, 579, 130, 24, muted, height=40)
      text(value, x, 615, 135, 31, height=50)

    # A large central vehicle keeps wheel positions readable across the full home area.
    cx, cy, car_scale = 868, 392, 1.8
    def car_box(x, y, w, h, color, radius=.15):
      rounded(cx + x * car_scale, cy + y * car_scale, w * car_scale, h * car_scale, color, radius)
    def car_polygon(coords, color):
      polygon([(cx + x * car_scale, cy + y * car_scale) for x, y in coords], color)
    def car_line(x1, y1, x2, y2, color, thickness=2):
      line(cx + x1 * car_scale, cy + y1 * car_scale, cx + x2 * car_scale, cy + y2 * car_scale, color, thickness)
    car_box(-63, -130, 126, 255, rl.Color(70, 78, 86, 255), .8)
    car_box(-58, -126, 116, 247, rl.Color(178, 187, 196, 255), .8)
    car_polygon([(-48, -68), (48, -68), (37, -30), (-37, -30)], bg)
    car_box(-37, -28, 74, 82, rl.Color(38, 44, 51, 255), .18)
    car_polygon([(-36, 57), (36, 57), (46, 86), (-46, 86)], bg)
    car_line(-40, -104, 40, -104, rl.Color(221, 230, 235, 255), 5)
    car_line(-40, 106, -14, 108, red, 5)
    car_line(14, 108, 40, 106, red, 5)
    for dx in (-67, 57):
      for dy in (-76, 64):
        car_box(dx, dy, 10, 39, rl.Color(5, 7, 9, 255), .45)
    for dx in (-53, 53):
      car_line(dx, -39, dx, 49, rl.Color(113, 129, 143, 255), 3)
    wheels = v.get("pressure", [{"label": n, "text": "—"} for n in ("左前", "右前", "左后", "右后")])
    for i, wheel in enumerate(wheels):
      x, y = (553 if i % 2 == 0 else 1042), (212 if i < 2 else 482)
      color = red if wheel.get("warning") else white
      text(wheel["label"], x, y - 10, 155, 30, muted, height=48)
      text(wheel["text"].replace(" bar", ""), x - 3, y + 43, 173, 68, color, height=86)
      text("bar", x, y + 126, 130, 29, muted, height=48)
      text("电池 " + wheel.get("battery", "—"), x, y + 168, 190, 25, muted, height=42)
    for x in (512, 1232):
      line(x, 143, x, 646, rl.Color(46, 51, 58, 255), 1)
    for kind, label, value, unit, y in (
      ("road", "总里程", v.get("odometer", "—"), "km", 116),
      ("bolt", "累计放电", v.get("discharge", "—"), "kWh", 315),
      ("bolt", "累计充电", v.get("charge", "—"), "kWh", 514),
    ):
      icon(kind, 1272, y + 6, muted)
      text(label, 1320, y - 11, 325, 34, muted, height=60)
      text(value.replace(" " + unit, ""), 1268, y + 55, 415, 64, height=82)
      text(unit, 1274, y + 139, 380, 27, muted, height=44)

    rounded(32, 724, 1676, 132, panel, .18)
    text("刹车温度", 62, 739, 230, 34, muted, height=54)
    for i, brake in enumerate(v.get("brakes", [])):
      x = 338 + i * 332
      text(brake.get("label", "—"), x, 739, 135, 25, muted, height=42)
      text(brake.get("text", "—"), x, 778, 240, 43, height=60)
    if self.error:
      text("等待车辆连接", 48, 866, 1600, 26, muted, height=32)

    if self.details_open:
      rounded(32, 106, 1676, 575, rl.Color(35, 40, 47, 255), .09)
      text("更多信息", 76, 126, 1400, 48, height=74)
      lines = [v.get("range_note", "等待原车续航信号"),
               "高压状态  " + high_voltage.get("hv_state", "—") + " · 接触器 " + high_voltage.get("contactor", "—"),
               "电池状态  " + high_voltage.get("state", "—") + " · 充电 " + high_voltage.get("charge_status", "—"),
               "累计放电  " + v.get("discharge", "—"), "累计充电  " + v.get("charge", "—"),
               "电耗：未发现可信原厂平均值，已取消显示"]
      for i, value in enumerate(lines):
        text(value, 78, 216 + i * 74, 1580, 36, muted, height=66)
