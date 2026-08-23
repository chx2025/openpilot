from __future__ import annotations

from dataclasses import dataclass
import math

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import Widget


RED = rl.Color(255, 72, 72, 255)
AMBER = rl.Color(255, 190, 50, 255)
GREEN = rl.Color(53, 220, 118, 255)
LAMP_OFF = rl.Color(65, 69, 76, 220)
CARD = rl.Color(12, 15, 19, 210)
BORDER = rl.Color(255, 255, 255, 38)
TEXT = rl.Color(245, 247, 250, 255)
MUTED = rl.Color(190, 197, 205, 255)


@dataclass(frozen=True)
class TrafficSignalDisplayState:
  visible: bool = False
  has_signal: bool = False
  control_active: bool = False
  direction_unknown: bool = False
  driver_override_active: bool = False
  light_state: int = 0
  distance_m: float = 0.0
  phase: int = int(TrafficControlPhase.off)
  flashing: bool = False

  @classmethod
  def from_plan(cls, target, *, valid: bool = True) -> TrafficSignalDisplayState:
    if not valid:
      return cls()
    phase = int(target.phase)
    mode = int(target.mode)
    light = int(target.lightState)
    raw_distance = float(target.rawDistance)
    has_signal = bool(
      int(target.quality) > 0
      and 0.0 <= raw_distance <= 200.0
      and 0 <= light <= 3
      and phase != int(TrafficControlPhase.passed)
    )
    return cls(
      visible=mode == 4,
      has_signal=has_signal,
      control_active=bool(target.applied),
      direction_unknown=bool(target.directionUnknown),
      driver_override_active=bool(target.driverOverrideActive),
      light_state=light,
      distance_m=raw_distance if has_signal else 0.0,
      phase=phase,
      flashing=has_signal and phase in (
        int(TrafficControlPhase.greenFlashCandidate),
        int(TrafficControlPhase.flashingGreenStop),
      ),
    )


class TrafficControlRenderer(Widget):
  """Compact traffic-light card driven by the already-published final plan."""

  def __init__(self) -> None:
    super().__init__()
    self.state = TrafficSignalDisplayState()
    self.font = gui_app.font(FontWeight.BOLD)
    self.font_regular = gui_app.font(FontWeight.NORMAL)

  def update(self) -> None:
    sm = ui_state.sm
    if not sm.alive["longitudinalPlanSP"] or not sm.valid["longitudinalPlanSP"]:
      self.state = TrafficSignalDisplayState()
      return
    if sm.updated["longitudinalPlanSP"]:
      self.state = TrafficSignalDisplayState.from_plan(
        sm["longitudinalPlanSP"].teslaTrafficControl,
        valid=bool(sm.valid["longitudinalPlanSP"]),
      )

  def _render(self, rect: rl.Rectangle) -> None:
    if not self.state.visible:
      return

    width, height = 250.0, 112.0
    x = rect.x + rect.width / 2 + 155.0
    y = rect.y + rect.height / 4 - 158.0
    card = rl.Rectangle(x, y, width, height)
    rl.draw_rectangle_rounded(card, 0.28, 12, CARD)
    rl.draw_rectangle_rounded_lines_ex(card, 0.28, 12, 2, BORDER)

    housing = rl.Rectangle(x + 14, y + 13, 40, 86)
    rl.draw_rectangle_rounded(housing, 0.45, 10, rl.Color(0, 0, 0, 210))
    colors = (RED, AMBER, GREEN)
    states = (1, 3, 2)
    for index, (color, state) in enumerate(zip(colors, states, strict=True)):
      center = rl.Vector2(housing.x + housing.width / 2, housing.y + 17 + index * 26)
      active = self.state.has_signal and self.state.light_state == state
      if self.state.has_signal and self.state.flashing and state == 2:
        active = int(gui_app.frame / max(1, gui_app.target_fps // 2)) % 2 == 0
      if active:
        glow = 3.0 + 2.0 * math.sin(gui_app.frame / max(1, gui_app.target_fps) * math.pi)
        rl.draw_circle_v(center, 10.0 + glow, rl.Color(color.r, color.g, color.b, 42))
      rl.draw_circle_v(center, 9.0, color if active else LAMP_OFF)

    distance_text = f"{self.state.distance_m:.0f} m" if self.state.has_signal else "-- m"
    distance_size = 46
    distance_pos = rl.Vector2(x + 70, y + 19)
    rl.draw_text_ex(self.font, distance_text, distance_pos, distance_size, 0, TEXT)

    if self.state.driver_override_active:
      detail = "DRIVER OVERRIDE"
      detail_color = MUTED
    elif self.state.direction_unknown:
      detail = "DIRECTION · SHADOW"
      detail_color = AMBER
    elif not self.state.has_signal:
      detail = "NO SIGNAL"
      detail_color = MUTED
    elif self.state.flashing:
      detail = "FLASH · STOP"
      detail_color = AMBER
    elif self.state.control_active and self.state.phase in (
      int(TrafficControlPhase.approachRed), int(TrafficControlPhase.braking),
      int(TrafficControlPhase.hold), int(TrafficControlPhase.yellowStop),
    ):
      detail = "STOP"
      detail_color = RED if self.state.light_state == 1 else AMBER
    elif self.state.light_state == 2:
      detail = "GO"
      detail_color = GREEN
    elif self.state.light_state == 1:
      detail = "RED · TRACKING"
      detail_color = RED
    elif self.state.light_state == 3:
      detail = "YELLOW"
      detail_color = AMBER
    else:
      detail = "SIGNAL"
      detail_color = MUTED
    detail_size = 24
    rl.draw_text_ex(self.font_regular, detail, rl.Vector2(x + 70, y + 76), detail_size, 0, detail_color)
