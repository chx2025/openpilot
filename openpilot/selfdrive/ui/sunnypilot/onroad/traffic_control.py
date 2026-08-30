from __future__ import annotations

from dataclasses import dataclass
import math
import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


RED = rl.Color(255, 72, 72, 255)
AMBER = rl.Color(255, 190, 50, 255)
GREEN = rl.Color(53, 220, 118, 255)
LAMP_OFF = rl.Color(65, 69, 76, 220)
CARD = rl.Color(12, 15, 19, 160)
BORDER = rl.Color(255, 255, 255, 38)
TEXT = rl.Color(245, 247, 250, 255)
MUTED = rl.Color(190, 197, 205, 255)
CONTROL_BADGE = rl.Color(36, 112, 184, 230)
TRAFFIC_CARD_WIDTH = 940.0
TRAFFIC_CARD_HEIGHT = 240.0
TRAFFIC_CARD_TOP_OFFSET = 47.0
TRAFFIC_DISTANCE_FONT_SIZE = 82
TRAFFIC_DETAIL_FONT_SIZE = 52
TRAFFIC_SOURCE_FONT_SIZE = 36
TRAFFIC_SOURCE_BADGE_HEIGHT = 58.0
TRAFFIC_SOURCE_BADGE_PADDING = 22.0
TRAFFIC_LIGHT_HOUSING_WIDTH = 92.0
TRAFFIC_LIGHT_HOUSING_HEIGHT = 192.0
TRAFFIC_LIGHT_RADIUS = 22.0
TRAFFIC_TEXT_X_OFFSET = 140.0
TRAFFIC_ACTION_NOTICE_HOLD_S = 2.0
TRAFFIC_ACTIONS = (1, 2, 3, 4, 5)
TRAFFIC_STOP_NOTICE_PHASES = (
  int(TrafficControlPhase.approachRed), int(TrafficControlPhase.braking),
  int(TrafficControlPhase.hold), int(TrafficControlPhase.flashingGreenStop),
  int(TrafficControlPhase.yellowStop),
)


def traffic_card_rect(rect: rl.Rectangle) -> rl.Rectangle:
  return rl.Rectangle(
    rect.x + 46.0,
    rect.y + UI_CONFIG.header_height + TRAFFIC_CARD_TOP_OFFSET,
    TRAFFIC_CARD_WIDTH,
    TRAFFIC_CARD_HEIGHT,
  )


@dataclass(frozen=True)
class TrafficSignalDisplayState:
  visible: bool = False
  has_signal: bool = False
  control_active: bool = False
  driver_override_active: bool = False
  raw_observation_fresh: bool = False
  light_state: int = 0
  distance_m: float = 0.0
  phase: int = int(TrafficControlPhase.off)
  flashing: bool = False
  action: int = 0
  should_stop: bool = False
  stop_session_id: int = 0

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
      driver_override_active=bool(target.driverOverrideActive),
      raw_observation_fresh=bool(target.rawObservationFresh),
      light_state=light,
      distance_m=raw_distance if has_signal else 0.0,
      phase=phase,
      flashing=has_signal and phase == int(TrafficControlPhase.flashingGreenStop),
      action=int(target.action),
      should_stop=bool(target.shouldStop),
      stop_session_id=int(target.stopSessionId),
    )


@dataclass(frozen=True)
class TrafficActionNotice:
  state: TrafficSignalDisplayState
  action: int
  recent: bool = False


class TrafficActionNoticeLatch:
  def __init__(self) -> None:
    self._action = 0
    self._session_id = 0
    self._green_release = False
    self._expires_at_s = 0.0
    self._recent = False

  def update(self, state: TrafficSignalDisplayState, *, now_s: float) -> TrafficActionNotice | None:
    context_active = bool(
      state.visible
      and not state.driver_override_active
      and state.phase not in (int(TrafficControlPhase.off), int(TrafficControlPhase.passed))
    )
    if not context_active:
      self.clear()
      return None

    if state.control_active and state.action in TRAFFIC_ACTIONS:
      self._action = state.action
      self._session_id = state.stop_session_id
      self._green_release = bool(
        state.action == 4 and state.light_state == 2
        and state.phase == int(TrafficControlPhase.release)
      )
      self._expires_at_s = now_s + TRAFFIC_ACTION_NOTICE_HOLD_S
      self._recent = False
      return TrafficActionNotice(state, state.action)

    compatible = bool(
      self._action in TRAFFIC_ACTIONS
      and now_s <= self._expires_at_s
      and state.has_signal
      and state.raw_observation_fresh
      and state.stop_session_id > 0
      and state.stop_session_id == self._session_id
      and self._signal_compatible(state)
    )
    if compatible:
      self._recent = True
      return TrafficActionNotice(state, self._action, recent=True)

    self.clear()
    return None

  def clear(self) -> None:
    self._action = 0
    self._session_id = 0
    self._green_release = False
    self._expires_at_s = 0.0
    self._recent = False

  def current(self, state: TrafficSignalDisplayState, *, now_s: float) -> TrafficActionNotice | None:
    if self._action in TRAFFIC_ACTIONS and now_s <= self._expires_at_s:
      return TrafficActionNotice(state, self._action, recent=self._recent)
    self.clear()
    return None

  def _signal_compatible(self, current: TrafficSignalDisplayState) -> bool:
    if self._action in (1, 2):
      return bool(
        current.phase in TRAFFIC_STOP_NOTICE_PHASES
        and (current.flashing or current.light_state in (1, 3))
      )
    if self._action in (3, 5):
      return bool(
        current.phase == int(TrafficControlPhase.release)
        and current.light_state == 2 and not current.should_stop
      )
    if self._action == 4:
      if self._green_release:
        return bool(current.phase == int(TrafficControlPhase.release) and current.light_state == 2)
      return bool(current.phase in TRAFFIC_STOP_NOTICE_PHASES and current.light_state in (1, 3))
    return False


def traffic_action_text(state: TrafficSignalDisplayState) -> tuple[str, rl.Color]:
  # Only describe actions that Traffic Control actually applied to the final
  # longitudinal plan. Raw observations, blocked GO requests, and base-planner
  # stops remain visible through the lamp/distance without claiming ownership.
  if not state.control_active:
    return "", MUTED
  if not state.raw_observation_fresh:
    if state.action == 1:
      return tr("Signal lost · slowing continues"), AMBER
    if state.action == 2:
      return tr("Signal lost · holding stop"), AMBER
    if state.action == 3:
      return tr("Green · auto start"), GREEN
    if state.action == 4:
      return tr("Signal lost · smooth transition"), AMBER
    if state.action == 5:
      return tr("Green · continuing"), GREEN
    return "", MUTED
  if state.action == 1:
    if state.flashing or state.light_state == 3:
      return tr("Signal changing · stopping"), AMBER
    if state.light_state == 1:
      return tr("Red · slowing to stop"), RED
    return tr("Signal changing · stopping"), AMBER
  if state.action == 2:
    if state.light_state == 1:
      return tr("Red · holding"), RED
    if state.light_state == 2:
      return tr("Green · confirming release"), AMBER
    return tr("Signal changing · stopping"), AMBER
  if state.action == 3:
    return tr("Green · auto start"), GREEN
  if state.action == 4:
    if state.light_state == 2 and state.phase == int(TrafficControlPhase.release):
      return tr("Green · releasing brakes"), GREEN
    return tr("Stop control · smooth transition"), AMBER
  if state.action == 5:
    return tr("Green · continuing"), GREEN
  return "", MUTED


def traffic_notice_text(notice: TrafficActionNotice) -> tuple[str, rl.Color]:
  if not notice.recent:
    return traffic_action_text(notice.state)
  state = notice.state
  if notice.action in (1, 2):
    if state.light_state == 1:
      return tr("Red · stop control applied"), RED
    return tr("Signal changing · stop applied"), AMBER
  if notice.action == 3:
    return tr("Green · auto start applied"), GREEN
  if notice.action == 4:
    if state.light_state == 2 and state.phase == int(TrafficControlPhase.release):
      return tr("Green · stop control released"), GREEN
    return tr("Stop control · transition applied"), AMBER
  if notice.action == 5:
    return tr("Green · continuing applied"), GREEN
  return "", MUTED


class TrafficControlRenderer(Widget):
  """Compact traffic-light card driven by the already-published final plan."""

  def __init__(self) -> None:
    super().__init__()
    self.state = TrafficSignalDisplayState()
    self.notice: TrafficActionNotice | None = None
    self._notice_latch = TrafficActionNoticeLatch()
    self.font = gui_app.font(FontWeight.BOLD)
    self.font_regular = gui_app.font(FontWeight.NORMAL)

  def update(self) -> None:
    sm = ui_state.sm
    now_s = time.monotonic()
    if not sm.alive["longitudinalPlanSP"] or not sm.valid["longitudinalPlanSP"]:
      self.state = TrafficSignalDisplayState()
      self.notice = None
      self._notice_latch.clear()
      return
    if sm.updated["longitudinalPlanSP"]:
      self.state = TrafficSignalDisplayState.from_plan(
        sm["longitudinalPlanSP"].teslaTrafficControl,
        valid=bool(sm.valid["longitudinalPlanSP"]),
      )
      self.notice = self._notice_latch.update(self.state, now_s=now_s)
    else:
      self.notice = self._notice_latch.current(self.state, now_s=now_s)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self.state.visible:
      return

    # Keep the signal card close to the header so it remains visible above the
    # expanded eGPU diagnostics on the left side.
    card = traffic_card_rect(rect)
    x, y = card.x, card.y
    rl.draw_rectangle_rounded(card, 0.28, 12, CARD)
    rl.draw_rectangle_rounded_lines_ex(card, 0.28, 12, 2, BORDER)

    housing = rl.Rectangle(x + 22, y + 24, TRAFFIC_LIGHT_HOUSING_WIDTH, TRAFFIC_LIGHT_HOUSING_HEIGHT)
    rl.draw_rectangle_rounded(housing, 0.45, 10, rl.Color(0, 0, 0, 190))
    colors = (RED, AMBER, GREEN)
    states = (1, 3, 2)
    for index, (color, state) in enumerate(zip(colors, states, strict=True)):
      center = rl.Vector2(housing.x + housing.width / 2, housing.y + 40 + index * 56)
      active = self.state.has_signal and self.state.light_state == state
      if self.state.has_signal and self.state.flashing and state == 2:
        active = int(gui_app.frame / max(1, gui_app.target_fps // 2)) % 2 == 0
      if active:
        glow = 6.0 + 4.0 * math.sin(gui_app.frame / max(1, gui_app.target_fps) * math.pi)
        rl.draw_circle_v(center, 20.0 + glow, rl.Color(color.r, color.g, color.b, 42))
      rl.draw_circle_v(center, TRAFFIC_LIGHT_RADIUS, color if active else LAMP_OFF)

    distance_text = f"{self.state.distance_m:.0f} m" if self.state.has_signal else "-- m"
    distance_size = TRAFFIC_DISTANCE_FONT_SIZE
    distance_pos = rl.Vector2(x + TRAFFIC_TEXT_X_OFFSET, y + 22)
    rl.draw_text_ex(self.font, distance_text, distance_pos, distance_size, 0, TEXT)

    source = tr("Tesla Traffic Control") if self.notice is not None else ""
    if source:
      source_size = measure_text_cached(self.font_regular, source, TRAFFIC_SOURCE_FONT_SIZE, 0)
      badge_width = source_size.x + 2.0 * TRAFFIC_SOURCE_BADGE_PADDING
      badge = rl.Rectangle(
        card.x + card.width - badge_width - 22.0,
        y + 32.0,
        badge_width,
        TRAFFIC_SOURCE_BADGE_HEIGHT,
      )
      rl.draw_rectangle_rounded(badge, 0.45, 8, CONTROL_BADGE)
      rl.draw_text_ex(
        self.font_regular,
        source,
        rl.Vector2(
          badge.x + TRAFFIC_SOURCE_BADGE_PADDING,
          badge.y + (badge.height - source_size.y) / 2.0,
        ),
        TRAFFIC_SOURCE_FONT_SIZE,
        0,
        TEXT,
      )

    detail, detail_color = traffic_notice_text(self.notice) if self.notice is not None else ("", MUTED)
    if detail:
      rl.draw_text_ex(
        self.font_regular,
        detail,
        rl.Vector2(x + TRAFFIC_TEXT_X_OFFSET, y + 154),
        TRAFFIC_DETAIL_FONT_SIZE,
        0,
        detail_color,
      )
