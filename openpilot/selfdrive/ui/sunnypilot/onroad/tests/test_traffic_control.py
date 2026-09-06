from types import SimpleNamespace
import pyray as rl
import pytest

import openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control as traffic_control_module
from openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control import (
  TrafficSignalDisplayState,
  TRAFFIC_CARD_WIDTH,
  TRAFFIC_CARD_HEIGHT,
  TRAFFIC_LIGHT_HOUSING_HEIGHT,
  TRAFFIC_LIGHT_HOUSING_WIDTH,
  TRAFFIC_LIGHT_RADIUS,
  traffic_control_highlighted,
  traffic_card_rect,
)
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase


def target(*, light=1, raw_distance=80.0, remaining=35.0, reference=5.0,
           phase=TrafficControlPhase.braking, quality=2, mode=4, applied=False,
           driver_override=False, action=0, raw_fresh=True):
  return SimpleNamespace(
    lightState=light,
    mode=mode,
    rawDistance=raw_distance,
    applied=applied,
    driverOverrideActive=driver_override,
    rawObservationFresh=raw_fresh,
    remainingDistance=remaining,
    stopReference=reference,
    phase=int(phase),
    quality=quality,
    action=action,
  )


def display_state(**kwargs):
  return TrafficSignalDisplayState.from_plan(target(**kwargs))


class FakeSubMaster(dict):
  def __init__(self, traffic_target):
    super().__init__(longitudinalPlanSP=SimpleNamespace(teslaTrafficControl=traffic_target))
    self.alive = {"longitudinalPlanSP": True}
    self.valid = {"longitudinalPlanSP": True}
    self.updated = {"longitudinalPlanSP": True}


def make_renderer(monkeypatch, traffic_target):
  fake_sm = FakeSubMaster(traffic_target)
  monkeypatch.setattr(traffic_control_module, "gui_app", SimpleNamespace(font=lambda _weight: object()))
  monkeypatch.setattr(traffic_control_module.ui_state, "sm", fake_sm)
  return traffic_control_module.TrafficControlRenderer(), fake_sm


def test_view_model_displays_actual_signal_color():
  state = TrafficSignalDisplayState.from_plan(target())
  assert state.visible
  assert state.has_signal
  assert state.light_state == 1
  assert not state.flashing


def test_view_model_marks_flashing_green_stop():
  state = TrafficSignalDisplayState.from_plan(target(
    light=0, phase=TrafficControlPhase.flashingGreenStop,
  ))
  assert state.visible
  assert state.flashing


def test_explicit_off_animates_only_after_flash_confirmation():
  raw_off = display_state(light=4, quality=0, phase=TrafficControlPhase.off)
  confirmed = display_state(light=4, quality=0, phase=TrafficControlPhase.flashingGreenStop, applied=True)
  assert not raw_off.has_signal
  assert not raw_off.flashing
  assert confirmed.light_state == 4
  assert confirmed.has_signal and confirmed.flashing
  assert traffic_control_highlighted(confirmed)


def test_view_model_does_not_animate_historical_unconfirmed_flash_candidate():
  state = TrafficSignalDisplayState.from_plan(target(
    light=2, phase=TrafficControlPhase.greenFlashCandidate,
  ))
  assert state.visible
  assert not state.flashing


def test_view_model_stays_visible_without_a_current_signal():
  assert not TrafficSignalDisplayState.from_plan(target(), valid=False).visible
  unavailable = TrafficSignalDisplayState.from_plan(target(quality=0, raw_distance=255))
  assert unavailable.visible
  assert not unavailable.has_signal
  assert not unavailable.flashing

  passed = TrafficSignalDisplayState.from_plan(target(phase=TrafficControlPhase.passed, raw_distance=254))
  assert passed.visible
  assert not passed.has_signal

  assert not TrafficSignalDisplayState.from_plan(target(mode=1)).visible


@pytest.mark.parametrize(("raw_distance", "has_signal"), [
  (-1, False), (0, True), (153, True), (200, True), (200.1, False), (254, False),
])
def test_view_model_uses_raw_can_distance_for_signal_range(raw_distance, has_signal):
  state = TrafficSignalDisplayState.from_plan(target(raw_distance=raw_distance, remaining=0, reference=5))
  assert state.has_signal is has_signal


def test_view_model_exposes_driver_override():
  override = TrafficSignalDisplayState.from_plan(target(driver_override=True))
  assert override.driver_override_active


def test_card_is_raised_above_legacy_position_without_overlapping_egpu_panel():
  card = traffic_card_rect(rl.Rectangle(0, 0, 2160, 1080))
  assert card.x == 46
  assert card.y == 347
  assert card.width == 64
  assert card.height == 128
  assert card.y == 427 - 80
  bottom_status_top = 1080 - 61
  assert card.y + card.height <= bottom_status_top - 40


def test_traffic_signal_is_a_small_icon():
  assert TRAFFIC_LIGHT_HOUSING_WIDTH < TRAFFIC_CARD_WIDTH <= 64
  assert TRAFFIC_LIGHT_HOUSING_HEIGHT < TRAFFIC_CARD_HEIGHT <= 128
  assert TRAFFIC_LIGHT_RADIUS == 10


@pytest.mark.parametrize(("values", "blue"), [
  ({"applied": True, "action": 1}, True),
  ({"applied": True, "light": 2, "phase": TrafficControlPhase.release, "action": 3}, True),
  ({"applied": False, "action": 1}, False),
  ({"applied": True, "driver_override": True}, False),
  ({"applied": True, "phase": TrafficControlPhase.off}, False),
  ({"applied": True, "phase": TrafficControlPhase.passed}, False),
  ({"applied": True, "mode": 1}, False),
  ({"applied": True, "raw_fresh": False, "action": 2}, True),
])
def test_blue_outline_means_current_applied_control(values, blue):
  assert traffic_control_highlighted(display_state(**values)) is blue


def test_renderer_draws_only_lamps_and_no_text_or_large_card(monkeypatch):
  renderer, fake_sm = make_renderer(monkeypatch, target(applied=True, action=1))
  renderer.update()
  boxes, outlines, lamps = [], [], []
  monkeypatch.setattr(rl, "draw_rectangle_rounded", lambda rect, *args: boxes.append(rect))
  monkeypatch.setattr(rl, "draw_rectangle_rounded_lines_ex", lambda *args: outlines.append(args))
  monkeypatch.setattr(rl, "draw_circle_v", lambda *args: lamps.append(args))
  monkeypatch.setattr(rl, "draw_text_ex", lambda *args: pytest.fail("icon must not draw text"))
  rect = rl.Rectangle(0, 0, 2160, 1080)
  renderer._render(rect)
  assert all(box.width <= 64 and box.height <= 128 for box in boxes)
  assert len([lamp for lamp in lamps if lamp[1] == TRAFFIC_LIGHT_RADIUS]) == 3
  assert outlines[-1][-1].b == traffic_control_module.CONTROL_OUTLINE.b

  fake_sm["longitudinalPlanSP"] = SimpleNamespace(teslaTrafficControl=target(applied=False, action=1))
  renderer.update()
  renderer._render(rect)
  assert outlines[-1][-1].a == traffic_control_module.BORDER.a
  fake_sm.valid["longitudinalPlanSP"] = False
  renderer.update()
  previous_count = len(outlines)
  renderer._render(rect)
  assert len(outlines) == previous_count
