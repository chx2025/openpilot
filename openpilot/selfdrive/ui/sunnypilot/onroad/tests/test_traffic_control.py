from types import SimpleNamespace
from pathlib import Path

from fontTools.ttLib import TTFont
import pyray as rl
import pytest

import openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control as traffic_control_module
from openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control import (
  CARD,
  TrafficSignalDisplayState,
  TRAFFIC_DETAIL_FONT_SIZE,
  TRAFFIC_DISTANCE_FONT_SIZE,
  TRAFFIC_SOURCE_FONT_SIZE,
  TRAFFIC_CARD_WIDTH,
  TRAFFIC_CARD_HEIGHT,
  TRAFFIC_LIGHT_HOUSING_HEIGHT,
  TRAFFIC_LIGHT_HOUSING_WIDTH,
  TRAFFIC_LIGHT_RADIUS,
  TRAFFIC_TEXT_X_OFFSET,
  traffic_action_text,
  traffic_card_rect,
  traffic_source_text,
)
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase


ACTION_CASES = [
  ({"quality": 0, "raw_distance": 255}, ""),
  ({"light": 1, "stop_allowed": False}, ""),
  ({"light": 1, "stop_allowed": True}, ""),
  ({"light": 1, "applied": False, "action": 1}, ""),
  ({"light": 2, "applied": False, "action": 3}, ""),
  ({"light": 1, "applied": True, "action": 1}, "Red · slowing to stop"),
  ({"light": 1, "applied": True, "action": 2, "should_stop": True}, "Red · holding"),
  ({"light": 2, "applied": True, "action": 3}, "Green · auto start"),
  ({"light": 2, "applied": True, "action": 4}, "Green · releasing brakes"),
  ({"light": 2, "applied": True, "action": 5}, "Green · continuing"),
  ({"light": 2, "phase": TrafficControlPhase.release, "should_stop": True,
    "start_block_reason": 6}, ""),
  ({"light": 2, "phase": TrafficControlPhase.release, "should_stop": True}, ""),
  ({"light": 2, "phase": TrafficControlPhase.hold, "applied": True,
    "action": 2, "should_stop": True}, "Green · confirming release"),
  ({"light": 2, "phase": TrafficControlPhase.flashingGreenStop,
    "applied": True, "action": 1}, "Signal changing · stopping"),
  ({"light": 2, "direction_unknown": True, "should_stop": True}, ""),
  ({"light": 2}, ""),
  ({"light": 0, "should_stop": True}, ""),
  ({"light": 3, "should_stop": True}, ""),
  ({"light": 1, "raw_fresh": False}, ""),
  ({"light": 1, "raw_fresh": False, "applied": False, "action": 2}, ""),
  ({"light": 1, "raw_fresh": False, "applied": True, "action": 1}, "Signal lost · slowing continues"),
  ({"light": 1, "raw_fresh": False, "applied": True, "action": 2, "should_stop": True}, "Signal lost · holding stop"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 3}, "Green · auto start"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 4}, "Green · releasing brakes"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 5}, "Green · continuing"),
]


@pytest.fixture(autouse=True)
def identity_translation(monkeypatch):
  monkeypatch.setattr(traffic_control_module, "tr", lambda text: text)


def target(*, light=1, raw_distance=80.0, remaining=35.0, reference=5.0,
           phase=TrafficControlPhase.braking, quality=2, mode=4, applied=False,
           direction_unknown=False, driver_override=False, action=0, should_stop=False,
           raw_fresh=True, stop_allowed=None, stop_direction_unknown=None,
           start_block_reason=0):
  return SimpleNamespace(
    lightState=light,
    mode=mode,
    rawDistance=raw_distance,
    applied=applied,
    directionUnknown=direction_unknown,
    driverOverrideActive=driver_override,
    stopControlAllowed=applied if stop_allowed is None else stop_allowed,
    rawObservationFresh=raw_fresh,
    stopDirectionUnknown=(direction_unknown if stop_direction_unknown is None else stop_direction_unknown),
    remainingDistance=remaining,
    stopReference=reference,
    phase=int(phase),
    quality=quality,
    action=action,
    shouldStop=should_stop,
    startBlockReason=start_block_reason,
  )


def test_view_model_displays_actual_color_and_can_distance():
  state = TrafficSignalDisplayState.from_plan(target())
  assert state.visible
  assert state.has_signal
  assert state.light_state == 1
  assert state.distance_m == 80.0
  assert not state.flashing


def test_view_model_marks_flashing_green_stop():
  state = TrafficSignalDisplayState.from_plan(target(
    light=0, phase=TrafficControlPhase.flashingGreenStop,
  ))
  assert state.visible
  assert state.flashing


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


def test_view_model_never_replaces_far_raw_can_distance_with_five_meter_reference():
  state = TrafficSignalDisplayState.from_plan(target(raw_distance=153, remaining=0, reference=5))
  assert state.distance_m == 153


def test_view_model_exposes_direction_shadow_and_driver_override():
  direction = TrafficSignalDisplayState.from_plan(target(direction_unknown=True))
  override = TrafficSignalDisplayState.from_plan(target(driver_override=True))
  assert direction.direction_unknown
  assert override.driver_override_active


def test_card_is_raised_above_legacy_position_without_overlapping_egpu_panel():
  card = traffic_card_rect(rl.Rectangle(0, 0, 2160, 1080))
  assert card.x == 46
  assert card.y == 347
  assert card.width >= 940
  assert card.height >= 240
  assert card.y == 427 - 80
  bottom_status_top = 1080 - 61
  assert card.y + card.height <= bottom_status_top - 40


def test_traffic_text_is_large_enough_for_onroad_readability():
  assert TRAFFIC_DISTANCE_FONT_SIZE >= 82
  assert TRAFFIC_DETAIL_FONT_SIZE >= 52


def test_traffic_light_and_background_match_large_translucent_layout():
  assert TRAFFIC_LIGHT_HOUSING_WIDTH >= 92
  assert TRAFFIC_LIGHT_HOUSING_HEIGHT >= 192
  assert TRAFFIC_LIGHT_RADIUS >= 22
  assert CARD.a <= 160
  assert TRAFFIC_CARD_WIDTH >= 940
  assert TRAFFIC_CARD_HEIGHT >= 240


@pytest.mark.parametrize(("kwargs", "expected"), ACTION_CASES)
def test_action_text_matches_final_plan_action(kwargs, expected):
  state = TrafficSignalDisplayState.from_plan(target(**kwargs))
  assert traffic_action_text(state)[0] == expected


def test_applied_go_action_takes_priority_over_stop_only_direction_shadow():
  state = TrafficSignalDisplayState.from_plan(target(
    light=2, applied=True, action=3, direction_unknown=False, stop_direction_unknown=True,
  ))
  assert traffic_action_text(state)[0] == "Green · auto start"


def test_source_badge_only_identifies_applied_traffic_control():
  inactive = TrafficSignalDisplayState.from_plan(target(
    light=1, applied=False, should_stop=True,
  ))
  active = TrafficSignalDisplayState.from_plan(target(
    light=1, applied=True, action=2, should_stop=True,
  ))

  assert traffic_source_text(inactive) == ""
  assert traffic_action_text(inactive)[0] == ""
  assert traffic_source_text(active) == "Tesla Traffic Control"
  assert traffic_action_text(active)[0] == "Red · holding"


def test_all_english_action_labels_fit_the_card_at_render_size():
  font_path = Path(__file__).resolve().parents[4] / "assets" / "fonts" / "Inter-Regular.ttf"
  with TTFont(font_path) as font:
    cmap = font.getBestCmap()
    metrics = font["hmtx"].metrics
    units_per_em = font["head"].unitsPerEm

  labels = {expected for _, expected in ACTION_CASES if expected} | {
    "Signal changing · stopping", "Green · confirming release",
  }
  available_width = TRAFFIC_CARD_WIDTH - TRAFFIC_TEXT_X_OFFSET - 14.0
  for label in labels:
    width = sum(metrics[cmap.get(ord(char), ".notdef")][0] for char in label) / units_per_em * TRAFFIC_DETAIL_FONT_SIZE
    assert width <= available_width, f"{label!r} is {width:.1f}px wide; available width is {available_width:.1f}px"

  source = "Tesla Traffic Control"
  source_width = sum(metrics[cmap.get(ord(char), ".notdef")][0] for char in source) / units_per_em * TRAFFIC_SOURCE_FONT_SIZE
  assert source_width + 44.0 < TRAFFIC_CARD_WIDTH - TRAFFIC_TEXT_X_OFFSET
