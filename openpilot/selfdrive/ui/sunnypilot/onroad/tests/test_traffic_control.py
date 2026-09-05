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
  traffic_action_text,
  traffic_card_rect,
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
  ({"light": 2, "phase": TrafficControlPhase.release,
    "applied": True, "action": 4}, "Green · releasing brakes"),
  ({"light": 2, "applied": True, "action": 5}, "Green · continuing"),
  ({"light": 2, "phase": TrafficControlPhase.release, "should_stop": True}, ""),
  ({"light": 2, "phase": TrafficControlPhase.hold, "applied": True,
    "action": 2, "should_stop": True}, "Green · confirming release"),
  ({"light": 2, "phase": TrafficControlPhase.flashingGreenStop,
    "applied": True, "action": 1}, "Signal changing · stopping"),
  ({"light": 2}, ""),
  ({"light": 0, "should_stop": True}, ""),
  ({"light": 3, "should_stop": True}, ""),
  ({"light": 1, "raw_fresh": False}, ""),
  ({"light": 1, "raw_fresh": False, "applied": False, "action": 2}, ""),
  ({"light": 1, "raw_fresh": False, "applied": True, "action": 1}, "Signal lost · slowing continues"),
  ({"light": 1, "raw_fresh": False, "applied": True, "action": 2, "should_stop": True}, "Signal lost · holding stop"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 3}, "Green · auto start"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 4}, "Signal lost · smooth transition"),
  ({"light": 2, "raw_fresh": False, "applied": True, "action": 5}, "Green · continuing"),
]


@pytest.fixture(autouse=True)
def identity_translation(monkeypatch):
  monkeypatch.setattr(traffic_control_module, "tr", lambda text: text)


def target(*, light=1, raw_distance=80.0, remaining=35.0, reference=5.0,
           phase=TrafficControlPhase.braking, quality=2, mode=4, applied=False,
           direction_unknown=False, driver_override=False, action=0, should_stop=False,
           raw_fresh=True, stop_allowed=None, stop_direction_unknown=None,
           start_block_reason=0, stop_session_id=1):
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
    stopSessionId=stop_session_id,
  )


def display_state(**kwargs):
  return TrafficSignalDisplayState.from_plan(target(**kwargs))


def action_states(*, action=1, light=1, phase=TrafficControlPhase.braking, stop_session_id=7):
  values = {"action": action, "light": light, "phase": phase, "stop_session_id": stop_session_id}
  return display_state(applied=True, **values), display_state(applied=False, **values)


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


def test_view_model_never_replaces_far_raw_can_distance_with_five_meter_reference():
  state = TrafficSignalDisplayState.from_plan(target(raw_distance=153, remaining=0, reference=5))
  assert state.distance_m == 153


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
  assert renderer.notice is not None  # old notice latch must not keep the outline blue
  renderer._render(rect)
  assert outlines[-1][-1].a == traffic_control_module.BORDER.a
  fake_sm.valid["longitudinalPlanSP"] = False
  renderer.update()
  previous_count = len(outlines)
  renderer._render(rect)
  assert len(outlines) == previous_count


@pytest.mark.parametrize(("kwargs", "expected"), ACTION_CASES)
def test_action_text_matches_final_plan_action(kwargs, expected):
  state = TrafficSignalDisplayState.from_plan(target(**kwargs))
  assert traffic_action_text(state)[0] == expected


def test_applied_go_action_is_reported_directly():
  state = TrafficSignalDisplayState.from_plan(target(
    light=2, applied=True, action=3,
  ))
  assert traffic_action_text(state)[0] == "Green · auto start"


def test_red_release_describes_stop_transition_instead_of_green_signal():
  state = display_state(
    light=1, phase=TrafficControlPhase.braking, applied=True, action=4,
  )

  assert traffic_action_text(state)[0] == "Stop control · smooth transition"


def test_stale_release_does_not_claim_a_current_green_signal():
  state = display_state(
    light=2, phase=TrafficControlPhase.release, raw_fresh=False,
    applied=True, action=4,
  )

  assert traffic_action_text(state)[0] == "Signal lost · smooth transition"


def test_recent_applied_notice_remains_visible_for_two_seconds_in_past_tense():
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied, no_longer_applied = action_states()

  live_notice = latch.update(applied, now_s=10.0)
  assert live_notice is not None
  assert not live_notice.recent

  recent_notice = latch.update(no_longer_applied, now_s=11.99)
  assert recent_notice is not None
  assert recent_notice.recent
  assert traffic_control_module.traffic_notice_text(recent_notice)[0] == "Red · stop control applied"

  assert latch.update(no_longer_applied, now_s=12.01) is None


@pytest.mark.parametrize(("applied_kwargs", "current_kwargs"), [
  (
    {"action": 1, "light": 1, "phase": TrafficControlPhase.braking},
    {"action": 0, "light": 2, "phase": TrafficControlPhase.release},
  ),
  (
    {"action": 3, "light": 2, "phase": TrafficControlPhase.release},
    {"action": 1, "light": 1, "phase": TrafficControlPhase.braking, "should_stop": True},
  ),
])
def test_recent_notice_clears_immediately_on_opposite_signal(applied_kwargs, current_kwargs):
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied = display_state(applied=True, stop_session_id=7, **applied_kwargs)
  current = display_state(applied=False, stop_session_id=7, **current_kwargs)

  assert latch.update(applied, now_s=10.0) is not None
  assert latch.update(current, now_s=10.1) is None


def test_recent_stop_notice_uses_the_current_compatible_signal_color():
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied = display_state(
    light=1, phase=TrafficControlPhase.braking, applied=True, action=1,
    stop_session_id=7,
  )
  current = display_state(
    light=3, phase=TrafficControlPhase.yellowStop, applied=False, action=1,
    stop_session_id=7,
  )

  assert latch.update(applied, now_s=10.0) is not None
  recent = latch.update(current, now_s=10.1)
  assert recent is not None
  assert recent.state.light_state == 3
  assert traffic_control_module.traffic_notice_text(recent)[0] == "Signal changing · stop applied"


@pytest.mark.parametrize("current_kwargs", [
  {"stop_session_id": 8},
  {"driver_override": True},
  {"mode": 1},
  {"phase": TrafficControlPhase.off},
  {"phase": TrafficControlPhase.passed},
  {"raw_fresh": False},
  {"quality": 0, "raw_distance": 255.0},
])
def test_recent_notice_clears_when_control_context_ends(current_kwargs):
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied_stop, _ = action_states()
  current = display_state(**({
    "light": 1,
    "phase": TrafficControlPhase.braking,
    "applied": False,
    "action": 1,
    "stop_session_id": 7,
  } | current_kwargs))

  assert latch.update(applied_stop, now_s=10.0) is not None
  assert latch.update(current, now_s=10.1) is None


def test_action_that_never_applied_never_creates_a_notice():
  latch = traffic_control_module.TrafficActionNoticeLatch()
  planned_only = display_state(
    light=1, phase=TrafficControlPhase.braking, applied=False, action=1,
    stop_session_id=7,
  )

  assert latch.update(planned_only, now_s=10.0) is None


@pytest.mark.parametrize("invalid_context", [
  {"mode": 1},
  {"driver_override": True},
  {"phase": TrafficControlPhase.off},
  {"phase": TrafficControlPhase.passed},
])
def test_inconsistent_applied_frame_cannot_create_notice_outside_control_context(invalid_context):
  latch = traffic_control_module.TrafficActionNoticeLatch()
  values = {
    "light": 1,
    "phase": TrafficControlPhase.braking,
    "applied": True,
    "action": 1,
    "stop_session_id": 7,
    **invalid_context,
  }

  assert latch.update(display_state(**values), now_s=10.0) is None


def test_each_applied_cycle_refreshes_the_notice_expiry():
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied, no_longer_applied = action_states()

  assert latch.update(applied, now_s=10.0) is not None
  assert latch.update(applied, now_s=11.5) is not None
  assert latch.update(no_longer_applied, now_s=13.49) is not None
  assert latch.update(no_longer_applied, now_s=13.51) is None


@pytest.mark.parametrize(("action", "light", "phase", "expected"), [
  (3, 2, TrafficControlPhase.release, "Green · auto start applied"),
  (4, 2, TrafficControlPhase.release, "Green · stop control released"),
  (4, 1, TrafficControlPhase.braking, "Stop control · transition applied"),
  (5, 2, TrafficControlPhase.release, "Green · continuing applied"),
])
def test_recent_notice_uses_past_tense_for_each_applied_action(action, light, phase, expected):
  latch = traffic_control_module.TrafficActionNoticeLatch()
  applied, no_longer_applied = action_states(action=action, light=light, phase=phase)

  assert latch.update(applied, now_s=10.0) is not None
  recent = latch.update(no_longer_applied, now_s=10.1)
  assert recent is not None
  assert traffic_control_module.traffic_notice_text(recent)[0] == expected


def test_renderer_keeps_live_signal_geometry_while_retaining_recent_action(monkeypatch):
  renderer, fake_sm = make_renderer(monkeypatch, target(
    light=1, raw_distance=80.0, applied=True, action=1, stop_session_id=7,
  ))
  renderer.update()
  assert renderer.state.distance_m == 80.0

  fake_sm["longitudinalPlanSP"] = SimpleNamespace(teslaTrafficControl=target(
    light=1, raw_distance=40.0, applied=False, action=1, stop_session_id=7,
  ))
  renderer.update()

  assert renderer.state.distance_m == 40.0
  assert renderer.notice is not None
  assert renderer.notice.recent
  assert renderer.notice.state.distance_m == 40.0
  assert renderer.notice.action == 1


def test_renderer_does_not_refresh_notice_without_a_new_plan_sample(monkeypatch):
  renderer, fake_sm = make_renderer(monkeypatch, target(
    light=1, applied=True, action=1, stop_session_id=7,
  ))
  times = iter((10.0, 12.01))
  monkeypatch.setattr(traffic_control_module.time, "monotonic", lambda: next(times))
  renderer.update()
  assert renderer.notice is not None

  fake_sm.updated["longitudinalPlanSP"] = False
  renderer.update()
  assert renderer.notice is None


def test_renderer_clears_notice_when_plan_service_becomes_invalid(monkeypatch):
  renderer, fake_sm = make_renderer(monkeypatch, target(
    light=1, applied=True, action=1, stop_session_id=7,
  ))
  renderer.update()
  assert renderer.notice is not None

  fake_sm.valid["longitudinalPlanSP"] = False
  renderer.update()
  assert renderer.notice is None

  fake_sm.valid["longitudinalPlanSP"] = True
  fake_sm["longitudinalPlanSP"] = SimpleNamespace(teslaTrafficControl=target(
    light=1, applied=False, action=1, stop_session_id=7,
  ))
  renderer.update()
  assert renderer.notice is None
