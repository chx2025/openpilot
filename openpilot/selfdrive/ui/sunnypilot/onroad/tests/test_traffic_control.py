from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control import TrafficSignalDisplayState
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase


def target(*, light=1, raw_distance=80.0, remaining=35.0, reference=5.0,
           phase=TrafficControlPhase.braking, quality=2, mode=4, applied=False,
           direction_unknown=False, driver_override=False):
  return SimpleNamespace(
    lightState=light,
    mode=mode,
    rawDistance=raw_distance,
    applied=applied,
    directionUnknown=direction_unknown,
    driverOverrideActive=driver_override,
    remainingDistance=remaining,
    stopReference=reference,
    phase=int(phase),
    quality=quality,
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
