from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.traffic_control import TrafficSignalDisplayState
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase


def target(*, light=1, remaining=35.0, reference=5.0, phase=TrafficControlPhase.braking, quality=2):
  return SimpleNamespace(
    lightState=light,
    mode=4,
    remainingDistance=remaining,
    stopReference=reference,
    phase=int(phase),
    quality=quality,
  )


def test_view_model_displays_actual_color_and_can_distance():
  state = TrafficSignalDisplayState.from_plan(target())
  assert state.visible
  assert state.light_state == 1
  assert state.distance_m == 40.0
  assert not state.flashing


def test_view_model_marks_flashing_green_stop():
  state = TrafficSignalDisplayState.from_plan(target(
    light=0, phase=TrafficControlPhase.flashingGreenStop,
  ))
  assert state.visible
  assert state.flashing


def test_view_model_hides_invalid_or_out_of_range_context():
  assert not TrafficSignalDisplayState.from_plan(target(), valid=False).visible
  assert not TrafficSignalDisplayState.from_plan(target(remaining=200, reference=5)).visible
  assert not TrafficSignalDisplayState.from_plan(target(quality=0)).visible
  assert not TrafficSignalDisplayState.from_plan(target(phase=TrafficControlPhase.passed)).visible
