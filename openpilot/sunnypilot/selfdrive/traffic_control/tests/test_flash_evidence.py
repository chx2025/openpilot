"""Adversarial GREEN/OFF evidence through the production traffic controller."""

from dataclasses import replace

from opendbc.can import CANPacker
import pytest

from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlPhase
from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TeslaTrafficControlObserver
from openpilot.sunnypilot.selfdrive.traffic_control.tests.test_controller import controller, observation, update


def feed(c, frames, *, delays=None):
  decisions = []
  for index, (t, color, distance) in enumerate(frames):
    delay = 0.0 if delays is None else delays[index]
    decisions.append(update(c, t + delay, observation(distance, color, t), v_ego=0.0))
  return decisions


FLASH = [(1.0, 2, 80), (1.2, 4, 80), (1.7, 2, 80),
         (2.2, 4, 80), (2.7, 2, 80), (3.2, 4, 80), (3.4, 4, 80)]


def test_third_off_must_persist_in_real_frames_before_flash_stop():
  c = controller()
  decisions = feed(c, FLASH)
  assert all(not decision.apply_constraint for decision in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.flashingGreenStop


def test_two_hz_flash_confirms_on_third_off_completion_without_yellow():
  c = controller()
  frames = [(1.0 + i * 0.5, 2 if i % 2 == 0 else 4, 80) for i in range(7)]
  decisions = feed(c, frames)
  assert all(not decision.apply_constraint for decision in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.flashingGreenStop


def test_explicit_dbc_off_code_can_confirm_flash_without_becoming_a_red_command():
  c = controller()
  for t, color, distance in FLASH:
    raw = observation(distance, color, t)
    if color == 4:
      # Observer preserves DBC OFF but does not label it RED/GREEN eligible.
      raw = replace(raw, valid_for_control=False, quality=0)
    decision = update(c, t, raw, v_ego=0.0)
    if t < 3.4:
      assert not decision.apply_constraint
  assert decision.phase == TrafficControlPhase.flashingGreenStop
  assert decision.light_state == 4


@pytest.mark.parametrize("dlc", [6, 8])
@pytest.mark.parametrize("dark_color", [0, 4])
def test_raw_can_decode_distinguishes_none_from_off_before_flash_control(dlc, dark_color):
  c = controller()
  observer = TeslaTrafficControlObserver()
  packer = CANPacker("tesla_modely_hw4_perception")
  for t, color, distance in FLASH:
    raw_color = dark_color if color == 4 else color
    address, data, bus = packer.make_can_msg("APP_trafficControl", 2, {
      "APP_tcControlType": 3,
      "APP_tcControlSource": 3,
      "APP_tcControlDistance": distance,
      "APP_tcControlLightState": raw_color,
    })
    now_ns = round(t * 1e9)
    observer.update([(now_ns, [(address, data[:dlc], bus)])], now_ns)
    raw = observer.snapshot(now_ns)
    assert raw.available
    assert raw.light_state == raw_color
    if raw_color == 4:
      assert raw.quality == 0
      assert not raw.valid_for_control
    decision = update(c, t, raw, v_ego=0.0)
    if dark_color == 0 or t < 3.4:
      assert not decision.apply_constraint
  assert c.flash_latched == (dark_color == 4)


def test_none_interrupts_pending_off_flash_evidence():
  c = controller()
  frames = FLASH[:4] + [(2.4, 0, 80)] + FLASH[4:]
  assert all(not d.apply_constraint for d in feed(c, frames))


def test_periodic_green_none_never_counts_as_flash():
  c = controller()
  frames = [(1.0 + i * 0.5, 2 if i % 2 == 0 else 0, 80) for i in range(100)]
  for decision in feed(c, frames):
    assert not decision.apply_constraint
    assert not decision.should_stop
    assert decision.stop_session_id == 0
  assert not c.flash_latched


def test_period_restart_cannot_borrow_a_none_pulse():
  c = controller()
  frames = [(1.0, 2, 80), (1.2, 0, 80), (1.4, 2, 80), (1.6, 0, 80),
            (1.8, 2, 80), (2.2, 4, 80), (2.4, 2, 80), (2.8, 4, 80), (3.0, 4, 80)]
  assert all(not d.apply_constraint for d in feed(c, frames))


def test_isolated_explicit_off_never_starts_or_releases_a_stop():
  c = controller()
  frames = [(1.0 + i * 0.5, 4, 80) for i in range(8)]
  assert all(not d.apply_constraint for d in feed(c, frames))


def test_long_explicit_off_after_confirmed_flash_never_releases_hold():
  c = controller()
  feed(c, [(t, color, 5) for t, color, _ in FLASH])
  for i in range(10):
    t = 3.7 + 0.5 * i
    decision = update(c, t, observation(5, 4, t), v_ego=0.0)
    assert decision.phase == TrafficControlPhase.hold
    assert decision.should_stop


def test_periodic_fifty_ms_off_glitches_never_create_a_stop():
  c = controller()
  frames = [(1.0, 2, 80)]
  for i in range(6):
    frames.extend(((1.2 + i, 4, 80), (1.25 + i, 2, 80)))
  assert all(not d.apply_constraint for d in feed(c, frames))


def test_periodic_fifty_ms_green_glitches_never_create_a_stop():
  c = controller()
  frames = [(1.0, 4, 80)]
  for i in range(6):
    frames.extend(((1.2 + i, 2, 80), (1.25 + i, 4, 80)))
  assert all(not d.apply_constraint for d in feed(c, frames))


@pytest.mark.parametrize("short_color", [4, 2])
def test_short_glitches_with_continuous_high_rate_can_never_count(short_color):
  c = controller()
  for tick in range(101):
    color = short_color if tick % 20 == 4 else (2 if short_color == 4 else 4)
    t = 1.0 + tick * 0.05
    decision = update(c, t, observation(80, color, t), v_ego=0.0)
    assert not decision.apply_constraint


def test_green_target_jump_cannot_borrow_old_flash_pulses():
  c = controller()
  frames = FLASH[:4] + [(2.7, 2, 150), (3.2, 4, 150), (3.4, 4, 150)]
  assert all(not d.apply_constraint for d in feed(c, frames))


@pytest.mark.parametrize("invalid", [
  observation(80, 0, 2.4),
  observation(80, 5, 2.4),
  observation(80, 4, 2.4, control_type=0),
  observation(254, 4, 2.4),
])
def test_invalid_tuple_clears_pending_flash_evidence(invalid):
  c = controller()
  feed(c, FLASH[:4])
  update(c, 2.4, invalid, v_ego=0.0)
  assert all(not d.apply_constraint for d in feed(c, FLASH[4:]))


def test_repeated_third_off_snapshot_does_not_count_as_pulse_duration():
  c = controller()
  feed(c, FLASH[:-1])
  repeated = update(c, 3.4, observation(80, 4, 3.2), v_ego=0.0)
  assert not repeated.apply_constraint
  actual = update(c, 3.45, observation(80, 4, 3.45), v_ego=0.0)
  assert actual.phase == TrafficControlPhase.flashingGreenStop


def test_sampling_gap_under_two_seconds_still_breaks_flash_evidence():
  c = controller()
  feed(c, FLASH[:4])
  frames = [(3.1, 2, 80), (3.6, 4, 80), (3.8, 4, 80)]
  assert all(not d.apply_constraint for d in feed(c, frames))


def test_brief_empty_snapshot_does_not_break_fresh_two_hz_flash_evidence():
  c = controller()
  for t, color, distance in FLASH:
    decision = update(c, t, observation(distance, color, t), v_ego=0.0)
    # Source may briefly see no usable snapshot while its model timestamp
    # catches up with a newer car-state packet. No real CAN cadence gap exists.
    update(c, t + 0.01, observation(available=False), v_ego=0.0)
  assert decision.phase == TrafficControlPhase.flashingGreenStop


def test_flash_confirmation_uses_can_time_independent_of_dispatch_delay():
  immediate, delayed = controller(), controller()
  expected = feed(immediate, FLASH)
  actual = feed(delayed, FLASH, delays=[0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.6])
  assert [d.phase for d in actual] == [d.phase for d in expected]
  assert actual[-1].phase == TrafficControlPhase.flashingGreenStop


def test_long_running_flash_stable_green_exits_from_first_green_not_pattern_expiry():
  c = controller()
  frames = [(1.0 + i * 0.5, 2 if i % 2 == 0 else 4, 80) for i in range(12)]
  feed(c, frames)
  assert c.flash_latched
  session_id = c.stop_session_id
  decisions = feed(c, [(7.0, 2, 80), (7.5, 2, 80), (8.0, 2, 80), (8.5, 2, 80)])
  assert all(d.phase == TrafficControlPhase.flashingGreenStop for d in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.release
  assert decisions[-1].stop_session_id == session_id


@pytest.mark.parametrize("invalid_color", [0, 5])
def test_invalid_frame_interrupts_stable_green_without_dropping_flash_hold(invalid_color):
  c = controller()
  feed(c, FLASH)
  feed(c, [(3.7, 2, 80), (4.2, invalid_color, 80)])
  decisions = feed(c, [(4.7, 2, 80), (5.2, 2, 80), (5.7, 2, 80), (6.2, 2, 80)])
  assert all(d.phase == TrafficControlPhase.flashingGreenStop for d in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.release


def test_target_jump_interrupts_stable_green_confirmation():
  c = controller()
  feed(c, FLASH)
  frames = [(3.7, 2, 80), (4.2, 2, 150), (4.7, 2, 80),
            (5.2, 2, 80), (5.7, 2, 80), (6.2, 2, 80)]
  decisions = feed(c, frames)
  assert all(d.phase == TrafficControlPhase.flashingGreenStop for d in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.release


def test_two_hz_confirmation_on_green_starts_stable_exit_timer_immediately():
  c = controller()
  frames = [(1.0 + i * 0.5, 2 if i % 2 == 0 else 4, 80) for i in range(7)]
  feed(c, frames)
  assert c.flash_latched
  decisions = feed(c, [(4.5, 2, 80), (5.0, 2, 80), (5.5, 2, 80)])
  assert all(d.phase == TrafficControlPhase.flashingGreenStop for d in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.release


def test_repeated_off_restarts_stable_green_confirmation():
  c = controller()
  feed(c, FLASH)
  frames = [(3.7, 2, 80), (4.2, 4, 80), (4.7, 2, 80),
            (5.2, 2, 80), (5.7, 2, 80), (6.2, 2, 80)]
  decisions = feed(c, frames)
  assert all(d.phase == TrafficControlPhase.flashingGreenStop for d in decisions[:-1])
  assert decisions[-1].phase == TrafficControlPhase.release


def test_frozen_raw_distance_while_moving_does_not_confirm_flash():
  c = controller()
  frames = [(1.0 + i * 0.5, 2 if i % 2 == 0 else 4, 80) for i in range(10)]
  for t, color, distance in frames:
    decision = update(c, t, observation(distance, color, t), v_ego=8.0)
    assert not decision.apply_constraint


def test_steady_green_approach_can_start_a_later_flash_candidate():
  c = controller()
  for i in range(20):
    t = 1.0 + i * 0.5
    update(c, t, observation(190.0 - 8.0 * (t - 1), 2, t), v_ego=8.0)
  for i in range(7):
    t = 11.0 + i * 0.5
    decision = update(c, t, observation(190.0 - 8.0 * (t - 1), 2 if i % 2 == 0 else 4, t), v_ego=8.0)
  assert decision.phase == TrafficControlPhase.flashingGreenStop
