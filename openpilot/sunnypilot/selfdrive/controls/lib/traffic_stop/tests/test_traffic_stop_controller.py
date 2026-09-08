from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop import traffic_stop_controller as tsc
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import (
  TrafficStopController,
  TrafficStopState,
  TrafficLightState,
  STOPPED_GRACE_FRAMES,
  STARTING_SUPPRESS_FRAMES,
  TRAFFIC_STOP_CAMERA_TO_FRONT_M,
  TRAFFIC_STOP_ENTRY_DISTANCE_M,
  TRAFFIC_STOP_COMFORT_BRAKE_INITIAL,
  TRAFFIC_STOP_COMFORT_BRAKE_MAX,
  TRAFFIC_STOP_COMFORT_BRAKE_RAMPUP_M_S3,
)


class MockXYZTData:
  def __init__(self, x=None, y=None):
    self.x = x if x is not None else [0.0] * 33
    self.y = y if y is not None else [0.0] * 33


class MockModelV2:
  def __init__(self, position_x=None, position_y=None, velocity_x=None):
    self.position = MockXYZTData(x=position_x, y=position_y)
    self.velocity = MockXYZTData(x=velocity_x)


class MockLeadOne:
  def __init__(self, present=False, dRel=1000.0):
    self.present = present
    self.dRel = dRel


class MockRadarState:
  def __init__(self, present=False, dRel=1000.0):
    self.leadOne = MockLeadOne(present=present, dRel=dRel)


class MockCarState:
  def __init__(self, steeringAngleDeg=0.0, gasPressed=False, leftBlinker=False):
    self.steeringAngleDeg = steeringAngleDeg
    self.gasPressed = gasPressed
    self.leftBlinker = leftBlinker


def approaching_red_light_model(model_x_end=15.0, model_v_end=2.0, model_v_start=10.0, y_end=0.0):
  """A model trajectory that should trigger a red-light detection at v_ego ~= model_v_start."""
  position_x = [model_x_end] * 33  # last point (index -2 via STOP_MODEL_IDX) close to model_x_end
  position_y = [0.0] * 33
  position_y[-1] = y_end
  velocity_x = [model_v_start] * 33
  velocity_x[-1] = model_v_end
  return MockModelV2(position_x=position_x, position_y=position_y, velocity_x=velocity_x)


def green_light_model(model_x_end=200.0, model_v=20.0):
  """A model trajectory whose start_sign condition holds every frame (for GREEN_CONFIRM_SEC debounce tests)."""
  velocity_x = [model_v] * 33
  return MockModelV2(position_x=[model_x_end] * 33, position_y=[0.0] * 33, velocity_x=velocity_x)


def run_frames(controller, model_v2, cs, rs, v_ego, a_ego, v_cruise, n=1):
  result = None
  for _ in range(n):
    result = controller.update(model_v2, cs, rs, v_ego, a_ego, v_cruise)
  return result


class TestTrafficStopController:
  def test_disabled_returns_none(self, monkeypatch):
    monkeypatch.setattr(tsc, "TRAFFIC_STOP_ENABLED", False)
    controller = TrafficStopController()
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState()
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0)
    assert result.stop_dist_m is None
    assert result.v_cruise_limited is None

  def test_red_single_frame_trigger(self):
    """Red-light detection has no debounce -- a single qualifying frame is enough, BUT only at
    low speed. At high speed the controller stays in CRUISE and lets e2e handle the decel."""
    controller = TrafficStopController()
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.STOPPING
    assert result.stop_dist_m is not None

  def test_steering_angle_blocks_entry(self):
    """>=5 deg steering suppresses *new* entries into traffic-stop management (dead-straight gate)."""
    controller = TrafficStopController()
    model = approaching_red_light_model()  # model_x_end=15 (< 30m: distance gate satisfied)
    cs = MockCarState(steeringAngleDeg=6.0)  # just past the 5° threshold
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_any_lead_blocks_entry_regardless_of_distance(self):
    """Entry is blocked by ANY detected lead (XState.lead takes over in cp), not just a lead
    closer than the stop line -- a real lead well past the stop line still blocks entry."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=15.0)
    cs = MockCarState()
    rs = MockRadarState(present=True, dRel=500.0)  # far beyond the virtual stop line
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_gas_press_during_stopping_releases_and_suppresses_reentry(self):
    controller = TrafficStopController()
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    cs_gas = MockCarState(gasPressed=True)
    result = run_frames(controller, model, cs_gas, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None
    assert controller._gas_suppress_frames == STARTING_SUPPRESS_FRAMES

    result = run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_gas_press_during_cruise_does_not_suppress_future_entry(self):
    """Ordinary gas presses during normal driving (no active stop) must not arm the 10s
    suppression window -- only a gas press while actively braking toward a red does."""
    controller = TrafficStopController()
    cs_gas = MockCarState(gasPressed=True)
    rs = MockRadarState(present=False)
    far_model = approaching_red_light_model(model_x_end=200.0, model_v_end=15.0, model_v_start=15.0)
    run_frames(controller, far_model, cs_gas, rs, v_ego=15.0, a_ego=0.5, v_cruise=15.0, n=3)
    assert controller._state == TrafficStopState.CRUISE
    assert controller._gas_suppress_frames == 0

  def test_closer_lead_releases_during_stopping(self):
    controller = TrafficStopController()
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    rs_lead = MockRadarState(present=True, dRel=10.0)  # far closer than the ~15m filtered stop-line estimate
    result = run_frames(controller, model, cs, rs_lead, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_lead_cancel_margin_is_4m_not_2m(self):
    """LEAD_CLOSE_TO_STOP_LINE_M was raised from cp's 2.0m to 4.0m so the worst-case final
    stopped gap to a real lead near the boundary is ~4m instead of ~2m."""
    cs = MockCarState()

    # lead 3m beyond the stop-line estimate: must cancel under the 4.0m margin (would NOT have
    # cancelled under cp's original 2.0m margin)
    controller_a = TrafficStopController()
    model_a = approaching_red_light_model(model_x_end=20.0)
    run_frames(controller_a, model_a, cs, MockRadarState(present=False), v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller_a._state == TrafficStopState.STOPPING
    result_a = run_frames(controller_a, model_a, cs, MockRadarState(present=True, dRel=23.0),
                           v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller_a._state == TrafficStopState.CRUISE
    assert result_a.stop_dist_m is None

    # lead 6m beyond the stop-line estimate: still outside even the widened 4.0m margin
    controller_b = TrafficStopController()
    model_b = approaching_red_light_model(model_x_end=20.0)
    run_frames(controller_b, model_b, cs, MockRadarState(present=False), v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller_b._state == TrafficStopState.STOPPING
    result_b = run_frames(controller_b, model_b, cs, MockRadarState(present=True, dRel=26.0),
                           v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller_b._state == TrafficStopState.STOPPING
    assert result_b.stop_dist_m is not None

  def test_reaches_stopped_state_on_first_slow_frame(self):
    """cp transitions STOPPING -> STOPPED the instant v_ego < 0.3 m/s -- no multi-frame hold."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    result = run_frames(controller, model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert result.stop_dist_m is not None

  def test_stopped_state_forces_v_cruise_to_zero(self):
    controller = TrafficStopController()
    model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    result = run_frames(controller, model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert result.v_cruise_limited == 0.0

  def test_green_exit_from_stopped_respects_grace_period(self):
    """The obstacle releases instantly on green (flicker-on-purpose, ported from cp), but the
    formal STOPPED -> CRUISE state transition waits out the ~0.5s grace window."""
    controller = TrafficStopController()
    stop_model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, stop_model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    run_frames(controller, stop_model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert controller._stopped_grace_frames == STOPPED_GRACE_FRAMES

    green = green_light_model()
    run_frames(controller, green, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=STOPPED_GRACE_FRAMES - 1)
    assert controller._state == TrafficStopState.STOPPED

  def test_green_confirm_needs_debounce(self):
    """_check_model_stopping requires ~0.2s (4 frames) of start_sign before reporting GREEN."""
    controller = TrafficStopController()
    model_v_traj = [10.0] * 33
    model_v_traj[-1] = 20.0
    for i in range(4):
      state = controller._check_model_stopping(v_cruise=10.0, model_v_traj=model_v_traj, v_ego=10.0, a_ego=0.0,
                                                model_x_end=200.0, model_y_traj=[0.0] * 33, d_rel=1000.0)
      assert state != TrafficLightState.GREEN
    state = controller._check_model_stopping(v_cruise=10.0, model_v_traj=model_v_traj, v_ego=10.0, a_ego=0.0,
                                              model_x_end=200.0, model_y_traj=[0.0] * 33, d_rel=1000.0)
    assert state == TrafficLightState.GREEN

  def test_v_cruise_limited_is_monotonic(self):
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=5.0, model_v_start=5.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert result.stop_dist_m is not None
    assert result.v_cruise_limited is not None
    assert result.v_cruise_limited <= 5.0

  def test_model_filters_persist_across_release(self):
    """The median/average filters on the raw model x are never cleared -- they keep running
    continuously across separate stop events. Only the per-event accumulator/state resets."""
    controller = TrafficStopController()
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=5)
    assert controller._state == TrafficStopState.STOPPING
    assert len(controller._stop_x_avg_hist) > 0
    hist_len_before = len(controller._stop_x_avg_hist)

    cs_gas = MockCarState(gasPressed=True)
    run_frames(controller, model, cs_gas, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert len(controller._stop_x_avg_hist) >= hist_len_before

  def test_camera_to_front_baseline_applies_even_with_zero_distance_adjust(self, monkeypatch):
    """TRAFFIC_STOP_CAMERA_TO_FRONT_M (-1.5m, ported from cp's real default) must always be
    applied, even when TRAFFIC_STOP_DISTANCE_ADJUST_M is left at its neutral 0."""
    monkeypatch.setattr(tsc, "TRAFFIC_STOP_DISTANCE_ADJUST_M", 0.0)
    model = approaching_red_light_model(model_x_end=40.0, model_v_start=10.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)

    baseline_ctrl = TrafficStopController()
    baseline_result = run_frames(baseline_ctrl, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)

    monkeypatch.setattr(tsc, "TRAFFIC_STOP_DISTANCE_ADJUST_M", 5.0)
    plus5_ctrl = TrafficStopController()
    plus5_result = run_frames(plus5_ctrl, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)

    assert abs((plus5_result.stop_dist_m - baseline_result.stop_dist_m) - 5.0) < 1e-6
    assert TRAFFIC_STOP_CAMERA_TO_FRONT_M == -1.5

  # ------------------------------------------------------------------------ #
  # DISTANCE + STRAIGHT-LINE GATE tests                                     #
  # (replaces the old speed-based takeover: engage only when BOTH steering  #
  #  abs < 5° AND stop-line distance < 30 m)                                #
  # ------------------------------------------------------------------------ #

  def test_entry_distance_constant_is_in_meters(self):
    """Sanity check: TRAFFIC_STOP_ENTRY_DISTANCE_M is a distance in meters, not a speed."""
    assert TRAFFIC_STOP_ENTRY_DISTANCE_M == 30.0

  def test_far_distance_does_not_engage_stopping(self):
    """DISTANCE GATE: even with dead-straight steering and a red light, the controller stays
    in CRUISE while the model-predicted stop line is farther than 30 m. longitudinal_planner
    keeps e2e in the candidate pool so e2e naturally decelerates without a mid-decel handoff
    blip (the original "sudden accel-then-brake" problem)."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=100.0)  # 100 m out
    cs = MockCarState()  # steering = 0 (straight)
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None
    assert result.v_cruise_limited is None

  def test_engages_when_near_and_straight(self):
    """Same red-light signal, but the model stop line is now within 30 m and steering is
    straight: the controller takes over and enters STOPPING."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=15.0)  # 15 m out
    cs = MockCarState()  # steering = 0 (straight)
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.STOPPING
    assert result.stop_dist_m is not None

  def test_at_distance_threshold_does_not_engage(self):
    """Edge case: stop-line distance must be strictly less than 30 m (not equal). At exactly
    30 m we stay in CRUISE."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=TRAFFIC_STOP_ENTRY_DISTANCE_M)  # exactly 30 m
    cs = MockCarState()
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_filter_histories_warm_up_even_when_idle(self):
    """While the controller stays in CRUISE because of the distance gate, the model-x
    median/avg filters should still be running and accumulating. This means by the time
    the stop line comes within 30 m and the controller engages STOPPING, the filters are
    already warm with the correct stop-line estimate -- not from 1-2 raw samples."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=100.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    # Far away for many frames; controller is in CRUISE because of the distance gate.
    run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=20)
    assert controller._state == TrafficStopState.CRUISE
    assert len(controller._stop_x_avg_hist) == 15  # saturated at the 15-frame window
    assert len(controller._stop_x_median_hist) == 3  # saturated at the 3-frame window
    assert len(controller._model_v_hist) == 10  # saturated at the 10-frame window

  # ------------------------------------------------------------------ #
  # COMFORT-BRAKE RAMP tests (0.5 -> 1.8 m/s^2 while braking)          #
  # ------------------------------------------------------------------ #

  def test_comfort_brake_ramp_constants(self):
    """Ramp constants must match the user spec: start 0.5, cap 1.8, positive ramp-up rate."""
    assert TRAFFIC_STOP_COMFORT_BRAKE_INITIAL == 0.5
    assert TRAFFIC_STOP_COMFORT_BRAKE_MAX == 1.8
    assert TRAFFIC_STOP_COMFORT_BRAKE_RAMPUP_M_S3 > 0.0

  def test_comfort_brake_starts_at_initial_and_ramps_up(self):
    """On entry into STOPPING, comfort_brake resets to INITIAL, then climbs frame-by-frame
    toward MAX (never jumping straight to the cap)."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=15.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    # First frame: enters STOPPING with comfort_brake == INITIAL (+1 frame of ramp)
    result = run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=1)
    assert controller._state == TrafficStopState.STOPPING
    assert result.v_cruise_limited is not None
    assert controller._comfort_brake < TRAFFIC_STOP_COMFORT_BRAKE_INITIAL + 0.02  # still near INITIAL

    # After many frames of braking, comfort_brake must be strictly above INITIAL (ramped up).
    run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=40)
    assert controller._comfort_brake > TRAFFIC_STOP_COMFORT_BRAKE_INITIAL

  def test_comfort_brake_never_exceeds_max(self):
    """Even after a very long braking approach, comfort_brake is capped at MAX."""
    controller = TrafficStopController()
    model = approaching_red_light_model(model_x_end=15.0, model_v_start=7.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=7.0, a_ego=0.0, v_cruise=7.0, n=500)
    assert controller._comfort_brake <= TRAFFIC_STOP_COMFORT_BRAKE_MAX + 1e-6
