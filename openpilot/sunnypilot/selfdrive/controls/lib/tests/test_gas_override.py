"""Tests for GasOverrideController: driver gas-pedal longitudinal override.

Spec (user requirement, 2026-09-25):
  1. While the gas pedal is pressed, all deceleration commands coming from the
     model or the system must be paused or reduced so the car gently closes on
     the lead -- UNLESS
       a) lead distance < 4 m AND ego is faster than the lead by > 10 km/h, or
       b) lead distance < 2 m.
  2. After releasing the gas, if vEgo > vCruise -> coast down to the set speed,
     then the original logic takes over.
  3. After releasing the gas, if vEgo < vCruise -> the original system resumes
     normally after 0.5 s, with no dead gap.
  3b. (2026-09-27) The hand-over at the end of requirement 2/3 must not be a step:
     the floor is ramped back down over GAS_RELEASE_RAMP_S (section 5b).

The controller is a stand-alone sunnypilot module that longitudinal_planner
post-applies (`a_target = max(a_target, floor)`). These tests cover the
controller in isolation; the longitudinal_planner wiring is covered by the
repo-level smoke check.
"""
from openpilot.sunnypilot.selfdrive.controls.lib import gas_override
from openpilot.sunnypilot.selfdrive.controls.lib.gas_override import (
  GAS_OVERRIDE_ENABLE, GAS_OVERRIDE_PARAM_KEY, GAS_PRESSED_A_FLOOR,
  GAS_RELEASE_COAST_A_CEIL, GAS_RELEASE_COAST_EXIT_MARGIN_MS,
  GAS_RELEASE_COAST_MAX_DECEL, GAS_RELEASE_COAST_MAX_S,
  GAS_RELEASE_RAMP_ENABLE, GAS_RELEASE_RAMP_S,
  GAS_RELEASE_RESUME_DELAY_S, GAS_SAFE_A_TARGET_HARD,
  GAS_SAFE_A_TARGET_RELEASE, GAS_SAFE_CLOSING_KPH, GAS_SAFE_D_REL_CLOSE_M,
  GAS_SAFE_D_REL_MIN_M, GAS_SAFE_TTC_ENTER_S, GAS_SAFE_TTC_RELEASE_S,
  GasOverrideController,
)


DT = 0.05       # DT_MDL, plannerd runs at 20 Hz
KPH = 1.0 / 3.6  # km/h -> m/s

# "the system is asking for a mild brake" -- deliberately ABOVE the safety
# exception e entry threshold (-2.0), so the distance / coast / toggle cases
# below test their own thing instead of being pre-empted by e.
A_MILD_BRAKE = -1.5


def step(c: GasOverrideController, **kw) -> gas_override.GasOverrideResult:
  """One frame with sensible defaults (v_ego 20 m/s < v_cruise 25 m/s)."""
  args = dict(gas_pressed=False, v_ego=20.0, v_cruise=25.0, accel_coast=-0.3,
              a_target_in=-1.0, dt=DT)
  args.update(kw)
  return c.update(**args)


def run(c: GasOverrideController, n: int, **kw) -> gas_override.GasOverrideResult:
  for _ in range(n):
    res = step(c, **kw)
  return res


# ===== 0. constants match the user spec =====
def test_constants_match_user_spec():
  assert GAS_PRESSED_A_FLOOR == -0.3
  assert GAS_SAFE_D_REL_CLOSE_M == 4.0
  assert GAS_SAFE_CLOSING_KPH == 10.0
  assert GAS_SAFE_D_REL_MIN_M == 2.0
  assert GAS_RELEASE_RESUME_DELAY_S == 0.5
  assert GAS_RELEASE_COAST_A_CEIL == 0.0
  assert GAS_OVERRIDE_ENABLE is True
  # release ramp added 2026-09-27 (see section 5b)
  assert GAS_RELEASE_RAMP_ENABLE is True
  assert GAS_RELEASE_RAMP_S == 0.4
  # safety exceptions e/f added after the 2026-09-25 real-world event
  assert GAS_SAFE_A_TARGET_HARD == -2.0
  assert GAS_SAFE_TTC_ENTER_S == 4.0
  # hysteresis: the release thresholds must be strictly above the entry ones
  assert GAS_SAFE_A_TARGET_RELEASE > GAS_SAFE_A_TARGET_HARD
  assert GAS_SAFE_TTC_RELEASE_S > GAS_SAFE_TTC_ENTER_S


# ===== 1. baseline: nothing pressed -> transparent =====
def test_inactive_when_gas_not_pressed():
  c = GasOverrideController()
  res = step(c, gas_pressed=False, a_target_in=A_MILD_BRAKE)
  assert res.phase == "inactive"
  assert res.active is False
  assert res.a_floor is None
  assert res.a_target_out == A_MILD_BRAKE   # byte-identical passthrough
  assert res.reason == ""
  assert res.suppress_should_stop is False


# ===== 2. requirement 1: gas pressed raises the deceleration floor =====
def test_gas_pressed_reduces_braking():
  """The core behaviour: a mild model brake (-1.5) becomes -0.3.

  Note the mild value: since 2026-09-25 a *hard* request (<= -2.0) is no longer
  overridden -- see the safety exception e tests below.
  """
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE)
  assert res.phase == "pressed"
  assert res.active is True
  assert res.a_floor == GAS_PRESSED_A_FLOOR
  assert res.a_target_out == GAS_PRESSED_A_FLOOR


def test_gas_pressed_never_cuts_acceleration():
  """The floor must not touch the acceleration side at all."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=1.75)
  assert res.a_target_out == 1.75


def test_gas_pressed_leaves_milder_braking_untouched():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=-0.1)
  assert res.a_target_out == -0.1


def test_gas_pressed_suppresses_stop_intent():
  """Requirement 3 'no dead gap': LongControl must not latch into stopping."""
  on = GasOverrideController()
  assert step(on, gas_pressed=True, v_ego=0.2, a_target_in=-0.3).suppress_should_stop is True
  # a fresh controller that never saw the gas pedal must stay transparent
  off = GasOverrideController()
  assert step(off, gas_pressed=False, v_ego=0.2, a_target_in=-0.3).suppress_should_stop is False


# ===== 3. requirement 1 exceptions =====
def test_no_lead_means_no_exception():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=False)
  assert res.active is True


def test_lead_closer_than_2m_blocks_override():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=-2.0, lead_present=True, d_rel=1.9, v_lead=20.0)
  assert res.phase == "inactive"
  assert res.active is False
  assert res.a_target_out == -2.0          # original hard braking preserved
  assert "lead<" in res.reason


def test_lead_exactly_at_2m_still_overrides():
  """Spec says 'less than' 2 m, so exactly 2.0 m must still override."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True, d_rel=2.0, v_lead=20.0)
  assert res.active is True


def test_lead_close_and_fast_blocks_override():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=-2.0, lead_present=True,
             d_rel=3.5, v_lead=20.0 - 12.0 * KPH)
  assert res.active is False
  assert res.a_target_out == -2.0
  assert "fast" in res.reason


def test_lead_close_but_not_fast_keeps_override():
  """Close, but barely closing: neither distance exception applies, and the
  closing rate (1 km/h) keeps the TTC far away, so f stays quiet too."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=3.5, v_lead=20.0 - 1.0 * KPH)
  assert res.active is True


def test_lead_far_and_fast_keeps_override():
  """Far away and closing at 15 km/h -> TTC 7.2 s, nobody has to intervene."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=30.0, v_lead=20.0 - 15.0 * KPH)
  assert res.active is True


def test_closing_below_10kph_keeps_override(monkeypatch):
  """Spec says 'faster by more than 10 km/h', so 9.95 km/h must not trip it.

  f (TTC) is switched off on purpose: 3.5 m at a 9.95 km/h closing rate is a
  1.3 s TTC, which *should* stop the yield -- that case is covered by the ttc
  tests below. Here we only pin down the 10 km/h distance-exception boundary.
  """
  monkeypatch.setattr(gas_override, "GAS_SAFE_TTC_ENTER_S", None)
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=3.5, v_lead=20.0 - 9.95 * KPH)
  assert res.active is True


def test_closing_above_10kph_blocks_override(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_SAFE_TTC_ENTER_S", None)
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=3.5, v_lead=20.0 - 10.05 * KPH)
  assert res.active is False


def test_lead_distance_boundary_at_4m_keeps_override(monkeypatch):
  """Spec says 'less than' 4 m, so exactly 4.0 m must still override.

  f is switched off here for the same reason as above: 4 m at a 36 km/h closing
  rate is a 0.4 s TTC -- f *should* stop the yield, and test_ttc_* proves it.
  """
  monkeypatch.setattr(gas_override, "GAS_SAFE_TTC_ENTER_S", None)
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=GAS_SAFE_D_REL_CLOSE_M, v_lead=10.0)
  assert res.active is True


def test_fcw_guard_blocks_override():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=-3.0, fcw=True)
  assert res.active is False
  assert res.a_target_out == -3.0
  assert res.reason == "fcw"


def test_force_decel_guard_blocks_override():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, a_target_in=-3.0, force_decel=True)
  assert res.active is False
  assert res.reason == "forceDecel"


def test_fcw_guard_can_be_disabled(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_OVERRIDE_FCW_GUARD", False)
  c = GasOverrideController()
  assert step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE, fcw=True).active is True


def test_low_or_unset_v_cruise_does_not_override():
  c = GasOverrideController()
  for vc in (0.0, 0.5, 1.0):
    res = step(c, gas_pressed=True, v_cruise=vc, a_target_in=A_MILD_BRAKE)
    assert res.active is False
    assert res.a_target_out == A_MILD_BRAKE
    assert res.reason == "no_v_cruise"


# ===== 3b. safety exceptions e/f (added 2026-09-25, after a real event) =====
# Real event (device comma-e80e0b52, 09-25 09:39 local, evidence in gas_patch/evidence/):
#   lead 46.2 -> 22.3 km/h in 2 s, dRel 34.7 -> 28.7 m, a_target -2.75 m/s^2.
#   The distance-only exceptions (dRel < 4 m / < 2 m) stayed silent, the yield held
#   the floor at -0.3, ego *accelerated* 43.9 -> 49.0, and the car ended up parked
#   4.2 m behind the lead. e re-uses the system's own request (most sensitive);
#   f is the TTC backstop for the same situation expressed geometrically.
def test_hard_brake_request_blocks_override():
  """e: replay of the real event -- -2.75 must cancel the yield."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, v_ego=49.0 * KPH, v_cruise=58.0 * KPH,
             a_target_in=-2.75, lead_present=True, d_rel=28.7, v_lead=22.3 * KPH)
  assert res.phase == "inactive"
  assert res.active is False
  assert res.a_target_out == -2.75         # the braking request survives untouched
  assert res.reason.startswith("aTgt")


def test_hard_brake_threshold_is_inclusive():
  c = GasOverrideController()
  assert step(c, gas_pressed=True, a_target_in=GAS_SAFE_A_TARGET_HARD).active is False


def test_mild_brake_just_above_threshold_still_overrides():
  c = GasOverrideController()
  assert step(c, gas_pressed=True,
              a_target_in=GAS_SAFE_A_TARGET_HARD + 0.01).active is True


def test_hard_brake_latch_holds_inside_the_band():
  """Hysteresis: coming back up into the band must NOT resume yielding yet."""
  c = GasOverrideController()
  assert step(c, gas_pressed=True, a_target_in=-2.5).active is False
  assert step(c, gas_pressed=True, a_target_in=-1.5).active is False           # band
  assert step(c, gas_pressed=True, a_target_in=GAS_SAFE_A_TARGET_RELEASE).active is True


def test_output_does_not_flap_around_the_threshold():
  """Without hysteresis the output would be [-2.1, -0.3, -2.1, -0.3, -2.1]."""
  c = GasOverrideController()
  ins = [-2.1, -1.9, -2.1, -1.9, -2.1]
  outs = [step(c, gas_pressed=True, a_target_in=a).a_target_out for a in ins]
  assert outs == ins, f"output flapped: {outs}"


def test_hard_brake_exception_also_blocks_the_hold_window():
  """Requirement 3's 0.5 s window must not keep yielding while the system brakes hard.

  This is frame 2 of the real event: gas released, a_target -3.26, lead at 20.7 km/h.
  """
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=48.8 * KPH, v_cruise=58.0 * KPH)
  res = step(c, gas_pressed=False, v_ego=48.8 * KPH, v_cruise=58.0 * KPH,
             a_target_in=-3.26, lead_present=True, d_rel=27.1, v_lead=20.7 * KPH)
  assert res.phase == "inactive"
  assert res.a_target_out == -3.26
  assert c._hold_left == 0.0


def test_hard_brake_exception_can_be_disabled(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_SAFE_A_TARGET_HARD", None)
  c = GasOverrideController()
  assert step(c, gas_pressed=True, a_target_in=-3.0).active is True


def test_ttc_blocks_override_when_closing_fast():
  """f: 28.7 m sounds far, but at 7.4 m/s closing that is only 3.9 s."""
  c = GasOverrideController()
  res = step(c, gas_pressed=True, v_ego=49.0 * KPH, v_cruise=58.0 * KPH,
             a_target_in=A_MILD_BRAKE, lead_present=True,
             d_rel=28.7, v_lead=(49.0 - 26.7) * KPH)
  assert res.active is False
  assert res.reason.startswith("ttc")


def test_ttc_ignores_a_lead_that_is_pulling_away():
  c = GasOverrideController()
  res = step(c, gas_pressed=True, v_ego=10.0, v_cruise=16.0,
             a_target_in=A_MILD_BRAKE, lead_present=True, d_rel=5.0, v_lead=20.0)
  assert res.active is True


def test_ttc_latch_holds_inside_the_band():
  c = GasOverrideController()
  # TTC 2.5 s -> latched
  assert step(c, gas_pressed=True, v_ego=10.0, v_cruise=16.0, a_target_in=A_MILD_BRAKE,
              lead_present=True, d_rel=20.0, v_lead=10.0 - 8.0).active is False
  # TTC 5.0 s -> inside the (4, 6) band, stays latched
  assert step(c, gas_pressed=True, v_ego=10.0, v_cruise=16.0, a_target_in=A_MILD_BRAKE,
              lead_present=True, d_rel=25.0, v_lead=10.0 - 5.0).active is False
  # TTC 14 s -> released, yielding resumes
  assert step(c, gas_pressed=True, v_ego=10.0, v_cruise=16.0, a_target_in=A_MILD_BRAKE,
              lead_present=True, d_rel=70.0, v_lead=10.0 - 5.0).active is True


def test_safety_latches_are_cleared_by_reset():
  c = GasOverrideController()
  step(c, gas_pressed=True, a_target_in=-2.5)
  assert c._hard_decel_latch is True
  c.reset()
  assert c._hard_decel_latch is False
  assert c._ttc_latch is False


def test_hard_brake_exception_does_not_spam_the_log():
  """Steady-state hard braking (normal cruise following) must not log at 1 Hz."""
  c = GasOverrideController()
  assert step(c, gas_pressed=False, a_target_in=-2.5).log is not None    # transition
  for _ in range(60):
    assert step(c, gas_pressed=False, a_target_in=-2.5).log is None, "aTgt is spamming"


def test_master_switch_off_is_transparent(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_OVERRIDE_ENABLE", False)
  c = GasOverrideController()
  assert step(c, gas_pressed=True, a_target_in=-2.5).a_target_out == -2.5


# ===== 4. requirement 2: release while overspeed -> coast down =====
def test_release_overspeed_enters_coast():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0, a_target_in=-1.5)
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0, a_target_in=-1.5)
  assert res.phase == "coast"
  assert res.active is True
  assert res.a_floor == GAS_RELEASE_COAST_MAX_DECEL     # flat road: -0.3
  assert res.a_target_out == -0.3


def test_coast_ceiling_blocks_acceleration():
  """Coasting means no throttle, so a positive original target is capped."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0, a_target_in=0.8)
  assert res.phase == "coast"
  assert res.a_target_out == 0.0


def test_coast_uses_real_coast_decel_on_uphill():
  """accel_coast is more negative uphill -> allow the honest coast value."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0, accel_coast=-0.9)
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0, accel_coast=-0.9)
  assert res.a_floor == -0.9


def test_coast_still_decelerates_on_downhill():
  """accel_coast positive downhill -> clamp to -0.3 so it still converges."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0, accel_coast=0.2)
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0, accel_coast=0.2)
  assert res.a_floor == GAS_RELEASE_COAST_MAX_DECEL


def test_coast_exits_into_release_then_inactive():
  """[2026-09-27] coast 到期不再一帧交还：先进 release（缓释），走完才 inactive。"""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  assert step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0).phase == "coast"
  assert step(c, gas_pressed=False, v_ego=26.0, v_cruise=25.0).phase == "coast"
  res = step(c, gas_pressed=False, v_ego=25.0 + 0.1, v_cruise=25.0)
  assert res.phase == "release"
  assert res.active is True
  # 缓释走完后彻底交还：地板撤掉，输出与输入逐位相同
  n = int(GAS_RELEASE_RAMP_S / DT) + 4
  res = run(c, n, gas_pressed=False, v_ego=25.0, v_cruise=25.0)
  assert res.phase == "inactive"
  assert res.a_floor is None
  assert res.a_target_out == -1.0


def test_coast_holds_just_above_exit_margin():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=25.0 + GAS_RELEASE_COAST_EXIT_MARGIN_MS + 0.2, v_cruise=25.0)
  assert res.phase == "coast"


def test_coast_has_max_duration_fallback():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  n = int(GAS_RELEASE_COAST_MAX_S / DT) + int(GAS_RELEASE_RAMP_S / DT) + 5
  res = run(c, n, gas_pressed=False, v_ego=30.0, v_cruise=25.0)
  assert res.phase == "inactive"


def test_coast_can_be_disabled(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_RELEASE_COAST_ENABLE", False)
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0)
  assert res.phase == "hold"     # falls back to the short handover window


# ===== 5. requirement 3: release while underspeed -> 0.5 s then hand over =====
def test_release_underspeed_enters_hold():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0)
  assert res.phase == "hold"
  assert res.active is True
  assert res.a_floor == GAS_PRESSED_A_FLOOR


def test_hold_lasts_the_configured_delay():
  """The hand-over must happen after ~GAS_RELEASE_RESUME_DELAY_S, not before,
  and definitely not never. (Frame count is float-accumulated, so allow +-1.)"""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0)
  nominal = int(round(GAS_RELEASE_RESUME_DELAY_S / DT))
  n_hold = 0
  for _ in range(nominal + 5):
    if step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0).phase == "hold":
      n_hold += 1
    else:
      break
  assert abs(n_hold - nominal) <= 1, f"hold lasted {n_hold} frames, expected ~{nominal}"
  assert abs(n_hold * DT - GAS_RELEASE_RESUME_DELAY_S) <= 2 * DT


def test_hold_keeps_acceleration_open_no_dead_gap():
  """Requirement 3 'no dead gap': the original logic may accelerate freely."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=1.2)
  assert res.phase == "hold"
  assert res.a_target_out == 1.2
  assert res.suppress_should_stop is True


def test_hold_delay_zero_still_ramps_then_hands_over(monkeypatch):
  """The 0.5 s window can be zeroed, but the release ramp still applies afterwards."""
  monkeypatch.setattr(gas_override, "GAS_RELEASE_RESUME_DELAY_S", 0.0)
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0)
  assert res.phase == "release"
  n = int(GAS_RELEASE_RAMP_S / DT) + 4
  res = run(c, n, gas_pressed=False, v_ego=20.0, v_cruise=25.0)
  assert res.phase == "inactive"
  assert res.a_target_out == -1.0


def test_hold_delay_zero_and_ramp_off_hands_over_immediately(monkeypatch):
  """One-line revert of BOTH knobs => the pre-2026-09-27 behaviour."""
  monkeypatch.setattr(gas_override, "GAS_RELEASE_RESUME_DELAY_S", 0.0)
  monkeypatch.setattr(gas_override, "GAS_RELEASE_RAMP_ENABLE", False)
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0)
  assert res.phase == "inactive"
  assert res.a_target_out == -1.0


# ===== 5b. release ramp: the hand-over is spread over GAS_RELEASE_RAMP_S =====
# Added 2026-09-27, after a real drive. User report: turn the wheel with a bit of
# gas, straighten out, and the car brakes "all of a sudden" (both left and right
# turns). Log from that drive:
#   pressed  vEgo=15.3 vCruise=55.0 aIn=-1.66 floor=-0.30 aOut=-0.30   <- hidden
#   inactive vEgo=19.1 vCruise=22.1 aIn=-1.01 floor=None  aOut=-1.01   <- one frame
# The yield itself is by design (requirement 1: hide the upstream decel while the
# driver is on the gas). What was *not* by design is that the hand-over was a step:
# `max(a_target, -0.3)` -> `a_target` in a single 20 Hz frame, and LongControl is a
# plain PID+feedforward with no jerk limit, so it reaches the actuator 1:1.
# The ramp spreads that step over GAS_RELEASE_RAMP_S while keeping the invariant
# "only ever soften" (the ramp may never produce a harsher request than upstream).


def _drain(c, *, a_target_in: float, v_ego: float = 20.0, v_cruise: float = 25.0,
           frames: int = 60):
  """Press once, release, then collect frames until the module stops intervening."""
  step(c, gas_pressed=True, v_ego=v_ego, v_cruise=v_cruise, a_target_in=a_target_in)
  out = []
  for _ in range(frames):
    res = step(c, gas_pressed=False, v_ego=v_ego, v_cruise=v_cruise, a_target_in=a_target_in)
    out.append(res)
    if res.phase == "inactive":
      break
  return out


def test_release_ramp_spreads_the_handover_over_several_frames():
  """The whole point: -0.30 -> -1.01 must NOT happen inside one frame."""
  c = GasOverrideController()
  seq = _drain(c, a_target_in=-1.01)
  phases = [r.phase for r in seq]
  assert "release" in phases, phases
  n_rel = sum(1 for p in phases if p == "release")
  assert n_rel >= 3, f"ramp too short ({n_rel} frames): {phases}"

  outs = [r.a_target_out for r in seq]
  deltas = [abs(b - a) for a, b in zip(outs, outs[1:])]
  step_size = abs(-1.01 - GAS_PRESSED_A_FLOOR) / 3.0
  assert max(deltas) <= step_size, f"still a step: max delta {max(deltas):.3f} in {deltas}"
  # 地板只可能越来越允许刹车（单调），不会来回抖
  assert all(b <= a + 1e-9 for a, b in zip(outs, outs[1:])), outs


def test_release_ramp_ends_bit_exact():
  """The frame before the hand-over must be within one ramp step of a_target.

  That is what makes the hand-over itself a non-event: at alpha == 1 the floor
  equals a_target, so the last intervening frame and the first fully transparent
  frame produce the same number.
  """
  c = GasOverrideController()
  a_target = -1.01
  seq = _drain(c, a_target_in=a_target)
  last = seq[-1]
  assert last.phase == "inactive"
  assert last.a_floor is None
  assert last.a_target_out == a_target                 # byte-identical passthrough
  prev = seq[-2]
  assert prev.phase == "release"
  tol = abs(a_target - GAS_PRESSED_A_FLOOR) * DT / GAS_RELEASE_RAMP_S * 1.05 + 1e-9
  assert abs(prev.a_target_out - a_target) <= tol, (prev.a_target_out, a_target, tol)


def test_release_ramp_duration_is_bounded():
  """hold + ramp together last ~GAS_RELEASE_RESUME_DELAY_S + GAS_RELEASE_RAMP_S."""
  c = GasOverrideController()
  n_hold = n_rel = 0
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.9)
  for _ in range(60):
    res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.9)
    if res.phase == "hold":
      n_hold += 1
    elif res.phase == "release":
      n_rel += 1
    else:
      break
  assert abs(n_hold * DT - GAS_RELEASE_RESUME_DELAY_S) <= 2 * DT, n_hold
  assert abs(n_rel * DT - GAS_RELEASE_RAMP_S) <= 2 * DT, n_rel


def test_release_ramp_only_ever_softens():
  """Invariant: a_target_out >= a_target_in for every frame of the ramp."""
  c = GasOverrideController()
  a_target = -1.4
  for r in _drain(c, a_target_in=a_target):
    assert r.a_target_out >= a_target - 1e-12, (a_target, r.a_target_out)


def test_release_ramp_is_transparent_when_accelerating():
  """Upstream wants to accelerate => the ramp must leave it bit-identical."""
  c = GasOverrideController()
  for r in _drain(c, a_target_in=1.2):
    assert r.a_target_out == 1.2, r


def test_release_ramp_never_exceeds_the_original_request():
  """Fuzz the ramp over a grid of requests; it must never add acceleration nor hurt."""
  for a_target in (-1.99, -1.7, -1.0, -0.5, -0.2, -0.05, 0.0, 0.5, 2.0):
    c = GasOverrideController()
    for r in _drain(c, a_target_in=a_target):
      assert r.a_target_out >= a_target - 1e-12, (a_target, r.a_target_out)
      assert r.a_target_out <= max(a_target, 0.0) + 1e-12, (a_target, r.a_target_out)


def test_release_ramp_safety_exception_cancels_it_at_once():
  """Safety first: a hard request mid-ramp must hand over in ONE frame, not ramp."""
  c = GasOverrideController()
  seq = None
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
  for _ in range(40):
    res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
    if res.phase == "release":
      seq = res
      break
  assert seq is not None, "never entered the release phase"
  hard = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-2.5)
  assert hard.phase == "inactive"
  assert hard.a_target_out == -2.5            # 全额交还，没有缓释
  assert hard.reason.startswith("aTgt")
  assert c._release_from is None              # 缓释计时器已清


def test_release_ramp_repress_returns_to_pressed():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
  for _ in range(40):
    res = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
    if res.phase == "release":
      break
  else:
    raise AssertionError("never entered the release phase")
  res = step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
  assert res.phase == "pressed"
  assert res.a_floor == GAS_PRESSED_A_FLOOR
  assert c._release_from is None


def test_release_ramp_can_be_disabled(monkeypatch):
  """One-line revert (GAS_RELEASE_RAMP_ENABLE=False) => the old single-frame gap."""
  monkeypatch.setattr(gas_override, "GAS_RELEASE_RAMP_ENABLE", False)
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.01)
  seq = [step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.01)
         for _ in range(20)]
  phases = [r.phase for r in seq]
  assert "release" not in phases, phases
  assert phases[-1] == "inactive"
  assert seq[-1].a_target_out == -1.01


def test_release_ramp_zero_seconds_hands_over_at_once(monkeypatch):
  """GAS_RELEASE_RAMP_S = 0 must not add any extra phase.

  Note the 0.5 s hold window still comes first (requirement 3), so we drive the
  whole release and assert the ramp itself contributes nothing.
  """
  monkeypatch.setattr(gas_override, "GAS_RELEASE_RAMP_S", 0.0)
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.01)
  seq = [step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.01)
         for _ in range(20)]
  phases = [r.phase for r in seq]
  assert "release" not in phases, phases
  assert phases[-1] == "inactive"
  assert seq[-1].a_target_out == -1.01


def test_reset_clears_the_release_timer():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0)
  for _ in range(40):
    if step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0, a_target_in=-1.0).phase == "release":
      break
  else:
    raise AssertionError("never entered the release phase")
  c.reset()
  assert c._release_from is None
  assert c._release_elapsed == 0.0
  assert c._phase == "inactive"


# ===== 6. state handling =====
def test_reset_clears_everything():
  c = GasOverrideController()
  run(c, 3, gas_pressed=True, a_target_in=-2.0)
  c.reset()
  assert c._phase == "inactive"
  assert c._gas_last is False
  assert c._hold_left == 0.0
  assert c._coast_elapsed == 0.0
  assert c._release_from is None
  assert c._release_elapsed == 0.0
  assert step(c, gas_pressed=False, a_target_in=-2.0).phase == "inactive"


def test_repress_during_coast_returns_to_pressed():
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  assert step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0).phase == "coast"
  res = step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0, a_target_in=A_MILD_BRAKE)
  assert res.phase == "pressed"
  assert res.a_floor == GAS_PRESSED_A_FLOOR


def test_coast_does_not_stick_after_handover():
  """Once the coast -> release ramp is drained, staying inactive must be transparent."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0)
  assert step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0).phase == "coast"
  assert step(c, gas_pressed=False, v_ego=25.0, v_cruise=25.0).phase == "release"   # exits coast
  run(c, int(GAS_RELEASE_RAMP_S / DT) + 4, gas_pressed=False, v_ego=25.0, v_cruise=25.0)
  res = step(c, gas_pressed=False, v_ego=24.0, v_cruise=25.0, a_target_in=-1.7)
  assert res.phase == "inactive"
  assert res.a_target_out == -1.7


def test_safety_exception_on_press_survives_release():
  """If the lead is still dangerously close at release, do NOT start coasting."""
  c = GasOverrideController()
  assert step(c, gas_pressed=True, v_ego=30.0, v_cruise=25.0, lead_present=True,
              d_rel=1.5, v_lead=29.0).active is False
  res = step(c, gas_pressed=False, v_ego=30.0, v_cruise=25.0, lead_present=True,
             d_rel=1.5, v_lead=29.0, a_target_in=-2.5)
  assert res.phase == "inactive"
  assert res.a_target_out == -2.5


def test_safety_exception_clears_hold_window():
  """A gas press under the exception must not arm the 0.5 s window."""
  c = GasOverrideController()
  step(c, gas_pressed=True, v_ego=20.0, v_cruise=25.0, fcw=True)
  assert c._hold_left == 0.0


# ===== 7. runtime on/off switch (UI toggle, param GAS_OVERRIDE_PARAM_KEY) =====
class FakeParams:
  """Minimal stand-in for openpilot Params: only `get(key, return_default=True)`."""

  def __init__(self, value: bool | None = True):
    self.value = value
    self.fail = False
    self.calls = 0

  def get(self, key, return_default=False, block=False):
    self.calls += 1
    if self.fail:
      raise RuntimeError("param backend exploded")
    return self.value


def test_param_off_makes_the_module_fully_transparent():
  """UI toggle off -> byte-identical passthrough, regardless of the gas pedal."""
  fp = FakeParams(False)
  c = GasOverrideController(fp)
  res = step(c, gas_pressed=True, a_target_in=-2.6)
  assert res.enabled is False
  assert res.active is False
  assert res.a_floor is None
  assert res.a_target_out == -2.6
  assert res.reason == "disabled"
  assert res.suppress_should_stop is False


def test_param_on_keeps_normal_behaviour():
  c = GasOverrideController(FakeParams(True))
  res = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE)
  assert res.enabled is True
  assert res.a_target_out == GAS_PRESSED_A_FLOOR


def test_param_default_on_uses_declared_default():
  """A declared-default read is what makes the feature ON by default."""
  c = GasOverrideController(FakeParams(True))
  assert step(c, gas_pressed=False).enabled is True


def test_param_is_polled_at_about_1hz():
  """Flipping the toggle must take effect within ~1 s, not instantly and not never."""
  fp = FakeParams(True)
  c = GasOverrideController(fp)
  assert step(c, gas_pressed=True).enabled is True       # first frame syncs

  fp.value = False
  for i in range(15):                                    # 0.75 s: still on
    assert step(c, gas_pressed=True).enabled is True, f"took effect too early (frame {i})"

  for _ in range(10):                                    # must land by ~1.25 s
    if step(c, gas_pressed=True).enabled is False:
      break
  else:
    raise AssertionError("param change never took effect")


def test_param_read_failure_keeps_previous_state():
  """A broken param backend must never silently disable a safety-relevant guard."""
  fp = FakeParams(True)
  c = GasOverrideController(fp)
  assert step(c, gas_pressed=True).enabled is True
  fp.fail = True
  for _ in range(40):
    assert step(c, gas_pressed=True).enabled is True


def test_param_none_uses_code_default():
  c = GasOverrideController()                            # offline / test mode
  assert step(c, gas_pressed=True).enabled is True


def test_code_switch_and_param_are_anded(monkeypatch):
  monkeypatch.setattr(gas_override, "GAS_OVERRIDE_ENABLE", False)
  c = GasOverrideController(FakeParams(True))
  assert step(c, gas_pressed=True).enabled is False
  assert step(c, gas_pressed=True, a_target_in=-2.6).a_target_out == -2.6


def test_param_flip_is_reported_in_log():
  fp = FakeParams(True)
  c = GasOverrideController(fp)
  step(c, gas_pressed=False)
  fp.value = False
  note = None
  for _ in range(25):
    res = step(c, gas_pressed=False)
    if res.log and "param" in res.log:
      note = res.log
      break
  assert note is not None, "toggle flip was never logged"
  assert GAS_OVERRIDE_PARAM_KEY in note


def test_disabled_state_does_not_log_forever():
  """Off is the steady state when the user turns it off: log the flip, not 1 Hz."""
  c = GasOverrideController(FakeParams(False))
  assert step(c, gas_pressed=True).log is not None       # transition
  for _ in range(60):
    res = step(c, gas_pressed=True)
    assert res.log is None, "disabled state is spamming the log"


# ===== 8. logging throttle =====
def test_log_on_transition_only_within_throttle():
  c = GasOverrideController()
  first = step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE)
  assert first.log is not None
  assert "[GasOverride]" in first.log
  assert "pressed" in first.log
  # same phase, inside the 1 Hz throttle window -> silent
  assert step(c, gas_pressed=True, a_target_in=A_MILD_BRAKE).log is None
  # leaving the phase is always logged (so the tail of an event is visible)
  out = step(c, gas_pressed=False, v_ego=20.0, v_cruise=25.0)
  assert out.phase == "hold"
  assert out.log is not None


def test_no_log_spam_when_idle():
  c = GasOverrideController()
  for _ in range(50):
    assert step(c, gas_pressed=False, a_target_in=-1.0).log is None
