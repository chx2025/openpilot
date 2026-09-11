"""Tests for TurnDecelController: blinker-triggered slow-down for tight turns.

Spec (from user requirement):
  1. After blinker on for 2s, apply 0.5 m/s² decel toward 15 km/h target.
  2. Steering > 9° pauses the decel but still blocks any accel.
  3. Steering > 60° AND v_ego > 15 km/h blocks accel **independently of
     blinker** (no active decel, just a passive guard rail).
  4. Blinker off -> instant reset of the decel state machine; HOWEVER, if
     rule 3 (steering > 60° + v > 15 km/h) still holds, keep block_accel.
  5. At-target (v_ego <= 15 km/h) -> stop decel, keep block_accel.

The controller is intentionally placed in sunnypilot/ as a stand-alone module
that longitudinal_planner post-applies on top of the candidates-min pool. The
tests here cover the controller in isolation; longitudinal_planner integration
is verified by AST / smoke tests in the repo's own test target.
"""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib import turn_decel
from openpilot.sunnypilot.selfdrive.controls.lib.turn_decel import (
  BLINKER_DEBOUNCE_S, DECEL_M_S2, DECEL_RAMP_S, STEERING_BLOCK_ACCEL_DEG,
  STEERING_PAUSE_DEG, TARGET_V_KPH, TARGET_V_MS, TurnDecelController,
)


DT = 0.05  # DT_MDL


# ===== helper: run N frames at given state, return last result =====
def run(controller: TurnDecelController, n: int, *,
        blinker_on: bool, v_ego: float, steering_angle_deg: float) -> turn_decel.TurnDecelResult:
  for _ in range(n):
    res = controller.update(blinker_on=blinker_on, v_ego=v_ego,
                            steering_angle_deg=steering_angle_deg, dt=DT)
  return res


# ===== 1. idle / blinker off =====
def test_blinker_off_returns_idle_no_override():
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=5.0, dt=DT)
  assert res.a_target_override is None
  assert res.block_accel is False
  assert res.active is False
  assert res.phase == "idle"


def test_blinker_off_resets_decel_state():
  """If we were deceling and blinker goes off, next frame must be idle and
  internal ramp must be zeroed so re-trigger doesn't carry over old ramp."""
  c = TurnDecelController()
  run(c, 100, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)  # ramp to full
  assert c._decel_a > 0.0
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
  assert res.phase == "idle"
  assert c._decel_a == 0.0
  assert c._blinker_on_time == 0.0


# ===== 2. debounce / waiting phase =====
def test_debounce_waits_full_2s_before_decel():
  c = TurnDecelController()
  # Just under 2s: still waiting
  res = run(c, 39, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)  # 39*0.05=1.95s
  assert res.phase == "waiting"
  assert res.a_target_override is None
  assert res.active is False
  # 40th frame crosses the 2s mark
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
  assert res.phase == "deceling"
  assert res.a_target_override is not None
  assert res.active is True
  assert res.block_accel is True


def test_debounce_with_big_angle_during_wait_still_blocks_accel():
  """Even during the 2s wait, a > 60° turn at > 15 km/h must block accel."""
  c = TurnDecelController()
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=STEERING_BLOCK_ACCEL_DEG + 5, dt=DT)
  assert res.phase == "waiting"
  assert res.block_accel is True


# ===== 3. decel phase =====
def test_decel_outputs_negative_target_a():
  c = TurnDecelController()
  res = run(c, 60, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)  # past 2s + ramp
  assert res.phase == "deceling"
  assert res.a_target_override is not None
  # Should be the full decel (ramp finishes in 0.5s, we ran 60 frames = 3s)
  assert abs(res.a_target_override - (-DECEL_M_S2)) < 1e-6


def test_decel_ramp_does_not_jump_to_full():
  """The 0.5s ramp prevents a step from 0 to -0.5; the first frame after
  decel starts should be much less than -0.5."""
  c = TurnDecelController()
  # Cross the 2s mark exactly
  run(c, 40, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)
  # First decel frame: ramp = (1 frame / DECEL_RAMP_S) * DECEL_M_S2 = 0.05/0.5 * 0.5 = 0.05
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
  assert res.phase == "deceling"
  assert -res.a_target_override < DECEL_M_S2, "ramp clipped to full too early"
  assert -res.a_target_override > 0.0


def test_decel_respects_target_v_ego_floor():
  """Once v_ego <= 15 km/h, decel stops (phase becomes at_target)."""
  c = TurnDecelController()
  # At target
  res = run(c, 50, blinker_on=True, v_ego=TARGET_V_MS - 0.1, steering_angle_deg=2.0)
  assert res.phase == "at_target"
  assert res.a_target_override is None
  assert res.block_accel is True  # still no accel at target


# ===== 4. pause on steering > 9° =====
def test_steering_above_pause_threshold_pauses_decel():
  c = TurnDecelController()
  # Get into decel phase
  run(c, 60, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)
  assert c._decel_a > 0.0
  # Steer hard: pause
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=STEERING_PAUSE_DEG + 1, dt=DT)
  assert res.phase == "paused"
  assert res.a_target_override is None
  assert res.block_accel is True  # 保证不加速
  assert c._decel_a == 0.0, "ramp must reset so resume is smooth"


def test_steering_release_resumes_decel_smoothly():
  """After pause, decel should resume from 0 (not jump to full) so the
  driver doesn't feel a sudden re-brake."""
  c = TurnDecelController()
  # Get into full decel
  run(c, 80, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)
  full_decel = c._decel_a
  assert abs(full_decel - DECEL_M_S2) < 1e-6
  # Pause
  run(c, 20, blinker_on=True, v_ego=20.0, steering_angle_deg=STEERING_PAUSE_DEG + 1)
  assert c._decel_a == 0.0
  # Resume: first frame should ramp from 0
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
  assert res.phase == "deceling"
  assert -res.a_target_override < DECEL_M_S2  # not full yet


# ===== 5. big-angle block (independent of blinker) =====
def test_big_angle_block_at_speed_even_without_decel_active():
  """Steering > 60° AND v_ego > 15 km/h blocks accel even when we're in
  the at_target / paused / waiting phase (no active decel)."""
  for steering in (STEERING_BLOCK_ACCEL_DEG + 0.1, 90.0, 180.0):
    for v_ego in (TARGET_V_MS + 0.1, 25.0, 50.0):
      c = TurnDecelController()
      # Sit in at_target by being at low v_ego + decel ramped
      run(c, 50, blinker_on=True, v_ego=TARGET_V_MS, steering_angle_deg=2.0)
      # Now big angle
      res = c.update(blinker_on=True, v_ego=v_ego, steering_angle_deg=steering, dt=DT)
      assert res.block_accel is True, f"v_ego={v_ego} steer={steering} phase={res.phase}"


def test_big_angle_below_target_does_not_block():
  """Steering > 60° but already at/below 15 km/h: no need to block accel
  (driver may want to creep)."""
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=TARGET_V_MS - 0.5, steering_angle_deg=80.0, dt=DT)
  assert res.block_accel is False


def test_no_big_angle_no_block_at_high_speed():
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=5.0, dt=DT)
  assert res.block_accel is False
  assert res.phase == "idle"


# ===== 5b. big-angle block **independent of blinker** (the user's correction) =====
def test_big_angle_block_with_blinker_off():
  """User requirement: regardless of blinker state, steering > 60° AND
  v_ego > 15 km/h must block accel. Covers U-turn, parking, spiral ramps
  where the driver doesn't bother with the blinker."""
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=80.0, dt=DT)
  assert res.block_accel is True
  assert res.a_target_override is None  # no active decel
  assert res.active is False
  assert res.phase == "big_angle_guard"


def test_big_angle_block_with_blinker_off_persists_across_frames():
  """As long as steering > 60° AND v > 15 km/h, keep blocking accel even
  though no blinker is on."""
  c = TurnDecelController()
  for _ in range(50):
    res = c.update(blinker_on=False, v_ego=25.0, steering_angle_deg=90.0, dt=DT)
  assert res.phase == "big_angle_guard"
  assert res.block_accel is True


def test_big_angle_guard_releases_when_steering_straightened():
  """When steering drops back below 60° (while blinker still off), the
  big_angle_guard phase ends."""
  c = TurnDecelController()
  c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=80.0, dt=DT)
  assert c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=80.0, dt=DT).phase == "big_angle_guard"
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=10.0, dt=DT)
  assert res.phase == "idle"
  assert res.block_accel is False


def test_big_angle_guard_releases_at_target_speed():
  """If v drops to <= 15 km/h while blinker off and steering > 60°, release
  the block (driver may want to creep)."""
  c = TurnDecelController()
  c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=80.0, dt=DT)
  res = c.update(blinker_on=False, v_ego=TARGET_V_MS - 0.5, steering_angle_deg=80.0, dt=DT)
  assert res.phase == "idle"
  assert res.block_accel is False


def test_big_angle_guard_does_not_start_blinker_decel_when_blinker_comes_back():
  """If we're in big_angle_guard (blinker off) and the driver then turns
  on the blinker, the debounce timer starts from 0 (because we reset on
  the falling edge of the prior blinker-on period)."""
  c = TurnDecelController()
  # Enter big_angle_guard
  c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=80.0, dt=DT)
  # Now blinker comes on (big angle still active)
  res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=80.0, dt=DT)
  # Should be 'waiting' (just turned on, debounce timer fresh)
  assert res.phase == "waiting"
  assert res.block_accel is True  # still big_angle_block


def test_big_angle_guard_boundary_at_exactly_60deg():
  """Steering == 60° exactly does NOT trigger the block (strict >)."""
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=STEERING_BLOCK_ACCEL_DEG, dt=DT)
  assert res.block_accel is False
  assert res.phase == "idle"


def test_big_angle_guard_boundary_at_exactly_target_speed():
  """v == 15 km/h exactly does NOT trigger the block (strict >)."""
  c = TurnDecelController()
  res = c.update(blinker_on=False, v_ego=TARGET_V_MS, steering_angle_deg=80.0, dt=DT)
  assert res.block_accel is False
  assert res.phase == "idle"


# ===== 6. sign / magnitude sanity =====
def test_a_target_is_always_negative_when_active():
  c = TurnDecelController()
  for _ in range(120):  # 6s of decel
    res = c.update(blinker_on=True, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
    if res.a_target_override is not None:
      assert res.a_target_override < 0.0


def test_a_target_magnitude_capped_at_decel_m_s2():
  """Even after a long ramp, a_target_override must never exceed DECEL_M_S2."""
  c = TurnDecelController()
  res = run(c, 500, blinker_on=True, v_ego=30.0, steering_angle_deg=2.0)
  assert -res.a_target_override <= DECEL_M_S2 + 1e-6


# ===== 7. reset semantics =====
def test_external_reset_clears_all_state():
  c = TurnDecelController()
  run(c, 100, blinker_on=True, v_ego=20.0, steering_angle_deg=2.0)
  c.reset()
  assert c._blinker_on_time == 0.0
  assert c._decel_a == 0.0
  # Next frame with blinker off must be idle
  res = c.update(blinker_on=False, v_ego=20.0, steering_angle_deg=2.0, dt=DT)
  assert res.phase == "idle"


# ===== 8. config constants match spec =====
def test_constants_match_user_spec():
  assert BLINKER_DEBOUNCE_S == 2.0
  assert DECEL_M_S2 == 0.5
  assert TARGET_V_KPH == 15.0
  assert abs(TARGET_V_MS - 15.0 / 3.6) < 1e-9
  assert STEERING_PAUSE_DEG == 9.0
  assert STEERING_BLOCK_ACCEL_DEG == 60.0
