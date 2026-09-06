from openpilot.selfdrive.controls.lib.longitudinal_planner import taper_toward_less_conservative_output


class TestOutputTaper:
  """Covers the fix for: switching out of Experimental Mode (or the traffic-stop detector
  taking over) while e2e was the winning candidate could produce a visible accelerate-then-brake
  blip, because e2e was quietly covering for a detection lag in the other candidates."""

  def test_limits_the_increase_direction(self):
    result = taper_toward_less_conservative_output(candidate_min=0.05, prev_output=-0.8, j_taper=0.4, dt=0.05)
    assert result == -0.8 + 0.4 * 0.05
    assert result < 0.05

  def test_never_delays_harder_braking(self):
    result = taper_toward_less_conservative_output(candidate_min=-1.5, prev_output=-0.8, j_taper=0.4, dt=0.05)
    assert result == -1.5

  def test_converges_within_the_jerk_budget_over_several_frames(self):
    prev_output = -0.8
    candidate_min = 0.05
    j_taper = 0.4
    dt = 0.05
    for _ in range(50):
      prev_output = taper_toward_less_conservative_output(candidate_min, prev_output, j_taper, dt)
    assert prev_output == candidate_min

  def test_is_a_no_op_when_candidate_is_already_more_conservative_than_prev(self):
    result = taper_toward_less_conservative_output(candidate_min=-1.0, prev_output=-0.5, j_taper=0.4, dt=0.05)
    assert result == -1.0
