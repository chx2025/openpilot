import pytest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import resolve_pcm_long_required_max


@pytest.mark.parametrize(("limit_kph", "expected_kph"), ((40, 40), (50, 50), (70, 70), (75, 80), (121, 130)))
def test_tesla_pcm_confirmation_follows_the_actual_metric_limit(limit_kph, expected_kph):
  target = resolve_pcm_long_required_max(True, limit_kph, True, brand="tesla")
  assert round(target * CV.MS_TO_KPH) == expected_kph


def test_tesla_pcm_confirmation_without_a_limit_falls_back_to_the_maximum():
  target = resolve_pcm_long_required_max(True, 0, False, brand="tesla")
  assert round(target * CV.MS_TO_KPH) == 130


@pytest.mark.parametrize(("limit_kph", "expected_kph"), ((40, 120), (90, 130)))
def test_non_tesla_pcm_confirmation_preserves_upstream_behavior(limit_kph, expected_kph):
  target = resolve_pcm_long_required_max(True, limit_kph, True, brand="toyota")
  assert round(target * CV.MS_TO_KPH) == expected_kph
