import pytest

from openpilot.sunnypilot.selfdrive.car.tesla.control_profile import (
  INITIALIZATION_KEYS, TeslaRadarBackend, initialization_snapshot, normalize_mads_screen_button,
)


class FakeParams:
  def __init__(self):
    self.requested = []

  def get(self, key, block=False, encoding=None, return_default=False):
    self.requested.append((key, return_default))
    return f"value:{key}"


def test_initialization_snapshot_is_complete_and_ordered():
  params = FakeParams()

  snapshot = initialization_snapshot(params)

  assert [next(iter(item)) for item in snapshot] == list(INITIALIZATION_KEYS)
  assert params.requested == [(key, True) for key in INITIALIZATION_KEYS]
  assert len(INITIALIZATION_KEYS) == len(set(INITIALIZATION_KEYS))
  assert "TeslaTurnSignalValidation" in INITIALIZATION_KEYS
  assert "TeslaSpeedButtonValidation" in INITIALIZATION_KEYS


def test_radar_backend_values_match_opendbc_initialization_contract():
  assert tuple(TeslaRadarBackend) == (
    TeslaRadarBackend.OEM,
    TeslaRadarBackend.ARS408,
    TeslaRadarBackend.DISABLED,
  )
  assert [int(backend) for backend in TeslaRadarBackend] == [0, 1, 2]


@pytest.mark.parametrize(("raw", "expected"), [
  (None, 0), ("bad", 0), (-1, 0), (0, 0), (1, 1), (2, 2), (3, 2), (4, 0),
])
def test_mads_screen_button_normalizes_retired_ui_values(raw, expected):
  assert normalize_mads_screen_button(raw) == expected
