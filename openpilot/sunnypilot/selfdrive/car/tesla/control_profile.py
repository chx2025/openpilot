"""Configuration adapter between openpilot Params and the Tesla opendbc module.

Keep the generic car interface unaware of individual Tesla feature switches.  A
single snapshot is taken during CarParams initialization; dynamic switches that
are explicitly supported by opendbc are read there at runtime.
"""

from collections.abc import Mapping
from enum import IntEnum
from typing import Protocol


class ParamReader(Protocol):
  def get(self, key: str, block: bool = False, encoding: str | None = None,
          return_default: bool = False) -> bytes | str | None: ...


# This is the complete initialization Interface consumed by
# opendbc.sunnypilot.car.interfaces.  Adding a Tesla setting should change this
# Module, the Params declaration, and its owning opendbc test together.
INITIALIZATION_KEYS = (
  "TeslaCoopSteering",
  "TeslaMadsScreenButton",
  "TeslaARS408Radar",
  "DynamicAutoStock",
  "DynamicAutoStockSpeedKph",
  "DynamicAutoStockSpeedLowKph",
  "DynamicAutoStockBlinkerToSP",
  "DynamicAutoStockCurveToSP",
  "TeslaApHybrid",
  "TeslaDynamicApLongitudinal",
  "TeslaSpeedButtonValidation",
  "TeslaTurnSignalValidation",
)


class TeslaRadarBackend(IntEnum):
  OEM = 0
  ARS408 = 1
  DISABLED = 2


def normalize_mads_screen_button(raw: object) -> int:
  """Map the retired four-button UI encoding to Off/3-finger/5-finger.

  The old UI stored 3 for 5 fingers. Value 2 is already the current 5-finger
  enum, even though an intermediate UI mislabeled it as 4 fingers.
  """
  try:
    value = int(raw)
  except (TypeError, ValueError):
    return 0
  return 2 if value == 3 else value if value in (0, 1, 2) else 0


def initialization_snapshot(params: ParamReader) -> list[dict[str, bytes | str | None]]:
  """Return the stable Params payload passed across the opendbc Seam."""
  return [{key: params.get(key, return_default=True)} for key in INITIALIZATION_KEYS]


def snapshot_as_dict(params: ParamReader) -> Mapping[str, bytes | str | None]:
  """Dictionary form used by diagnostics and tests."""
  return {key: params.get(key, return_default=True) for key in INITIALIZATION_KEYS}
