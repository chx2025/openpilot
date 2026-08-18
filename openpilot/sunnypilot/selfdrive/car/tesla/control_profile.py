"""Configuration adapter between openpilot Params and the Tesla opendbc module.

Keep the generic car interface unaware of individual Tesla feature switches.  A
single snapshot is taken during CarParams initialization; dynamic switches that
are explicitly supported by opendbc are read there at runtime.
"""

from collections.abc import Mapping
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


def initialization_snapshot(params: ParamReader) -> list[dict[str, bytes | str | None]]:
  """Return the stable Params payload passed across the opendbc Seam."""
  return [{key: params.get(key, return_default=True)} for key in INITIALIZATION_KEYS]


def snapshot_as_dict(params: ParamReader) -> Mapping[str, bytes | str | None]:
  """Dictionary form used by diagnostics and tests."""
  return {key: params.get(key, return_default=True) for key in INITIALIZATION_KEYS}
