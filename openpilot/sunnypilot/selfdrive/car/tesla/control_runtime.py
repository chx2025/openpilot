"""Runtime Adapter for Tesla split-control ownership.

opendbc owns the Tesla state machine and publishes compact runtime flags.  This
Module converts those flags into the only policy generic selfdrived needs: which
longitudinal/lateral owner is active and whether OEM cruise transition events
must be filtered to preserve the mixed-control session.
"""

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Protocol

from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.cereal import log


MAX_STATE_AGE_NS = 50_000_000
EventName = log.OnroadEvent.EventName


class EventCollection(Protocol):
  def has(self, event_name: int) -> bool: ...
  def remove(self, event_name: int) -> None: ...


class TeslaLongitudinalOwner(StrEnum):
  sp = "sp"
  stock_unknown = "stockUnknown"
  dynamic_stock = "dynamicStock"
  manual_stock = "manualStock"
  ap_hybrid_sp = "apHybridSp"
  ap_hybrid_stock = "apHybridStock"


@dataclass(frozen=True)
class TeslaControlState:
  flags: TeslaFlagsSP = field(default_factory=lambda: TeslaFlagsSP(0))

  @property
  def stock_longitudinal(self) -> bool:
    return bool(self.flags & TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE)

  @property
  def ap_hybrid(self) -> bool:
    return bool(self.flags & TeslaFlagsSP.AP_HYBRID_ACTIVE)

  @property
  def stock_lateral(self) -> bool:
    return bool(self.flags & TeslaFlagsSP.AP_HYBRID_STOCK_LATERAL_ACTIVE)

  @property
  def exit_recovery(self) -> bool:
    return bool(self.flags & TeslaFlagsSP.AP_HYBRID_EXIT_RECOVERY_ACTIVE)

  @property
  def longitudinal_owner(self) -> TeslaLongitudinalOwner:
    if self.ap_hybrid:
      return TeslaLongitudinalOwner.ap_hybrid_stock if self.stock_longitudinal else TeslaLongitudinalOwner.ap_hybrid_sp
    if self.flags & TeslaFlagsSP.DYNAMIC_STOCK_ACTIVE:
      return TeslaLongitudinalOwner.dynamic_stock
    if self.flags & TeslaFlagsSP.MANUAL_STOCK_ACTIVE:
      return TeslaLongitudinalOwner.manual_stock
    if self.stock_longitudinal:
      return TeslaLongitudinalOwner.stock_unknown
    return TeslaLongitudinalOwner.sp


def state_is_fresh(car_state_mono_time: int, car_state_sp_mono_time: int) -> bool:
  age = car_state_mono_time - car_state_sp_mono_time
  return car_state_sp_mono_time > 0 and 0 <= age <= MAX_STATE_AGE_NS


class TeslaControlRuntime:
  """Fail-closed view of current and previous Tesla control ownership."""

  def __init__(self, enabled: bool):
    self.enabled = enabled
    self.current = TeslaControlState()
    self.previous = TeslaControlState()

  def update(self, flags: int, car_state_mono_time: int, car_state_sp_mono_time: int) -> TeslaControlState:
    fresh_flags = flags if self.enabled and state_is_fresh(car_state_mono_time, car_state_sp_mono_time) else 0
    self.current = TeslaControlState(TeslaFlagsSP(fresh_flags))
    return self.current

  @property
  def split_control_transition(self) -> bool:
    return (self.enabled and
            (self.current.stock_longitudinal or self.previous.stock_longitudinal or
             self.current.ap_hybrid or self.previous.ap_hybrid or self.current.exit_recovery))

  def filter_transition_events(self, events: EventCollection) -> None:
    if not self.split_control_transition:
      return

    # Accelerator override deliberately remains: it suppresses only SP
    # longitudinal output while keeping the mixed-control session alive.
    suppressed = [
      EventName.buttonCancel,
      EventName.invalidLkasSetting,
      EventName.wrongCarMode,
      EventName.wrongCruiseMode,
      EventName.pcmDisable,
    ]
    if not self.current.exit_recovery:
      suppressed.append(EventName.accFaulted)

    for event in suppressed:
      if events.has(event):
        events.remove(event)

  def commit_cycle(self) -> None:
    self.previous = self.current
