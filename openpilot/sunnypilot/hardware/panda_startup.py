from collections.abc import Callable
from enum import StrEnum
import time

from panda import Panda

from openpilot.common.hardware import HARDWARE
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile


class PandaStartupResult(StrEnum):
  RESET = "reset"
  RECOVER = "recover"
  APP_READY = "app_ready"
  BOOTSTUB_READY = "bootstub_ready"
  RECOVER_AFTER_TIMEOUT = "recover_after_timeout"
  INTERRUPTED = "interrupted"


class PandaStartupIO:
  def reset_internal(self) -> None:
    HARDWARE.reset_internal_panda()

  def recover_internal(self) -> None:
    HARDWARE.recover_internal_panda()

  def list_internal(self) -> list[str]:
    return Panda.spi_list()

  def is_bootstub(self, serial: str) -> bool:
    with Panda(serial) as panda:
      return panda.bootstub

  def monotonic(self) -> float:
    return time.monotonic()

  def sleep(self, seconds: float) -> None:
    time.sleep(seconds)


class PandaStartup:
  C3XL_APP_TIMEOUT_S = 20.0
  C3XL_POLL_INTERVAL_S = 0.5
  C3XL_RECOVERY_SETTLE_S = 5.0

  def __init__(self, io: PandaStartupIO | None = None, profile: HardwareProfile | None = None):
    self.io = io or PandaStartupIO()
    self.profile = profile or get_hardware_profile()

  def prepare(self, attempt: int, should_exit: Callable[[], bool]) -> PandaStartupResult:
    if self.profile != HardwareProfile.C3XL:
      if (attempt % 2) == 0:
        self.io.reset_internal()
        return PandaStartupResult.RESET
      self.io.recover_internal()
      return PandaStartupResult.RECOVER

    self.io.reset_internal()
    deadline = self.io.monotonic() + self.C3XL_APP_TIMEOUT_S
    last_bootstub_visible = False

    while self.io.monotonic() < deadline:
      if should_exit():
        return PandaStartupResult.INTERRUPTED

      last_bootstub_visible = False
      try:
        serials = self.io.list_internal()
      except Exception:
        serials = []
      for serial in serials:
        try:
          if self.io.is_bootstub(serial):
            last_bootstub_visible = True
          else:
            return PandaStartupResult.APP_READY
        except Exception:
          continue

      remaining = deadline - self.io.monotonic()
      self.io.sleep(min(self.C3XL_POLL_INTERVAL_S, max(0.0, remaining)))

    if should_exit():
      return PandaStartupResult.INTERRUPTED
    if last_bootstub_visible:
      return PandaStartupResult.BOOTSTUB_READY

    self.io.recover_internal()
    recovery_deadline = self.io.monotonic() + self.C3XL_RECOVERY_SETTLE_S
    while self.io.monotonic() < recovery_deadline:
      if should_exit():
        return PandaStartupResult.INTERRUPTED
      remaining = recovery_deadline - self.io.monotonic()
      self.io.sleep(min(self.C3XL_POLL_INTERVAL_S, max(0.0, remaining)))
    return PandaStartupResult.RECOVER_AFTER_TIMEOUT
