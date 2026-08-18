from dataclasses import dataclass, field

from openpilot.sunnypilot.hardware.panda_startup import PandaStartup, PandaStartupResult
from openpilot.sunnypilot.hardware.profile import HardwareProfile


@dataclass
class FakePandaStartupIO:
  states: list[dict[str, bool]] = field(default_factory=list)
  enumeration_errors: int = 0
  now: float = 0.0
  resets: int = 0
  recovers: int = 0
  polls: int = 0

  def reset_internal(self) -> None:
    self.resets += 1

  def recover_internal(self) -> None:
    self.recovers += 1

  def list_internal(self) -> list[str]:
    if self.polls < self.enumeration_errors:
      self.polls += 1
      raise RuntimeError("SPI enumeration failed")
    state = self.states[min(self.polls, len(self.states) - 1)] if self.states else {}
    self.polls += 1
    return list(state)

  def is_bootstub(self, serial: str) -> bool:
    state = self.states[min(self.polls - 1, len(self.states) - 1)]
    return state[serial]

  def monotonic(self) -> float:
    return self.now

  def sleep(self, seconds: float) -> None:
    self.now += seconds


def test_standard_profile_preserves_alternating_reset_recover() -> None:
  io = FakePandaStartupIO()
  startup = PandaStartup(io, HardwareProfile.STANDARD)

  assert startup.prepare(0, lambda: False) == PandaStartupResult.RESET
  assert startup.prepare(1, lambda: False) == PandaStartupResult.RECOVER
  assert (io.resets, io.recovers) == (1, 1)


def test_c3xl_waits_for_slow_application_without_recovery() -> None:
  io = FakePandaStartupIO(states=[{}] * 20 + [{"spi": False}])
  startup = PandaStartup(io, HardwareProfile.C3XL)

  assert startup.prepare(0, lambda: False) == PandaStartupResult.APP_READY
  assert io.now == 10.0
  assert (io.resets, io.recovers) == (1, 0)


def test_c3xl_bootstub_is_returned_to_firmware_flash_path() -> None:
  io = FakePandaStartupIO(states=[{"spi": True}])
  startup = PandaStartup(io, HardwareProfile.C3XL)

  assert startup.prepare(0, lambda: False) == PandaStartupResult.BOOTSTUB_READY
  assert io.now == startup.C3XL_APP_TIMEOUT_S
  assert (io.resets, io.recovers) == (1, 0)


def test_c3xl_recovers_only_after_application_timeout() -> None:
  io = FakePandaStartupIO(states=[])
  startup = PandaStartup(io, HardwareProfile.C3XL)

  assert startup.prepare(0, lambda: False) == PandaStartupResult.RECOVER_AFTER_TIMEOUT
  assert io.now == startup.C3XL_APP_TIMEOUT_S + startup.C3XL_RECOVERY_SETTLE_S
  assert (io.resets, io.recovers) == (1, 1)


def test_c3xl_wait_is_interruptible() -> None:
  io = FakePandaStartupIO(states=[])
  startup = PandaStartup(io, HardwareProfile.C3XL)

  assert startup.prepare(0, lambda: io.now >= 2.0) == PandaStartupResult.INTERRUPTED
  assert io.now == 2.0
  assert (io.resets, io.recovers) == (1, 0)


def test_c3xl_tolerates_transient_enumeration_errors() -> None:
  io = FakePandaStartupIO(states=[{}, {}, {"spi": False}], enumeration_errors=2)
  startup = PandaStartup(io, HardwareProfile.C3XL)

  assert startup.prepare(0, lambda: False) == PandaStartupResult.APP_READY
  assert (io.resets, io.recovers) == (1, 0)
