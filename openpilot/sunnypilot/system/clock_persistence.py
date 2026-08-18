import datetime
from collections.abc import Callable

from openpilot.common.time_helpers import MAX_DATE, min_date, system_time_valid


LAST_KNOWN_TIME_PARAM = "LastKnownGoodTime"
FALLBACK_PARAMS = (LAST_KNOWN_TIME_PARAM, "LastUpdateTime")
WRITE_INTERVAL = datetime.timedelta(hours=6)
CHECK_INTERVAL_S = 5 * 60


def _normalize(value) -> datetime.datetime | None:
  if not isinstance(value, datetime.datetime):
    return None
  return value.astimezone(datetime.UTC).replace(tzinfo=None) if value.tzinfo is not None else value


def _valid_time(value) -> datetime.datetime | None:
  value = _normalize(value)
  return value if value is not None and min_date() < value < MAX_DATE else None


class ClockPersistence:
  """Restore and rate-limit persistence around upstream timed's GPS clock."""

  def __init__(self, params, *, monotonic: Callable[[], float], now: Callable[[], datetime.datetime],
               time_is_valid: Callable[[], bool] = system_time_valid):
    self.params = params
    self.monotonic = monotonic
    self.now = now
    self.time_is_valid = time_is_valid
    self._next_check = 0.0

  def restore(self, set_time: Callable[[datetime.datetime], None]) -> bool:
    if self.time_is_valid():
      return False
    candidates = (_valid_time(self.params.get(key)) for key in FALLBACK_PARAMS)
    restore_time = max((candidate for candidate in candidates if candidate is not None), default=None)
    if restore_time is None:
      return False
    set_time(restore_time)
    return True

  def persist_if_due(self) -> bool:
    monotonic_now = self.monotonic()
    if monotonic_now < self._next_check:
      return False
    self._next_check = monotonic_now + CHECK_INTERVAL_S
    if not self.time_is_valid():
      return False

    now = _valid_time(self.now())
    if now is None:
      return False
    previous = _valid_time(self.params.get(LAST_KNOWN_TIME_PARAM))
    if previous is not None and now - previous < WRITE_INTERVAL:
      return False
    self.params.put(LAST_KNOWN_TIME_PARAM, now, block=False)
    return True
