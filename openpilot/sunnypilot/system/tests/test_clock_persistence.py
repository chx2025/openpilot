import datetime

from openpilot.sunnypilot.system.clock_persistence import ClockPersistence, WRITE_INTERVAL


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}
    self.puts = []

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value
    self.puts.append((key, value, block))


def test_restore_uses_newest_valid_persisted_time():
  older = datetime.datetime(2026, 8, 8, 12, 0)
  newer = datetime.datetime(2026, 8, 9, 2, 39)
  params = FakeParams({"LastKnownGoodTime": older, "LastUpdateTime": newer})
  restored = []
  persistence = ClockPersistence(params, monotonic=lambda: 0.0, now=lambda: newer, time_is_valid=lambda: False)

  assert persistence.restore(restored.append)
  assert restored == [newer]


def test_restore_rejects_invalid_persisted_time():
  params = FakeParams({"LastKnownGoodTime": datetime.datetime(2024, 1, 1)})
  restored = []
  persistence = ClockPersistence(params, monotonic=lambda: 0.0, now=lambda: datetime.datetime(2026, 8, 9),
                                 time_is_valid=lambda: False)

  assert not persistence.restore(restored.append)
  assert restored == []


def test_persist_is_check_and_write_limited():
  now = datetime.datetime(2026, 8, 9, 6, 0)
  monotonic_now = [0.0]
  params = FakeParams({"LastKnownGoodTime": now - datetime.timedelta(hours=1)})
  persistence = ClockPersistence(params, monotonic=lambda: monotonic_now[0], now=lambda: now,
                                 time_is_valid=lambda: True)

  assert not persistence.persist_if_due()
  monotonic_now[0] += 5 * 60
  later = now + WRITE_INTERVAL
  persistence.now = lambda: later
  assert persistence.persist_if_due()
  assert params.puts == [("LastKnownGoodTime", later, False)]
