from openpilot.selfdrive.debug.unknown_can_observer import UnknownCanObserver


def test_unknown_can_observer_groups_bus_rate_bytes_and_recent_samples() -> None:
  observer = UnknownCanObserver()
  observer.update([
    (1_000_000_000, [(0x37A, bytes.fromhex("00 10 20 30"), 2), (0x123, b"ignored", 2)]),
    (1_100_000_000, [(0x37A, bytes.fromhex("00 11 20 35"), 2)]),
    (1_200_000_000, [(0x3A9, bytes.fromhex("AA BB"), 4)]),
  ])

  snapshot = observer.snapshot(1_300_000_000)

  assert snapshot["available"]
  assert snapshot["window_seconds"] == 60
  assert snapshot["target_addresses"] == ["0x37A", "0x3A9", "0x3B1"]
  first = snapshot["frames"][0]
  assert first["address"] == "0x37A"
  assert first["source"] == 2
  assert first["sample_count"] == 2
  assert first["total_count"] == 2
  assert first["dlc_counts"] == {"4": 2}
  assert first["frequency_hz"] == 10.0
  assert first["median_period_ms"] == 100.0
  assert first["latest_hex"] == "00112035"
  assert first["byte_stats"] == [
    {"index": 0, "min": 0, "max": 0, "changes": 0},
    {"index": 1, "min": 16, "max": 17, "changes": 1},
    {"index": 2, "min": 32, "max": 32, "changes": 0},
    {"index": 3, "min": 48, "max": 53, "changes": 1},
  ]
  assert first["recent_samples"][-1] == {"age_ms": 200.0, "hex": "00112035"}


def test_unknown_can_observer_expires_window_but_keeps_lifetime_count() -> None:
  observer = UnknownCanObserver()
  observer.update([(1_000_000_000, [(0x3B1, b"\x01", 2)])])

  snapshot = observer.snapshot(62_000_000_000)

  assert not snapshot["available"]
  assert snapshot["frames"] == []
  assert snapshot["lifetime_counts"] == {"0x3B1/source2": 1}
