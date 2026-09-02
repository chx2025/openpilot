import json

from openpilot.selfdrive.debug.onroad_block_log import OnroadBlockLogger


def test_records_only_new_onroad_block_states(tmp_path):
  logger = OnroadBlockLogger(tmp_path)
  startup = {"accepted_terms": True, "free_space": False}
  onroad = {"ignition": True, "device_temp_good": True}

  logger.update(ignition=False, started=False, startup_conditions=startup, onroad_conditions=onroad)
  logger.update(ignition=True, started=False, startup_conditions=startup, onroad_conditions=onroad)
  logger.update(ignition=True, started=False, startup_conditions=startup, onroad_conditions=onroad)
  logger.update(ignition=True, started=False, startup_conditions={**startup, "free_space": True},
                onroad_conditions={**onroad, "device_temp_good": False})
  logger.update(ignition=False, started=False, startup_conditions=startup, onroad_conditions=onroad)
  logger.update(ignition=True, started=False, startup_conditions=startup, onroad_conditions=onroad)

  records = [json.loads(line) for line in (tmp_path / "onroad-block.jsonl").read_text().splitlines()]
  assert len(records) == 3
  assert records[0]["event"] == "onroad_blocked"
  assert records[0]["blocked_startup"] == ["free_space"]
  assert records[0]["blocked_onroad"] == []
  assert records[1]["blocked_startup"] == []
  assert records[1]["blocked_onroad"] == ["device_temp_good"]
  assert records[2]["blocked_startup"] == ["free_space"]


def test_can_record_one_missing_ignition_snapshot(tmp_path):
  logger = OnroadBlockLogger(tmp_path)

  logger.update(ignition=False, started=False, startup_conditions={}, onroad_conditions={"ignition": False},
                details={"panda_states": [{"harness_status": "notConnected"}]}, record_missing_ignition=True)
  logger.update(ignition=False, started=False, startup_conditions={}, onroad_conditions={"ignition": False},
                details={"panda_states": [{"harness_status": "notConnected"}]}, record_missing_ignition=True)

  [record] = [json.loads(line) for line in (tmp_path / "onroad-block.jsonl").read_text().splitlines()]
  assert record["event"] == "onroad_waiting_for_ignition"
  assert record["blocked_onroad"] == ["ignition"]
  assert record["panda_states"] == [{"harness_status": "notConnected"}]


def test_rotation_bounds_file_count_and_size(tmp_path):
  logger = OnroadBlockLogger(tmp_path, max_bytes=256, backups=2)

  for index in range(20):
    logger.update(
      ignition=True,
      started=False,
      startup_conditions={f"condition_{index}": False},
      onroad_conditions={"ignition": True},
    )

  files = sorted(tmp_path.glob("onroad-block.jsonl*"))
  assert len(files) == 3
  assert all(path.stat().st_size <= 256 for path in files)


def test_log_io_failure_never_blocks_onroad_decision(tmp_path, monkeypatch):
  logger = OnroadBlockLogger(tmp_path)
  monkeypatch.setattr(logger, "_append", lambda _record: (_ for _ in ()).throw(OSError("read-only")))

  logger.update(
    ignition=True,
    started=False,
    startup_conditions={"free_space": False},
    onroad_conditions={"ignition": True},
  )
