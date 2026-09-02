from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.debug.device_system_diagnostics import (
  MAX_SYSTEM_DIAGNOSTIC_BYTES,
  collect_system_diagnostics,
)


def test_collects_bounded_system_error_diagnostics(tmp_path):
  launch = tmp_path / "launch_log"
  crash = tmp_path / "error.log"
  launch.write_bytes(b"L" * (1024 * 1024))
  crash.write_bytes(b"C" * (2 * 1024 * 1024))

  def run(*args, **kwargs):
    return SimpleNamespace(returncode=0, stdout=b"J" * (5 * 1024 * 1024))

  diagnostics = collect_system_diagnostics(
    journal_runner=run,
    launch_log=launch,
    crash_log=crash,
  )

  by_name = {entry.archive_name: entry.data for entry in diagnostics}
  assert set(by_name) == {
    "system/diagnostics.json",
    "system/journal-warning-current-boot.log",
    "system/launch_log.txt",
    "system/error.log",
  }
  assert sum(len(entry.data) for entry in diagnostics) <= MAX_SYSTEM_DIAGNOSTIC_BYTES
  assert by_name["system/journal-warning-current-boot.log"].endswith(b"J" * 1024)
  assert by_name["system/launch_log.txt"].endswith(b"L" * 1024)
  assert by_name["system/error.log"].endswith(b"C" * 1024)


def test_missing_diagnostic_sources_still_produce_metadata(tmp_path):
  def fail(*args, **kwargs):
    raise TimeoutError("journal unavailable")

  diagnostics = collect_system_diagnostics(
    journal_runner=fail,
    launch_log=Path(tmp_path / "missing-launch"),
    crash_log=Path(tmp_path / "missing-error"),
  )

  assert [entry.archive_name for entry in diagnostics] == ["system/diagnostics.json"]


def test_collects_rotating_onroad_block_logs(tmp_path):
  onroad_root = tmp_path / "onroad-block"
  onroad_root.mkdir()
  (onroad_root / "onroad-block.jsonl").write_bytes(b"current\n")
  (onroad_root / "onroad-block.jsonl.1").write_bytes(b"older\n")

  diagnostics = collect_system_diagnostics(
    journal_runner=lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout=b""),
    launch_log=tmp_path / "missing-launch",
    crash_log=tmp_path / "missing-crash",
    onroad_block_root=onroad_root,
  )

  by_name = {entry.archive_name: entry.data for entry in diagnostics}
  assert by_name["system/onroad-block/onroad-block.jsonl"] == b"current\n"
  assert by_name["system/onroad-block/onroad-block.jsonl.1"] == b"older\n"
  assert sum(len(entry.data) for entry in diagnostics) <= MAX_SYSTEM_DIAGNOSTIC_BYTES
