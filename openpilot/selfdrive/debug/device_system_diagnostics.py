"""Bounded system-error diagnostics for local browser downloads."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
import json
from pathlib import Path
import subprocess
from openpilot.common.hardware.hw import Paths


MAX_SYSTEM_DIAGNOSTIC_BYTES = 4 * 1024 * 1024
MAX_JOURNAL_BYTES = 2 * 1024 * 1024
MAX_LAUNCH_LOG_BYTES = 512 * 1024
MAX_CRASH_LOG_BYTES = 1024 * 1024


@dataclass(frozen=True)
class DiagnosticFile:
  archive_name: str
  data: bytes


def _tail_bytes(data: bytes, limit: int) -> bytes:
  return data[-limit:] if len(data) > limit else data


def _tail_file(path: Path, limit: int) -> bytes:
  try:
    with path.open("rb") as file:
      file.seek(0, 2)
      size = file.tell()
      file.seek(max(0, size - limit))
      return file.read(limit)
  except OSError:
    return b""


def collect_system_diagnostics(
  *,
  journal_runner: Callable = subprocess.run,
  launch_log: Path = Path("/tmp/launch_log"),
  crash_log: Path | None = None,
) -> tuple[DiagnosticFile, ...]:
  """Collect current-boot warnings and local crash files without unbounded reads."""
  crash_log = crash_log or Path(Paths.crash_log_root()) / "error.log"
  diagnostics: list[DiagnosticFile] = []
  source_status: dict[str, str] = {}

  try:
    result = journal_runner(
      ["journalctl", "-b", "--no-pager", "-p", "warning", "-n", "4000", "-o", "short-iso"],
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      timeout=5,
      check=False,
    )
    journal = result.stdout if isinstance(result.stdout, bytes) else str(result.stdout or "").encode()
    journal = _tail_bytes(journal, MAX_JOURNAL_BYTES)
    if journal:
      diagnostics.append(DiagnosticFile("system/journal-warning-current-boot.log", journal))
    source_status["journal"] = f"exit={result.returncode} bytes={len(journal)}"
  except Exception as error:
    source_status["journal"] = f"unavailable: {type(error).__name__}: {error}"

  for name, path, limit in (
    ("launch_log", launch_log, MAX_LAUNCH_LOG_BYTES),
    ("error_log", crash_log, MAX_CRASH_LOG_BYTES),
  ):
    data = _tail_file(path, limit)
    if data:
      archive_name = "system/launch_log.txt" if name == "launch_log" else "system/error.log"
      diagnostics.append(DiagnosticFile(archive_name, data))
    source_status[name] = f"bytes={len(data)} path={path}"

  metadata = json.dumps({
    "generated_at": datetime.now(tz=UTC).isoformat(),
    "max_total_bytes": MAX_SYSTEM_DIAGNOSTIC_BYTES,
    "sources": source_status,
  }, ensure_ascii=False, indent=2).encode()
  result = (DiagnosticFile("system/diagnostics.json", metadata), *diagnostics)
  if sum(len(entry.data) for entry in result) > MAX_SYSTEM_DIAGNOSTIC_BYTES:
    raise RuntimeError("system diagnostic collection exceeded its hard size limit")
  return result
