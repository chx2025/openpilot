"""Safe, streaming route-log downloads for the local device console."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
import io
import json
import os
from pathlib import Path
import re
import stat
from typing import BinaryIO
import zipfile

from openpilot.common.hardware.hw import Paths
from openpilot.selfdrive.debug.local_diagnostics import (
  LOCAL_DIAGNOSTIC_RE,
  LocalDiagnosticFile,
  local_diagnostic_root,
  scan_local_diagnostics,
)
from openpilot.selfdrive.debug.device_system_diagnostics import DiagnosticFile


MAX_LOG_RANGE_SECONDS = 72 * 60 * 60
MAX_DOWNLOAD_BYTES = 16 * 1024 * 1024 * 1024
MAX_DOWNLOAD_FILES = 20_000
SEGMENT_NAME_RE = re.compile(r"^[0-9a-f]{8}--[0-9a-f]{10}--[0-9]+$")
LOG_FILE_RE = re.compile(r"^(?:qlog|rlog)(?:\.(?:zst|bz2))?$")


@dataclass(frozen=True)
class LogFile:
  path: Path
  segment: str
  name: str
  size: int
  modified_ns: int

  @property
  def archive_name(self) -> str:
    return f"{self.segment}/{self.name}"


@dataclass(frozen=True)
class LogSegment:
  name: str
  start_ms: int
  end_ms: int
  files: tuple[LogFile, ...]


@dataclass(frozen=True)
class LogSelection:
  start_ms: int
  end_ms: int
  segments: tuple[LogSegment, ...]
  diagnostic_files: tuple[LocalDiagnosticFile, ...] = ()

  @property
  def files(self) -> tuple[LogFile, ...]:
    return tuple(file for segment in self.segments for file in segment.files)

  @property
  def total_bytes(self) -> int:
    return sum(file.size for file in self.files) + sum(file.size for file in self.diagnostic_files)

  def summary(self) -> dict:
    return {
      "start_ms": self.start_ms,
      "end_ms": self.end_ms,
      "segment_count": len(self.segments),
      "route_file_count": len(self.files),
      "local_diagnostic_count": len(self.diagnostic_files),
      "file_count": len(self.files) + len(self.diagnostic_files),
      "total_bytes": self.total_bytes,
      "segments": [segment.name for segment in self.segments],
    }


@dataclass(frozen=True)
class LogDeletion:
  segment_count: int
  file_count: int
  total_bytes: int
  skipped_files: tuple[str, ...]

  def summary(self) -> dict:
    return {
      "segment_count": self.segment_count,
      "file_count": self.file_count,
      "total_bytes": self.total_bytes,
      "skipped_files": list(self.skipped_files),
    }


def _log_root(root: str | Path | None = None) -> Path:
  candidate = Path(Paths.log_root() if root is None else root)
  return candidate.resolve()


def _diagnostic_root_for_route(root: str | Path | None,
                               diagnostic_root: str | Path | None) -> Path:
  if diagnostic_root is not None or root is None:
    return local_diagnostic_root(diagnostic_root)
  return local_diagnostic_root(Path(root).resolve().parent / "spdiagnostics")


def scan_log_segments(root: str | Path | None = None, *, include_rlog: bool = False) -> tuple[LogSegment, ...]:
  log_root = _log_root(root)
  if not log_root.is_dir():
    return ()

  segments: list[LogSegment] = []
  try:
    directories = tuple(log_root.iterdir())
  except OSError:
    return ()
  for directory in directories:
    if directory.is_symlink() or not directory.is_dir() or not SEGMENT_NAME_RE.fullmatch(directory.name):
      continue
    files: list[LogFile] = []
    try:
      paths = tuple(directory.iterdir())
      resolved_directory = directory.resolve()
    except OSError:
      continue
    for path in paths:
      if (path.is_symlink() or not path.is_file() or not LOG_FILE_RE.fullmatch(path.name) or
          (not include_rlog and not path.name.startswith("qlog"))):
        continue
      try:
        resolved = path.resolve()
        stat = resolved.stat()
      except OSError:
        continue
      if resolved.parent != resolved_directory:
        continue
      files.append(LogFile(resolved, directory.name, path.name, stat.st_size, stat.st_mtime_ns))
    if not files:
      continue
    files.sort(key=lambda file: file.name)
    try:
      directory_mtime_ns = directory.stat().st_mtime_ns
    except OSError:
      continue
    start_ns = min(directory_mtime_ns, *(file.modified_ns for file in files))
    end_ns = max(directory_mtime_ns, *(file.modified_ns for file in files))
    segments.append(LogSegment(
      name=directory.name,
      start_ms=start_ns // 1_000_000,
      end_ms=end_ns // 1_000_000,
      files=tuple(files),
    ))
  return tuple(sorted(segments, key=lambda segment: (segment.start_ms, segment.name)))


def available_log_range(root: str | Path | None = None, *,
                        diagnostic_root: str | Path | None = None) -> dict:
  segments = scan_log_segments(root, include_rlog=False)
  diagnostic_files = scan_local_diagnostics(_diagnostic_root_for_route(root, diagnostic_root))
  starts = [segment.start_ms for segment in segments] + [file.start_ms for file in diagnostic_files]
  ends = [segment.end_ms for segment in segments] + [file.end_ms for file in diagnostic_files]
  return {
    "available": bool(segments or diagnostic_files),
    "start_ms": min(starts, default=None),
    "end_ms": max(ends, default=None),
    "segment_count": len(segments),
    "local_diagnostic_count": len(diagnostic_files),
    "max_range_seconds": MAX_LOG_RANGE_SECONDS,
    "max_download_bytes": MAX_DOWNLOAD_BYTES,
    "max_download_files": MAX_DOWNLOAD_FILES,
  }


def select_log_range(start_ms: int, end_ms: int, root: str | Path | None = None, *,
                     include_rlog: bool = False, include_local_diagnostics: bool = True,
                     diagnostic_root: str | Path | None = None) -> LogSelection:
  if start_ms <= 0 or end_ms <= 0 or end_ms <= start_ms:
    raise ValueError("日志开始时间必须早于结束时间")
  if end_ms - start_ms > MAX_LOG_RANGE_SECONDS * 1000:
    raise ValueError(f"单次最多选择 {MAX_LOG_RANGE_SECONDS // 3600} 小时")

  selected = tuple(
    segment for segment in scan_log_segments(root, include_rlog=include_rlog)
    if segment.end_ms >= start_ms and segment.start_ms <= end_ms
  )
  diagnostics = tuple(
    file for file in scan_local_diagnostics(_diagnostic_root_for_route(root, diagnostic_root))
    if include_local_diagnostics and file.end_ms >= start_ms and file.start_ms <= end_ms
  )
  selection = LogSelection(start_ms, end_ms, selected, diagnostics)
  if len(selection.files) + len(selection.diagnostic_files) > MAX_DOWNLOAD_FILES:
    raise ValueError(f"日志文件过多，单次最多 {MAX_DOWNLOAD_FILES} 个")
  if selection.total_bytes > MAX_DOWNLOAD_BYTES:
    raise ValueError(f"日志总大小超过 {MAX_DOWNLOAD_BYTES // (1024 ** 3)} GiB，请缩短时间范围")
  return selection


def delete_log_selection(selection: LogSelection, root: str | Path | None = None, *,
                         diagnostic_root: str | Path | None = None) -> LogDeletion:
  """Delete unchanged selected route and local-diagnostic files."""
  log_root = _log_root(root)
  deleted_segments: set[str] = set()
  deleted_files = 0
  deleted_bytes = 0
  skipped_files: list[str] = []

  for file in selection.files:
    directory = log_root / file.segment
    if (not SEGMENT_NAME_RE.fullmatch(file.segment) or not LOG_FILE_RE.fullmatch(file.name) or
        file.path != directory / file.name):
      skipped_files.append(file.archive_name)
      continue

    directory_fd: int | None = None
    try:
      directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
      current = os.stat(file.name, dir_fd=directory_fd, follow_symlinks=False)
      if (not stat.S_ISREG(current.st_mode) or current.st_size != file.size or
          current.st_mtime_ns != file.modified_ns):
        skipped_files.append(file.archive_name)
        continue
      os.unlink(file.name, dir_fd=directory_fd)
      deleted_segments.add(file.segment)
      deleted_files += 1
      deleted_bytes += file.size
    except OSError:
      skipped_files.append(file.archive_name)
    finally:
      if directory_fd is not None:
        os.close(directory_fd)

  resolved_diagnostic_root = _diagnostic_root_for_route(root, diagnostic_root)
  for file in selection.diagnostic_files:
    if (not LOCAL_DIAGNOSTIC_RE.fullmatch(file.name) or
        file.path != resolved_diagnostic_root / file.name):
      skipped_files.append(file.archive_name)
      continue
    try:
      current = os.stat(file.path, follow_symlinks=False)
      if (not stat.S_ISREG(current.st_mode) or current.st_size != file.size or
          current.st_mtime_ns != file.modified_ns):
        skipped_files.append(file.archive_name)
        continue
      file.path.unlink()
      deleted_segments.add("local-diagnostics")
      deleted_files += 1
      deleted_bytes += file.size
    except OSError:
      skipped_files.append(file.archive_name)

  return LogDeletion(len(deleted_segments), deleted_files, deleted_bytes, tuple(skipped_files))


def download_filename(selection: LogSelection) -> str:
  def stamp(value_ms: int) -> str:
    return datetime.fromtimestamp(value_ms / 1000, tz=UTC).strftime("%Y%m%dT%H%M%SZ")
  return f"openpilot-logs-{stamp(selection.start_ms)}-{stamp(selection.end_ms)}.zip"


def _manifest(selection: LogSelection, diagnostics: tuple[DiagnosticFile, ...]) -> bytes:
  payload = {
    "generated_at": datetime.now(tz=UTC).isoformat(),
    **selection.summary(),
    "files": [
      {"path": file.archive_name, "size": file.size, "modified_ns": file.modified_ns}
      for file in selection.files
    ],
    "local_diagnostics": [
      {"path": file.archive_name, "size": file.size, "modified_ns": file.modified_ns,
       "start_ms": file.start_ms, "end_ms": file.end_ms}
      for file in selection.diagnostic_files
    ],
    "system_diagnostics": [
      {"path": entry.archive_name, "size": len(entry.data)}
      for entry in diagnostics
    ],
  }
  return json.dumps(payload, ensure_ascii=False, indent=2).encode()


def stream_log_zip(selection: LogSelection, output: BinaryIO,
                   *, diagnostics: tuple[DiagnosticFile, ...] = ()) -> None:
  # rlog/qlog are already compressed; ZIP_STORED avoids wasting device CPU.
  with zipfile.ZipFile(output, mode="w", compression=zipfile.ZIP_STORED,
                       allowZip64=True, strict_timestamps=False) as archive:
    archive.writestr("manifest.json", _manifest(selection, diagnostics))
    for file in selection.files:
      archive.write(file.path, arcname=file.archive_name)
    for file in selection.diagnostic_files:
      archive.write(file.path, arcname=file.archive_name)
    for entry in diagnostics:
      if (not entry.archive_name.startswith("system/") or ".." in Path(entry.archive_name).parts or
          Path(entry.archive_name).is_absolute()):
        raise ValueError("invalid system diagnostic archive path")
      archive.writestr(entry.archive_name, entry.data)


def build_log_zip(selection: LogSelection, *, diagnostics: tuple[DiagnosticFile, ...] = ()) -> bytes:
  """Small in-memory helper used only by tests."""
  output = io.BytesIO()
  stream_log_zip(selection, output, diagnostics=diagnostics)
  return output.getvalue()
