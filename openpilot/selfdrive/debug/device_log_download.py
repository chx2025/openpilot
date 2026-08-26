"""Safe, streaming route-log downloads for the local device console."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
import io
import json
from pathlib import Path
import re
from typing import BinaryIO
import zipfile

from openpilot.common.hardware.hw import Paths


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

  @property
  def files(self) -> tuple[LogFile, ...]:
    return tuple(file for segment in self.segments for file in segment.files)

  @property
  def total_bytes(self) -> int:
    return sum(file.size for file in self.files)

  def summary(self) -> dict:
    return {
      "start_ms": self.start_ms,
      "end_ms": self.end_ms,
      "segment_count": len(self.segments),
      "file_count": len(self.files),
      "total_bytes": self.total_bytes,
      "segments": [segment.name for segment in self.segments],
    }


def _log_root(root: str | Path | None = None) -> Path:
  candidate = Path(Paths.log_root() if root is None else root)
  return candidate.resolve()


def scan_log_segments(root: str | Path | None = None) -> tuple[LogSegment, ...]:
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
      if path.is_symlink() or not path.is_file() or not LOG_FILE_RE.fullmatch(path.name):
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


def available_log_range(root: str | Path | None = None) -> dict:
  segments = scan_log_segments(root)
  return {
    "available": bool(segments),
    "start_ms": min((segment.start_ms for segment in segments), default=None),
    "end_ms": max((segment.end_ms for segment in segments), default=None),
    "segment_count": len(segments),
    "max_range_seconds": MAX_LOG_RANGE_SECONDS,
    "max_download_bytes": MAX_DOWNLOAD_BYTES,
    "max_download_files": MAX_DOWNLOAD_FILES,
  }


def select_log_range(start_ms: int, end_ms: int, root: str | Path | None = None) -> LogSelection:
  if start_ms <= 0 or end_ms <= 0 or end_ms <= start_ms:
    raise ValueError("日志开始时间必须早于结束时间")
  if end_ms - start_ms > MAX_LOG_RANGE_SECONDS * 1000:
    raise ValueError(f"单次最多选择 {MAX_LOG_RANGE_SECONDS // 3600} 小时")

  selected = tuple(
    segment for segment in scan_log_segments(root)
    if segment.end_ms >= start_ms and segment.start_ms <= end_ms
  )
  selection = LogSelection(start_ms, end_ms, selected)
  if len(selection.files) > MAX_DOWNLOAD_FILES:
    raise ValueError(f"日志文件过多，单次最多 {MAX_DOWNLOAD_FILES} 个")
  if selection.total_bytes > MAX_DOWNLOAD_BYTES:
    raise ValueError(f"日志总大小超过 {MAX_DOWNLOAD_BYTES // (1024 ** 3)} GiB，请缩短时间范围")
  return selection


def download_filename(selection: LogSelection) -> str:
  def stamp(value_ms: int) -> str:
    return datetime.fromtimestamp(value_ms / 1000, tz=UTC).strftime("%Y%m%dT%H%M%SZ")
  return f"openpilot-logs-{stamp(selection.start_ms)}-{stamp(selection.end_ms)}.zip"


def _manifest(selection: LogSelection) -> bytes:
  payload = {
    "generated_at": datetime.now(tz=UTC).isoformat(),
    **selection.summary(),
    "files": [
      {"path": file.archive_name, "size": file.size, "modified_ns": file.modified_ns}
      for file in selection.files
    ],
  }
  return json.dumps(payload, ensure_ascii=False, indent=2).encode()


def stream_log_zip(selection: LogSelection, output: BinaryIO) -> None:
  # rlog/qlog are already compressed; ZIP_STORED avoids wasting device CPU.
  with zipfile.ZipFile(output, mode="w", compression=zipfile.ZIP_STORED,
                       allowZip64=True, strict_timestamps=False) as archive:
    archive.writestr("manifest.json", _manifest(selection))
    for file in selection.files:
      archive.write(file.path, arcname=file.archive_name)


def build_log_zip(selection: LogSelection) -> bytes:
  """Small in-memory helper used only by tests."""
  output = io.BytesIO()
  stream_log_zip(selection, output)
  return output.getvalue()
