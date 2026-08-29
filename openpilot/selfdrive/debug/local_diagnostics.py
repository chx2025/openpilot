"""Small, bounded C3XL diagnostics stream for local-only feature messages.

The files contain concatenated cereal Event messages inside a zstd frame, so
they remain readable with LogReader without placing these messages in route
rlog/qlog files.  A .partial file is never offered for download.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import time
from typing import BinaryIO

import zstandard as zstd

from openpilot.cereal import messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.hw import Paths


# The service registry is the single source of truth. Standard hardware keeps
# normal rlog/qlog behavior; C3XL loggerd diverts entries with this marker here.
LOCAL_DIAGNOSTIC_SERVICES: dict[str, int] = {
  name: service.local_diagnostic_decimation
  for name, service in SERVICE_LIST.items()
  if service.local_diagnostic_decimation is not None
}

DEFAULT_MAX_UNCOMPRESSED_BYTES = 8 * 1024 * 1024
DEFAULT_MAX_FILE_AGE_SECONDS = 5 * 60
DEFAULT_MAX_FILES = 8
DEFAULT_MAX_TOTAL_BYTES = 64 * 1024 * 1024
LOCAL_DIAGNOSTIC_RE = re.compile(
  r"^spdiag-(?P<start>[0-9]{13})-(?P<end>[0-9]{13})-(?P<sequence>[0-9]{6})\.zst$"
)


def local_diagnostic_root(root: str | Path | None = None) -> Path:
  if root is not None:
    return Path(root).resolve()
  return (Path(Paths.log_root()).resolve().parent / "spdiagnostics").resolve()


@dataclass(frozen=True)
class LocalDiagnosticFile:
  path: Path
  name: str
  start_ms: int
  end_ms: int
  size: int
  modified_ns: int

  @property
  def archive_name(self) -> str:
    return f"local-diagnostics/{self.name}"


def scan_local_diagnostics(root: str | Path | None = None) -> tuple[LocalDiagnosticFile, ...]:
  diagnostic_root = local_diagnostic_root(root)
  if not diagnostic_root.is_dir() or diagnostic_root.is_symlink():
    return ()

  files: list[LocalDiagnosticFile] = []
  try:
    paths = tuple(diagnostic_root.iterdir())
    resolved_root = diagnostic_root.resolve()
  except OSError:
    return ()

  for path in paths:
    match = LOCAL_DIAGNOSTIC_RE.fullmatch(path.name)
    if match is None or path.is_symlink() or not path.is_file():
      continue
    try:
      resolved = path.resolve()
      file_stat = resolved.stat()
    except OSError:
      continue
    if resolved.parent != resolved_root:
      continue
    files.append(LocalDiagnosticFile(
      path=resolved,
      name=path.name,
      start_ms=int(match.group("start")),
      end_ms=int(match.group("end")),
      size=file_stat.st_size,
      modified_ns=file_stat.st_mtime_ns,
    ))
  return tuple(sorted(files, key=lambda file: (file.start_ms, file.name)))


class LocalDiagnosticWriter:
  def __init__(self, root: str | Path | None = None, *,
               max_uncompressed_bytes: int = DEFAULT_MAX_UNCOMPRESSED_BYTES,
               max_file_age_seconds: int = DEFAULT_MAX_FILE_AGE_SECONDS,
               max_files: int = DEFAULT_MAX_FILES,
               max_total_bytes: int = DEFAULT_MAX_TOTAL_BYTES) -> None:
    if min(max_uncompressed_bytes, max_file_age_seconds, max_files, max_total_bytes) <= 0:
      raise ValueError("diagnostic rotation limits must be positive")
    self.root = local_diagnostic_root(root)
    self.max_uncompressed_bytes = max_uncompressed_bytes
    self.max_file_age_ms = max_file_age_seconds * 1000
    self.max_files = max_files
    self.max_total_bytes = max_total_bytes
    self._raw_file: BinaryIO | None = None
    self._zstd_writer: BinaryIO | None = None
    self._partial_path: Path | None = None
    self._start_ms: int | None = None
    self._uncompressed_bytes = 0
    self._sequence = 0

  def _open(self, wall_time_ms: int) -> None:
    self.root.mkdir(parents=True, exist_ok=True, mode=0o775)
    self._start_ms = wall_time_ms
    self._partial_path = self.root / f"spdiag-{wall_time_ms:013d}-{os.getpid()}-{self._sequence:06d}.partial"
    self._sequence += 1
    self._raw_file = self._partial_path.open("xb")
    self._zstd_writer = zstd.ZstdCompressor(level=3).stream_writer(self._raw_file, closefd=False)
    self._uncompressed_bytes = 0

  def _should_rotate(self, payload_size: int, wall_time_ms: int) -> bool:
    return bool(
      self._zstd_writer is not None and self._uncompressed_bytes > 0 and
      (self._uncompressed_bytes + payload_size > self.max_uncompressed_bytes or
       wall_time_ms - (self._start_ms or wall_time_ms) >= self.max_file_age_ms)
    )

  def write(self, payload: bytes, *, wall_time_ms: int | None = None) -> None:
    if not payload:
      return
    now_ms = time.time_ns() // 1_000_000 if wall_time_ms is None else wall_time_ms
    if self._should_rotate(len(payload), now_ms):
      self.close(wall_time_ms=now_ms)
    if self._zstd_writer is None:
      self._open(now_ms)
    assert self._zstd_writer is not None
    self._zstd_writer.write(payload)
    self._uncompressed_bytes += len(payload)

  def close(self, *, wall_time_ms: int | None = None) -> None:
    if self._zstd_writer is None or self._raw_file is None or self._partial_path is None or self._start_ms is None:
      return
    end_ms = max(self._start_ms, time.time_ns() // 1_000_000 if wall_time_ms is None else wall_time_ms)
    final_path = self.root / f"spdiag-{self._start_ms:013d}-{end_ms:013d}-{self._sequence - 1:06d}.zst"
    try:
      self._zstd_writer.flush(zstd.FLUSH_FRAME)
      self._zstd_writer.close()
      self._raw_file.flush()
      os.fsync(self._raw_file.fileno())
      self._raw_file.close()
      os.replace(self._partial_path, final_path)
      final_path.chmod(0o644)
    finally:
      if not self._raw_file.closed:
        self._raw_file.close()
      self._zstd_writer = None
      self._raw_file = None
      self._partial_path = None
      self._start_ms = None
      self._uncompressed_bytes = 0
    self._prune()

  def _prune(self) -> None:
    files = list(scan_local_diagnostics(self.root))
    total_bytes = sum(file.size for file in files)
    while files and (len(files) > self.max_files or total_bytes > self.max_total_bytes):
      oldest = files.pop(0)
      try:
        oldest.path.unlink()
        total_bytes -= oldest.size
      except OSError:
        pass


def main() -> None:
  services = tuple(name for name in LOCAL_DIAGNOSTIC_SERVICES if name in SERVICE_LIST)
  poller = messaging.Poller()
  for service in services:
    messaging.sub_sock(service, poller=poller, conflate=False)
  counters = dict.fromkeys(services, 0)
  writer = LocalDiagnosticWriter()
  try:
    while True:
      for socket in poller.poll(1000):
        for payload in messaging.drain_sock_raw(socket):
          service = messaging.log_from_bytes(payload).which()
          if service not in counters:
            continue
          counter = counters[service]
          counters[service] = counter + 1
          if counter % LOCAL_DIAGNOSTIC_SERVICES[service] == 0:
            writer.write(payload)
  finally:
    writer.close()


if __name__ == "__main__":
  main()
