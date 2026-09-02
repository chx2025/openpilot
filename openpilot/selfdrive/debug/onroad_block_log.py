"""Small rotating local log for ignition-present onroad startup failures."""
from __future__ import annotations

from datetime import UTC, datetime
import json
import os
from pathlib import Path


ONROAD_BLOCK_LOG_ROOT = Path("/data/onroad-block")
ONROAD_BLOCK_LOG_NAME = "onroad-block.jsonl"
ONROAD_BLOCK_LOG_MAX_BYTES = 128 * 1024
ONROAD_BLOCK_LOG_BACKUPS = 2


class OnroadBlockLogger:
  def __init__(self, root: str | Path = ONROAD_BLOCK_LOG_ROOT, *,
               max_bytes: int = ONROAD_BLOCK_LOG_MAX_BYTES,
               backups: int = ONROAD_BLOCK_LOG_BACKUPS) -> None:
    if max_bytes <= 0 or backups < 0:
      raise ValueError("onroad block log rotation limits are invalid")
    self.root = Path(root)
    self.path = self.root / ONROAD_BLOCK_LOG_NAME
    self.max_bytes = max_bytes
    self.backups = backups
    self._last_signature: tuple[tuple[str, ...], tuple[str, ...]] | None = None

  def _rotate(self, incoming_bytes: int) -> None:
    try:
      current_size = self.path.stat().st_size
    except OSError:
      current_size = 0
    if current_size == 0 or current_size + incoming_bytes <= self.max_bytes:
      return
    if self.backups == 0:
      self.path.unlink(missing_ok=True)
      return
    oldest = self.path.with_name(f"{self.path.name}.{self.backups}")
    oldest.unlink(missing_ok=True)
    for index in range(self.backups - 1, 0, -1):
      source = self.path.with_name(f"{self.path.name}.{index}")
      if source.exists() and not source.is_symlink():
        os.replace(source, self.path.with_name(f"{self.path.name}.{index + 1}"))
    if self.path.exists() and not self.path.is_symlink():
      os.replace(self.path, self.path.with_name(f"{self.path.name}.1"))

  def _append(self, record: dict) -> None:
    payload = (json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode()
    self.root.mkdir(parents=True, exist_ok=True, mode=0o775)
    self._rotate(len(payload))
    fd = os.open(self.path, os.O_APPEND | os.O_CREAT | os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW, 0o644)
    try:
      os.write(fd, payload)
    finally:
      os.close(fd)

  def _safe_append(self, record: dict) -> None:
    try:
      self._append(record)
    except OSError:
      pass

  def update(self, *, ignition: bool, started: bool,
             startup_conditions: dict[str, bool], onroad_conditions: dict[str, bool],
             details: dict | None = None, now: datetime | None = None,
             record_missing_ignition: bool = False) -> None:
    if not ignition:
      if record_missing_ignition:
        signature = ((), ("ignition",))
        if signature != self._last_signature:
          self._last_signature = signature
          self._safe_append({
            "timestamp": (now or datetime.now(tz=UTC)).isoformat(),
            "event": "onroad_waiting_for_ignition",
            "blocked_startup": [],
            "blocked_onroad": ["ignition"],
            **(details or {}),
          })
        return
      self._last_signature = None
      return
    if started:
      if self._last_signature is not None:
        self._safe_append({
          "timestamp": (now or datetime.now(tz=UTC)).isoformat(),
          "event": "onroad_started",
          **(details or {}),
        })
      self._last_signature = None
      return

    blocked_startup = tuple(sorted(name for name, passed in startup_conditions.items() if not passed))
    blocked_onroad = tuple(sorted(name for name, passed in onroad_conditions.items() if not passed and name != "ignition"))
    signature = (blocked_startup, blocked_onroad)
    if signature == self._last_signature:
      return
    self._last_signature = signature
    self._safe_append({
      "timestamp": (now or datetime.now(tz=UTC)).isoformat(),
      "event": "onroad_blocked",
      "blocked_startup": list(blocked_startup),
      "blocked_onroad": list(blocked_onroad),
      **(details or {}),
    })
