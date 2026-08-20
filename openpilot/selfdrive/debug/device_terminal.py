"""Password-protected, opt-in and offroad-only arbitrary command terminal."""
from __future__ import annotations

import hmac
import os
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from openpilot.common.params import Params
from openpilot.selfdrive.debug.device_console_auth import console_status, require_offroad


BASE_DIR = Path(__file__).resolve().parents[3]
MAX_COMMAND_LENGTH = 4096
MAX_OUTPUT_LENGTH = 64 * 1024
COMMAND_TIMEOUT_S = 20.0


def terminal_status(params: Params | None = None) -> dict[str, bool]:
  return console_status(params)


def _authorize(password: str | None, params: Params) -> None:
  if not params.get_bool("WebTerminalEnabled"):
    raise PermissionError("网页终端未启用")
  expected = params.get("WebTerminalPassword", return_default=True)
  if not isinstance(expected, str) or not password or not hmac.compare_digest(password, expected):
    raise PermissionError("终端密码错误")
  require_offroad(params)


def _terminate(proc: subprocess.Popen) -> None:
  try:
    os.killpg(proc.pid, signal.SIGTERM)
    proc.wait(timeout=1.0)
  except (ProcessLookupError, subprocess.TimeoutExpired):
    try:
      os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass


def run_command(command: str, password: str | None, params: Params | None = None) -> dict[str, object]:
  params = params or Params()
  _authorize(password, params)
  if not isinstance(command, str) or not command.strip() or len(command) > MAX_COMMAND_LENGTH:
    raise ValueError("命令必须为 1 到 4096 个字符")

  # Bash receives the command as an argument, not through Python's shell=True.
  proc = subprocess.Popen(
    ["/bin/bash", "-lc", command], cwd=BASE_DIR,
    env={**os.environ, "PYTHONUNBUFFERED": "1"}, start_new_session=True,
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
  )
  output_chunks: deque[str] = deque()
  output_size = 0
  output_lock = threading.Lock()

  def drain_output() -> None:
    nonlocal output_size
    if proc.stdout is None:
      return
    while chunk := proc.stdout.read(4096):
      with output_lock:
        output_chunks.append(chunk)
        output_size += len(chunk)
        while output_size > MAX_OUTPUT_LENGTH and output_chunks:
          removed = output_chunks.popleft()
          output_size -= len(removed)

  reader = threading.Thread(target=drain_output, daemon=True)
  reader.start()
  deadline = time.monotonic() + COMMAND_TIMEOUT_S
  blocked_onroad = False
  timed_out = False
  while proc.poll() is None:
    if not params.get_bool("IsOffroad"):
      blocked_onroad = True
      _terminate(proc)
      break
    if time.monotonic() >= deadline:
      timed_out = True
      _terminate(proc)
      break
    time.sleep(0.1)
  reader.join(timeout=1.0)
  with output_lock:
    output = "".join(output_chunks)[-MAX_OUTPUT_LENGTH:]
  return {
    "exit_code": proc.returncode,
    "timed_out": timed_out,
    "blocked_onroad": blocked_onroad,
    "output": output,
  }


def change_password(current_password: str | None, new_password: str, params: Params | None = None) -> None:
  params = params or Params()
  _authorize(current_password, params)
  if not isinstance(new_password, str) or not 4 <= len(new_password) <= 64:
    raise ValueError("新密码必须为 4 到 64 个字符")
  params.put("WebTerminalPassword", new_password, block=True)
