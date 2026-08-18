"""Control the preconfigured local Wi-Fi hotspot used for 8088 access."""
import subprocess
from collections.abc import Callable


HOTSPOT_CONNECTION = "Hotspot"
HOTSPOT_URL = "http://192.168.43.1:8088"


def _nmcli(*args: str, runner: Callable = subprocess.run) -> subprocess.CompletedProcess:
  return runner(["sudo", "-n", "nmcli", *args], check=False, capture_output=True, text=True)


def hotspot_status(runner: Callable = subprocess.run) -> dict:
  """Return the state without exposing the hotspot passphrase."""
  result = _nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active", runner=runner)
  active = any(line.split(":", 1)[0] == HOTSPOT_CONNECTION for line in result.stdout.splitlines())
  return {
    "available": result.returncode == 0,
    "active": active,
    "connection": HOTSPOT_CONNECTION,
    "url": HOTSPOT_URL,
  }


def set_hotspot_enabled(enabled: bool, runner: Callable = subprocess.run) -> dict:
  command = ("connection", "up", "id", HOTSPOT_CONNECTION) if enabled else ("connection", "down", "id", HOTSPOT_CONNECTION)
  result = _nmcli(*command, runner=runner)
  if result.returncode != 0:
    message = (result.stderr or result.stdout or "NetworkManager 命令失败").strip()
    raise RuntimeError(message)
  return hotspot_status(runner=runner)
