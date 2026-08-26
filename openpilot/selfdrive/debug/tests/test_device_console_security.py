import shutil
import subprocess
from unittest.mock import Mock

import pytest

from openpilot.selfdrive.debug.device_console_auth import authorize, client_is_local
from openpilot.selfdrive.debug.device_console import render_page
from openpilot.selfdrive.debug.device_terminal import run_command


class FakeParams:
  def __init__(self, *, terminal=True, offroad=True, password="terminal-password"):
    self.values = {
      "WebTerminalEnabled": terminal,
      "WebTerminalPassword": password,
      "IsOffroad": offroad,
    }

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value


@pytest.mark.parametrize("address", ["127.0.0.1", "192.168.43.10", "10.0.0.4", "fe80::1"])
def test_local_network_addresses_are_allowed(address):
  assert client_is_local(address)


@pytest.mark.parametrize("address", ["8.8.8.8", "not-an-address"])
def test_public_or_invalid_addresses_are_rejected(address):
  assert not client_is_local(address)


def test_console_is_completely_unauthenticated_in_this_test_version():
  authorize(None, FakeParams())
  authorize("wrong", FakeParams())


def test_console_page_exposes_driving_information(monkeypatch):
  monkeypatch.setattr("openpilot.selfdrive.debug.device_console.driving_status_enabled", lambda: True)
  page = render_page().decode()

  assert "driving-tab" in page
  assert "driving-panel" in page
  assert "/api/driving-status" in page


def test_console_page_exposes_requested_vehicle_can_diagnostics(monkeypatch):
  monkeypatch.setattr("openpilot.selfdrive.debug.device_console.driving_status_enabled", lambda: True)
  page = render_page().decode()

  for address in ("0x238", "0x23E", "0x1FC", "0x132", "0x212", "0x219", "0x25A", "0x31F",
                  "0x266", "0x2E5", "0x315", "0x376", "0x3B6", "0x3D2", "0x3FE", "0x679"):
    assert address in page


def test_console_page_exposes_tesla_turn_signal_validation(monkeypatch):
  monkeypatch.setattr("openpilot.selfdrive.debug.device_console.driving_status_enabled", lambda: True)
  page = render_page().decode()

  assert "turn-tab" in page
  assert "/api/turn/" in page
  assert "左转" in page
  assert "右转" in page
  assert "立即取消" in page


def test_console_embedded_javascript_parses(monkeypatch):
  node = shutil.which("node")
  if node is None:
    pytest.skip("node is required to parse the embedded browser script")
  monkeypatch.setattr("openpilot.selfdrive.debug.device_console.driving_status_enabled", lambda: True)
  page = render_page().decode()
  script = page.split("<script>", 1)[1].split("</script>", 1)[0]

  result = subprocess.run([node, "--check", "-"], input=script, text=True, capture_output=True, check=False)

  assert result.returncode == 0, result.stderr


def test_terminal_requires_its_own_password():
  with pytest.raises(PermissionError, match="密码"):
    run_command("true", "wrong", FakeParams())


def test_terminal_is_offroad_only():
  with pytest.raises(PermissionError, match="行驶中"):
    run_command("true", "terminal-password", FakeParams(offroad=False))


def test_terminal_passes_command_as_bash_argument_without_python_shell(monkeypatch):
  proc = Mock()
  proc.pid = 123
  proc.poll.side_effect = [None, 0]
  proc.returncode = 0
  proc.stdout.read.side_effect = ["ok\n", ""]
  popen = Mock(return_value=proc)
  monkeypatch.setattr("openpilot.selfdrive.debug.device_terminal.subprocess.Popen", popen)
  monkeypatch.setattr("openpilot.selfdrive.debug.device_terminal.time.sleep", lambda _: None)

  result = run_command("printf ok", "terminal-password", FakeParams())

  assert result["output"] == "ok\n"
  args, kwargs = popen.call_args
  assert args[0] == ["/bin/bash", "-lc", "printf ok"]
  assert "shell" not in kwargs
