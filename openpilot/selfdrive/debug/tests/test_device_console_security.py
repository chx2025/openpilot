from unittest.mock import Mock

import pytest

from openpilot.selfdrive.debug.device_console_auth import authorize, client_is_local
from openpilot.selfdrive.debug.device_terminal import run_command


class FakeParams:
  def __init__(self, *, enabled=True, terminal=True, offroad=True, token="device-token-123456"):
    self.values = {
      "DeviceConsoleEnabled": enabled,
      "WebTerminalEnabled": terminal,
      "IsOffroad": offroad,
      "DeviceConsoleToken": token,
    }

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value


@pytest.mark.parametrize("address", ["127.0.0.1", "192.168.43.10", "10.0.0.4", "fe80::1"])
def test_local_network_addresses_are_allowed(address):
  assert client_is_local(address)


@pytest.mark.parametrize("address", ["8.8.8.8", "not-an-address"])
def test_public_or_invalid_addresses_are_rejected(address):
  assert not client_is_local(address)


def test_console_requires_explicit_enable_and_matching_token():
  with pytest.raises(PermissionError):
    authorize("device-token-123456", FakeParams(enabled=False))
  with pytest.raises(PermissionError):
    authorize("wrong", FakeParams())
  authorize("device-token-123456", FakeParams())


def test_terminal_is_offroad_only():
  with pytest.raises(PermissionError, match="行驶中"):
    run_command("true", "device-token-123456", FakeParams(offroad=False))


def test_terminal_passes_command_as_bash_argument_without_python_shell(monkeypatch):
  proc = Mock()
  proc.pid = 123
  proc.poll.side_effect = [None, 0]
  proc.returncode = 0
  proc.stdout.read.side_effect = ["ok\n", ""]
  popen = Mock(return_value=proc)
  monkeypatch.setattr("openpilot.selfdrive.debug.device_terminal.subprocess.Popen", popen)
  monkeypatch.setattr("openpilot.selfdrive.debug.device_terminal.time.sleep", lambda _: None)

  result = run_command("printf ok", "device-token-123456", FakeParams())

  assert result["output"] == "ok\n"
  args, kwargs = popen.call_args
  assert args[0] == ["/bin/bash", "-lc", "printf ok"]
  assert "shell" not in kwargs
