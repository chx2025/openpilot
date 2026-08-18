"""Authentication and state gates for the local device console.

The console has no remote bootstrap secret and is disabled by default. The
token is generated locally, displayed on the device, and never returned by an
HTTP endpoint or included in logs.
"""
from __future__ import annotations

import hmac
import ipaddress
import secrets

from openpilot.common.params import Params


CONSOLE_ENABLED_PARAM = "DeviceConsoleEnabled"
CONSOLE_TOKEN_PARAM = "DeviceConsoleToken"
TERMINAL_ENABLED_PARAM = "WebTerminalEnabled"
TOKEN_BYTES = 18


def ensure_console_token(params: Params | None = None) -> str:
  params = params or Params()
  token = params.get(CONSOLE_TOKEN_PARAM)
  if isinstance(token, str) and len(token) >= 16:
    return token
  token = secrets.token_urlsafe(TOKEN_BYTES)
  params.put(CONSOLE_TOKEN_PARAM, token, block=True)
  return token


def rotate_console_token(params: Params | None = None) -> str:
  params = params or Params()
  token = secrets.token_urlsafe(TOKEN_BYTES)
  params.put(CONSOLE_TOKEN_PARAM, token, block=True)
  return token


def client_is_local(address: str) -> bool:
  try:
    ip = ipaddress.ip_address(address)
  except ValueError:
    return False
  return ip.is_loopback or ip.is_private or ip.is_link_local


def authorize(token: str | None, params: Params | None = None) -> None:
  params = params or Params()
  if not params.get_bool(CONSOLE_ENABLED_PARAM):
    raise PermissionError("本地网页控制台未启用")
  expected = ensure_console_token(params)
  if not token or not hmac.compare_digest(token, expected):
    raise PermissionError("设备控制台访问令牌错误")


def require_offroad(params: Params | None = None) -> None:
  params = params or Params()
  if not params.get_bool("IsOffroad"):
    raise PermissionError("行驶中禁止执行该操作")


def console_status(params: Params | None = None) -> dict[str, bool]:
  params = params or Params()
  return {
    "enabled": params.get_bool(CONSOLE_ENABLED_PARAM),
    "terminal_enabled": params.get_bool(TERMINAL_ENABLED_PARAM),
    "onroad": not params.get_bool("IsOffroad"),
  }
