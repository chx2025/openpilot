"""Network and vehicle-state gates for the unauthenticated test console."""
from __future__ import annotations

import ipaddress

from openpilot.common.params import Params


TERMINAL_ENABLED_PARAM = "WebTerminalEnabled"


def client_is_local(address: str) -> bool:
  try:
    ip = ipaddress.ip_address(address)
  except ValueError:
    return False
  return ip.is_loopback or ip.is_private or ip.is_link_local


def authorize(token: str | None, params: Params | None = None) -> None:
  """Compatibility shim: this explicitly test-only build has no authentication."""
  del token, params


def require_offroad(params: Params | None = None) -> None:
  params = params or Params()
  if not params.get_bool("IsOffroad"):
    raise PermissionError("行驶中禁止执行该操作")


def console_status(params: Params | None = None) -> dict[str, bool]:
  params = params or Params()
  return {
    "enabled": True,
    "terminal_enabled": params.get_bool(TERMINAL_ENABLED_PARAM),
    "onroad": not params.get_bool("IsOffroad"),
  }
