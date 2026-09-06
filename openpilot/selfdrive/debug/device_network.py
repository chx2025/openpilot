"""Read the device's LAN IPv4 address without contacting an external host."""
import ipaddress
import json
import subprocess
import threading
import time

_LOCK = threading.Lock()
_NEXT_REFRESH = 0.0
_ADDRESS: str | None = None


def select_lan_ipv4(interfaces: list[dict]) -> str | None:
  candidates = []
  for interface in interfaces:
    name = interface.get("ifname", "")
    if interface.get("operstate") == "DOWN" or not name.startswith(("wl", "eth", "en", "ap", "usb")):
      continue
    for item in interface.get("addr_info", []):
      if item.get("family") != "inet" or item.get("scope") != "global":
        continue
      try:
        address = ipaddress.IPv4Address(item.get("local", ""))
      except ipaddress.AddressValueError:
        continue
      if address.is_loopback or address.is_link_local or address.is_unspecified or address.is_multicast:
        continue
      candidates.append((not name.startswith(("wl", "ap")), name, str(address)))
  return min(candidates)[2] if candidates else None


def device_ip_address() -> str | None:
  global _NEXT_REFRESH, _ADDRESS
  with _LOCK:
    now = time.monotonic()
    if now >= _NEXT_REFRESH:
      try:
        result = subprocess.run(["ip", "-j", "-4", "address", "show", "up"], capture_output=True, text=True, timeout=1, check=True)
        _ADDRESS = select_lan_ipv4(json.loads(result.stdout))
      except (OSError, subprocess.SubprocessError, ValueError, TypeError, AttributeError):
        _ADDRESS = None
      _NEXT_REFRESH = time.monotonic() + 5
    return _ADDRESS
