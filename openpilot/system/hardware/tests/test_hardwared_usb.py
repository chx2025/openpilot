import queue

from openpilot.cereal import log
from openpilot.system.hardware import hardwared
from openpilot.system.hardware.hardwared import put_latest


def test_usb_edge_replaces_stale_queued_hardware_state():
  updates = queue.Queue(maxsize=1)
  updates.put_nowait("connected")

  put_latest(updates, "disconnected")

  assert updates.get_nowait() == "disconnected"


class StopAfterLoops:
  def __init__(self, loops: int):
    self.remaining = loops

  def is_set(self) -> bool:
    if self.remaining <= 0:
      return True
    self.remaining -= 1
    return False


def test_hardware_state_polls_usb_presence_every_cycle(monkeypatch):
  connected = [{"vendorId": 0x3801, "productId": 0x0001, "speedMbps": 5000}]
  usb_samples = iter((connected, []))
  monkeypatch.setattr(hardwared, "get_usb_state", lambda: next(usb_samples))
  monkeypatch.setattr(hardwared.time, "sleep", lambda _: None)
  monkeypatch.setattr(hardwared.HARDWARE, "get_network_type", lambda: log.DeviceState.NetworkType.none)
  monkeypatch.setattr(hardwared.HARDWARE, "get_modem_temperatures", list)
  monkeypatch.setattr(hardwared.HARDWARE, "get_modem_data_usage", lambda: (0, 0))
  monkeypatch.setattr(hardwared.HARDWARE, "get_network_info", lambda: None)
  monkeypatch.setattr(hardwared.HARDWARE, "get_network_strength", lambda _: log.DeviceState.NetworkStrength.unknown)
  monkeypatch.setattr(hardwared.HARDWARE, "get_network_metered", lambda _: False)

  updates = queue.Queue(maxsize=1)
  hardwared.hw_state_thread(StopAfterLoops(2), updates)

  assert updates.get_nowait().usb_state == []
