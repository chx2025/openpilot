import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from openpilot.selfdrive.debug import device_console


@pytest.fixture(autouse=True)
def local_api(monkeypatch):
  monkeypatch.setattr(device_console.DeviceConsoleHandler, "_authorize_api", lambda self: True)


@pytest.fixture
def server():
  httpd = ThreadingHTTPServer(("127.0.0.1", 0), device_console.DeviceConsoleHandler)
  thread = threading.Thread(target=httpd.serve_forever, daemon=True)
  thread.start()
  try:
    yield httpd
  finally:
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=2)


def test_driving_status_route_returns_snapshot(monkeypatch, server):
  snapshot = {"onroad": True, "speed_kph": 42.0}
  monkeypatch.setattr(device_console, "driving_status_snapshot", lambda: snapshot)

  with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/driving-status", timeout=2) as response:
    assert json.loads(response.read()) == snapshot


def test_driving_page_contains_oem_sp_comparison_charts(monkeypatch):
  monkeypatch.setattr(device_console, "driving_status_enabled", lambda: True)

  page = device_console.render_page().decode()

  for chart_id in ("lateral-chart", "accel-chart", "speed-chart", "lane-chart"):
    assert f'id="{chart_id}"' in page
  assert "0x488" in page
  assert "0x2B9" in page
  assert "0x209" in page
  assert "SP / FSD 车道线" in page


def test_turn_signal_route_starts_requested_direction(monkeypatch, server):
  requested = []
  monkeypatch.setattr(device_console, "_ACTIVE_WEB_TEST_ID", None)
  monkeypatch.setattr(device_console, "start_validation_session",
                      lambda direction: requested.append(direction) or "test-session")

  request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/api/turn/left", method="POST")
  with urllib.request.urlopen(request, timeout=2) as response:
    payload = json.loads(response.read())

  assert payload["ok"] is True
  assert payload["test_id"] == "test-session"
  assert requested == ["left"]


def test_turn_signal_status_and_cancel(monkeypatch, server):
  cancelled = []
  monkeypatch.setattr(device_console, "_ACTIVE_WEB_TEST_ID", "test-session")
  monkeypatch.setattr(device_console, "_ACTIVE_WEB_SESSION_STARTED", device_console.time.monotonic())
  monkeypatch.setattr(device_console, "get_validation_status",
                      lambda test_id: {"test_id": test_id, "done": False, "phase": "lane_changing"})
  monkeypatch.setattr(device_console, "cancel_validation_session", cancelled.append)

  with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/status/test-session", timeout=2) as response:
    assert json.loads(response.read())["phase"] == "lane_changing"

  request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/api/cancel/test-session", method="POST")
  with urllib.request.urlopen(request, timeout=2) as response:
    assert json.loads(response.read())["ok"] is True
  assert cancelled == ["test-session"]


def test_speed_validation_reports_safety_block(monkeypatch, server):
  monkeypatch.setattr(device_console, "run_validation", lambda action: (_ for _ in ()).throw(RuntimeError("disabled")))

  request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/api/speed/increase", method="POST")
  with pytest.raises(urllib.error.HTTPError) as exc_info:
    urllib.request.urlopen(request, timeout=2)

  assert exc_info.value.code == 503
  assert "测试被阻止" in json.loads(exc_info.value.read())["message"]
