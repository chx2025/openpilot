import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from openpilot.selfdrive.debug import device_console as tesla_turn_signal_web


@pytest.fixture(autouse=True)
def authenticated_api(monkeypatch):
  # Endpoint behavior is covered here; authentication policy has dedicated
  # unit tests and must not depend on the machine's persistent Params store.
  monkeypatch.setattr(tesla_turn_signal_web.TurnSignalHandler, "_authorize_api", lambda self: True)


def test_turn_signal_web_page_exposes_sp_driven_actions_and_cancel(monkeypatch):
  monkeypatch.setattr(tesla_turn_signal_web, "driving_status_enabled", lambda: True)
  page = tesla_turn_signal_web.render_page().decode()
  assert "行驶信息" in page
  assert "左转" in page
  assert "右转" in page
  assert "设备控制台访问令牌" in page
  assert "速度按钮" in page
  assert "123456" not in page
  assert "立即取消" in page
  assert "card 实时线程" in page
  assert "HW4 Model Y" in page
  assert "车道" in page
  assert "周边目标" in page
  assert "交通控制" in page


def test_turn_signal_web_returns_read_only_driving_status(monkeypatch):
  snapshot = {"onroad": True, "speed_kph": 42.0}
  monkeypatch.setattr(tesla_turn_signal_web, "driving_status_enabled", lambda: True)
  monkeypatch.setattr(tesla_turn_signal_web, "driving_status_snapshot", lambda: snapshot)
  server = ThreadingHTTPServer(("127.0.0.1", 0), tesla_turn_signal_web.TurnSignalHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/driving-status", timeout=2) as response:
      assert json.loads(response.read()) == snapshot
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_turn_signal_web_blocks_driving_status_when_disabled(monkeypatch):
  monkeypatch.setattr(tesla_turn_signal_web, "driving_status_enabled", lambda: False)
  monkeypatch.setattr(tesla_turn_signal_web, "driving_status_snapshot",
                      lambda: (_ for _ in ()).throw(PermissionError("功能未开启")))
  assert "driving-tab\" disabled" in tesla_turn_signal_web.render_page().decode()

  server = ThreadingHTTPServer(("127.0.0.1", 0), tesla_turn_signal_web.TurnSignalHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    with pytest.raises(urllib.error.HTTPError) as exc_info:
      urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/driving-status", timeout=2)
    assert exc_info.value.code == 403
    assert json.loads(exc_info.value.read())["message"] == "功能未开启"
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_turn_signal_web_post_runs_requested_direction(monkeypatch):
  requested = []
  monkeypatch.setattr(tesla_turn_signal_web, "_ACTIVE_WEB_TEST_ID", None)
  monkeypatch.setattr(tesla_turn_signal_web, "start_validation_session",
                      lambda direction: requested.append(direction) or "test-session")
  server = ThreadingHTTPServer(("127.0.0.1", 0), tesla_turn_signal_web.TurnSignalHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/api/turn/left", method="POST")
    with urllib.request.urlopen(request, timeout=2) as response:
      payload = json.loads(response.read())
    assert payload["ok"] is True
    assert payload["test_id"] == "test-session"
    assert requested == ["left"]
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_turn_signal_web_status_and_cancel(monkeypatch):
  cancelled = []
  monkeypatch.setattr(tesla_turn_signal_web, "_ACTIVE_WEB_TEST_ID", "test-session")
  monkeypatch.setattr(tesla_turn_signal_web, "_ACTIVE_WEB_SESSION_STARTED", tesla_turn_signal_web.time.monotonic())
  monkeypatch.setattr(tesla_turn_signal_web, "get_validation_status",
                      lambda test_id: {"test_id": test_id, "done": False, "phase": "lane_changing"})
  monkeypatch.setattr(tesla_turn_signal_web, "cancel_validation_session", cancelled.append)
  server = ThreadingHTTPServer(("127.0.0.1", 0), tesla_turn_signal_web.TurnSignalHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/status/test-session", timeout=2) as response:
      payload = json.loads(response.read())
    assert payload["phase"] == "lane_changing"

    request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/api/cancel/test-session", method="POST")
    with urllib.request.urlopen(request, timeout=2) as response:
      payload = json.loads(response.read())
    assert payload["ok"] is True
    assert cancelled == ["test-session"]
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_turn_signal_web_expires_stale_queued_session(monkeypatch):
  cancelled = []
  monkeypatch.setattr(tesla_turn_signal_web, "_ACTIVE_WEB_TEST_ID", "stale-session")
  monkeypatch.setattr(tesla_turn_signal_web, "_ACTIVE_WEB_SESSION_STARTED", 1.0)
  monkeypatch.setattr(tesla_turn_signal_web.time, "monotonic",
                      lambda: 1.0 + tesla_turn_signal_web.WEB_SESSION_TIMEOUT_S)
  monkeypatch.setattr(tesla_turn_signal_web, "cancel_validation_session", cancelled.append)
  server = ThreadingHTTPServer(("127.0.0.1", 0), tesla_turn_signal_web.TurnSignalHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/status/stale-session", timeout=2) as response:
      payload = json.loads(response.read())
    assert payload["done"] is True
    assert payload["result"] == "WEB_SESSION_TIMEOUT"
    assert cancelled == ["stale-session"]
    assert tesla_turn_signal_web._ACTIVE_WEB_TEST_ID is None
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)
