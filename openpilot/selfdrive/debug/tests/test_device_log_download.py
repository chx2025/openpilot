import json
import os
from pathlib import Path
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
import zipfile

import pytest

from openpilot.selfdrive.debug.device_log_download import (
  LogSelection,
  MAX_LOG_RANGE_SECONDS,
  available_log_range,
  build_log_zip,
  download_filename,
  scan_log_segments,
  select_log_range,
)
from openpilot.selfdrive.debug import device_console


def make_segment(root: Path, name: str, start_ms: int, files: dict[str, bytes], duration_ms: int = 60_000) -> Path:
  segment = root / name
  segment.mkdir()
  for filename, content in files.items():
    path = segment / filename
    path.write_bytes(content)
    end_ns = (start_ms + duration_ms) * 1_000_000
    os.utime(path, ns=(end_ns, end_ns))
  os.utime(segment, ns=(start_ms * 1_000_000, start_ms * 1_000_000))
  return segment


def test_scans_only_structured_route_logs_and_uses_segment_interval(tmp_path):
  segment = make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"qlog",
    "rlog.zst": b"rlog",
    "fcamera.hevc": b"video",
  })
  (tmp_path / "not-a-route").mkdir()
  (segment / "linked-rlog.zst").symlink_to(segment / "rlog.zst")
  os.utime(segment, ns=(1_000_000 * 1_000_000, 1_000_000 * 1_000_000))

  segments = scan_log_segments(tmp_path)

  assert len(segments) == 1
  assert segments[0].start_ms == 1_000_000
  assert segments[0].end_ms == 1_060_000
  assert [file.name for file in segments[0].files] == ["qlog.zst", "rlog.zst"]


def test_selects_every_segment_overlapping_the_requested_time(tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {"qlog.zst": b"a"})
  make_segment(tmp_path, "00000001--123456789a--1", 1_060_000, {"rlog.zst": b"bb"})
  make_segment(tmp_path, "00000002--abcdef1234--0", 2_000_000, {"qlog.zst": b"c"})

  selection = select_log_range(1_030_000, 1_080_000, tmp_path)

  assert [segment.name for segment in selection.segments] == [
    "00000001--123456789a--0", "00000001--123456789a--1",
  ]
  assert selection.total_bytes == 3


def test_rejects_invalid_or_excessive_ranges(tmp_path):
  with pytest.raises(ValueError, match="早于"):
    select_log_range(1000, 1000, tmp_path)
  with pytest.raises(ValueError, match="最多选择"):
    select_log_range(1000, 1000 + (MAX_LOG_RANGE_SECONDS + 1) * 1000, tmp_path)


def test_status_and_empty_selection_are_explicit(tmp_path):
  empty = available_log_range(tmp_path)
  assert empty["available"] is False
  assert empty["start_ms"] is None

  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {"qlog.zst": b"a"})
  status = available_log_range(tmp_path)
  assert status["available"] is True
  assert status["segment_count"] == 1
  assert select_log_range(2_000_000, 2_001_000, tmp_path).files == ()


def test_builds_browser_friendly_zip_with_manifest_and_no_video(tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"qlog-data",
    "rlog.zst": b"rlog-data",
    "ecamera.hevc": b"video-data",
  })
  selection = select_log_range(990_000, 1_070_000, tmp_path)
  archive_path = tmp_path / "download.zip"
  archive_path.write_bytes(build_log_zip(selection))

  with zipfile.ZipFile(archive_path) as archive:
    assert archive.namelist() == [
      "manifest.json",
      "00000001--123456789a--0/qlog.zst",
      "00000001--123456789a--0/rlog.zst",
    ]
    manifest = json.loads(archive.read("manifest.json"))
    assert manifest["segment_count"] == 1
    assert manifest["file_count"] == 2
    assert archive.read("00000001--123456789a--0/rlog.zst") == b"rlog-data"

  assert download_filename(selection).startswith("openpilot-logs-")
  assert download_filename(selection).endswith(".zip")


@pytest.fixture
def console_server(monkeypatch):
  monkeypatch.setattr(device_console.DeviceConsoleHandler, "_authorize_api", lambda self: True)
  server = ThreadingHTTPServer(("127.0.0.1", 0), device_console.DeviceConsoleHandler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    yield server
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_console_page_exposes_time_range_log_download(monkeypatch):
  monkeypatch.setattr(device_console, "driving_status_enabled", lambda: True)
  page = device_console.render_page().decode()

  assert "logs-tab" in page
  assert 'type="datetime-local"' in page
  assert "/api/logs/status" in page
  assert "/api/logs/preview" in page
  assert "/api/logs/download" in page
  assert "仅包含 rlog/qlog，不包含视频" in page


def test_console_log_status_and_preview_routes(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "available_log_range", lambda: {
    "available": True, "start_ms": 1_000_000, "end_ms": 1_060_000, "segment_count": 1,
  })
  monkeypatch.setattr(device_console, "console_status", lambda: {"onroad": False})
  monkeypatch.setattr(device_console, "select_log_range", lambda start, end: LogSelection(start, end, ()))

  base = f"http://127.0.0.1:{console_server.server_port}"
  with urllib.request.urlopen(base + "/api/logs/status", timeout=2) as response:
    status = json.loads(response.read())
  with urllib.request.urlopen(base + "/api/logs/preview?start_ms=1000&end_ms=2000", timeout=2) as response:
    preview = json.loads(response.read())

  assert status["available"] is True
  assert status["onroad"] is False
  assert status["structured_logs_only"] is True
  assert preview["start_ms"] == 1000
  assert preview["end_ms"] == 2000
  assert preview["file_count"] == 0


def test_console_streams_selected_logs_as_zip(monkeypatch, console_server, tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"qlog-data", "rlog.zst": b"rlog-data", "fcamera.hevc": b"video",
  })
  selection = select_log_range(990_000, 1_070_000, tmp_path)
  monkeypatch.setattr(device_console, "select_log_range", lambda start, end: selection)
  monkeypatch.setattr(device_console, "require_offroad", lambda: None)

  url = f"http://127.0.0.1:{console_server.server_port}/api/logs/download?start_ms=990000&end_ms=1070000"
  with urllib.request.urlopen(url, timeout=5) as response:
    body = response.read()
    disposition = response.headers["Content-Disposition"]

  archive_path = tmp_path / "browser.zip"
  archive_path.write_bytes(body)
  with zipfile.ZipFile(archive_path) as archive:
    assert "manifest.json" in archive.namelist()
    assert not any(name.endswith(".hevc") for name in archive.namelist())
  assert disposition.startswith('attachment; filename="openpilot-logs-')


def test_console_blocks_log_download_while_onroad(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "require_offroad",
                      lambda: (_ for _ in ()).throw(PermissionError("行驶中禁止执行该操作")))
  url = f"http://127.0.0.1:{console_server.server_port}/api/logs/download?start_ms=1000&end_ms=2000"

  with pytest.raises(urllib.error.HTTPError) as exc_info:
    urllib.request.urlopen(url, timeout=2)

  assert exc_info.value.code == 403
  assert "行驶中禁止" in json.loads(exc_info.value.read())["message"]
