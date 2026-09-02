import json
import os
from pathlib import Path
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
import zipfile

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.debug.device_log_download import (
  DiagnosticFile,
  LogDeletion,
  LogSelection,
  MAX_LOG_RANGE_SECONDS,
  available_log_range,
  build_log_zip,
  delete_log_selection,
  download_filename,
  scan_log_segments,
  select_log_range,
)
from openpilot.selfdrive.debug import device_console
from openpilot.selfdrive.debug.local_diagnostics import LocalDiagnosticWriter, scan_local_diagnostics


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
  assert [file.name for file in segments[0].files] == ["qlog.zst"]
  assert [file.name for file in scan_log_segments(tmp_path, include_rlog=True)[0].files] == ["qlog.zst", "rlog.zst"]


def test_selects_every_segment_overlapping_the_requested_time(tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {"qlog.zst": b"a"})
  make_segment(tmp_path, "00000001--123456789a--1", 1_060_000, {"rlog.zst": b"bb"})
  make_segment(tmp_path, "00000002--abcdef1234--0", 2_000_000, {"qlog.zst": b"c"})

  selection = select_log_range(1_030_000, 1_080_000, tmp_path)

  assert [segment.name for segment in selection.segments] == ["00000001--123456789a--0"]
  assert selection.total_bytes == 1


def test_qlog_only_selection_excludes_rlog_and_video(tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"small",
    "rlog.zst": b"large-rlog",
    "fcamera.hevc": b"video",
  })

  selection = select_log_range(990_000, 1_070_000, tmp_path, include_rlog=False)

  assert [file.name for file in selection.files] == ["qlog.zst"]
  assert selection.total_bytes == len(b"small")


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
  diagnostic_root = tmp_path / "spdiagnostics"
  writer = LocalDiagnosticWriter(diagnostic_root)
  writer.write(messaging.new_message("trafficRadarState").to_bytes(), wall_time_ms=1_000_000)
  writer.close(wall_time_ms=1_060_000)
  local_diagnostic = scan_local_diagnostics(diagnostic_root)[0]
  selection = select_log_range(990_000, 1_070_000, tmp_path, diagnostic_root=diagnostic_root)
  diagnostics = (
    DiagnosticFile("system/journal-warning-current-boot.log", b"journal error"),
    DiagnosticFile("system/launch_log.txt", b"launch error"),
  )
  archive_path = tmp_path / "download.zip"
  archive_path.write_bytes(build_log_zip(selection, diagnostics=diagnostics))

  with zipfile.ZipFile(archive_path) as archive:
    assert archive.namelist() == [
      "manifest.json",
      "00000001--123456789a--0/qlog.zst",
      local_diagnostic.archive_name,
      "system/journal-warning-current-boot.log",
      "system/launch_log.txt",
    ]
    manifest = json.loads(archive.read("manifest.json"))
    assert manifest["segment_count"] == 1
    assert manifest["file_count"] == 2
    assert manifest["route_file_count"] == 1
    assert manifest["local_diagnostic_count"] == 1
    assert manifest["local_diagnostics"][0]["path"] == local_diagnostic.archive_name
    assert manifest["system_diagnostics"] == [
      {"path": "system/journal-warning-current-boot.log", "size": len(b"journal error")},
      {"path": "system/launch_log.txt", "size": len(b"launch error")},
    ]
    assert archive.read("system/journal-warning-current-boot.log") == b"journal error"
    assert not any(name.endswith((".hevc", ".ts")) for name in archive.namelist())

  assert download_filename(selection).startswith("openpilot-logs-")
  assert download_filename(selection).endswith(".zip")


def test_deletes_only_selected_structured_logs_and_preserves_video(tmp_path):
  segment = make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"qlog-data",
    "rlog.zst": b"rlog-data",
    "fcamera.hevc": b"video-data",
  })
  selection = select_log_range(990_000, 1_070_000, tmp_path, include_rlog=True)

  result = delete_log_selection(selection, tmp_path)

  assert result.file_count == 2
  assert result.total_bytes == len(b"qlog-data") + len(b"rlog-data")
  assert result.segment_count == 1
  assert result.skipped_files == ()
  assert not (segment / "qlog.zst").exists()
  assert not (segment / "rlog.zst").exists()
  assert (segment / "fcamera.hevc").read_bytes() == b"video-data"
  assert segment.is_dir()


def test_delete_skips_log_replaced_by_symlink_after_selection(tmp_path):
  segment = make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {"qlog.zst": b"selected"})
  selection = select_log_range(990_000, 1_070_000, tmp_path)
  outside = tmp_path / "outside.txt"
  outside.write_bytes(b"keep")
  (segment / "qlog.zst").unlink()
  (segment / "qlog.zst").symlink_to(outside)

  result = delete_log_selection(selection, tmp_path)

  assert result.file_count == 0
  assert result.skipped_files == ("00000001--123456789a--0/qlog.zst",)
  assert outside.read_bytes() == b"keep"
  assert (segment / "qlog.zst").is_symlink()


def test_delete_removes_selected_local_diagnostics_but_not_unrelated_files(tmp_path):
  diagnostic_root = tmp_path / "spdiagnostics"
  writer = LocalDiagnosticWriter(diagnostic_root)
  writer.write(messaging.new_message("chestnutState").to_bytes(), wall_time_ms=1_000_000)
  writer.close(wall_time_ms=1_060_000)
  unrelated = diagnostic_root / "notes.txt"
  unrelated.write_text("keep")
  selection = select_log_range(990_000, 1_070_000, tmp_path, diagnostic_root=diagnostic_root)

  result = delete_log_selection(selection, tmp_path, diagnostic_root=diagnostic_root)

  assert result.file_count == 1
  assert not scan_local_diagnostics(diagnostic_root)
  assert unrelated.read_text() == "keep"


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
  assert "/api/logs/delete" in page
  assert "清理所选日志" in page
  assert "log-include-rlog" not in page
  assert "系统错误日志" in page
  assert "遗留 rlog" in page
  assert "不包含 rlog 或视频" in page


def test_console_log_status_and_preview_routes(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "available_log_range", lambda: {
    "available": True, "start_ms": 1_000_000, "end_ms": 1_060_000, "segment_count": 1,
    "local_diagnostic_count": 0,
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
  assert status["export_excludes_rlog"] is True
  assert status["export_excludes_video"] is True
  assert preview["start_ms"] == 1000
  assert preview["end_ms"] == 2000
  assert preview["file_count"] == 0
  assert preview["includes_system_diagnostics"] is True
  assert preview["max_system_diagnostic_bytes"] == device_console.MAX_SYSTEM_DIAGNOSTIC_BYTES


def test_console_offers_system_diagnostics_when_no_route_exists(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "available_log_range", lambda: {
    "available": False, "start_ms": None, "end_ms": None, "segment_count": 0,
    "local_diagnostic_count": 0,
  })
  monkeypatch.setattr(device_console, "console_status", lambda: {"onroad": False})

  base = f"http://127.0.0.1:{console_server.server_port}"
  with urllib.request.urlopen(base + "/api/logs/status", timeout=2) as response:
    status = json.loads(response.read())

  assert status["available"] is True
  assert status["system_diagnostics_available"] is True
  assert status["start_ms"] < status["end_ms"]


def test_console_streams_selected_logs_as_zip(monkeypatch, console_server, tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {
    "qlog.zst": b"qlog-data", "rlog.zst": b"rlog-data", "fcamera.hevc": b"video",
  })
  selection = select_log_range(990_000, 1_070_000, tmp_path)
  monkeypatch.setattr(device_console, "select_log_range", lambda start, end: selection)
  monkeypatch.setattr(device_console, "collect_system_diagnostics",
                      lambda: (DiagnosticFile("system/error.log", b"system error"),))
  monkeypatch.setattr(device_console, "require_offroad", lambda: None)

  url = f"http://127.0.0.1:{console_server.server_port}/api/logs/download?start_ms=990000&end_ms=1070000"
  with urllib.request.urlopen(url, timeout=5) as response:
    body = response.read()
    disposition = response.headers["Content-Disposition"]

  archive_path = tmp_path / "browser.zip"
  archive_path.write_bytes(body)
  with zipfile.ZipFile(archive_path) as archive:
    assert "manifest.json" in archive.namelist()
    assert "system/error.log" in archive.namelist()
    assert not any(name.endswith((".hevc", "/rlog.zst")) for name in archive.namelist())
  assert disposition.startswith('attachment; filename="openpilot-logs-')


def test_console_exports_onroad_block_diagnostics_without_route_logs(monkeypatch, console_server, tmp_path):
  selection = LogSelection(1_000, 2_000, ())
  monkeypatch.setattr(device_console, "select_log_range", lambda start, end: selection)
  monkeypatch.setattr(device_console, "collect_system_diagnostics", lambda: (
    DiagnosticFile("system/onroad-block/onroad-block.jsonl", b'{"event":"onroad_blocked"}\n'),
  ))
  monkeypatch.setattr(device_console, "require_offroad", lambda: None)

  url = f"http://127.0.0.1:{console_server.server_port}/api/logs/download?start_ms=1000&end_ms=2000"
  with urllib.request.urlopen(url, timeout=5) as response:
    body = response.read()

  archive_path = tmp_path / "onroad-block.zip"
  archive_path.write_bytes(body)
  with zipfile.ZipFile(archive_path) as archive:
    assert archive.read("system/onroad-block/onroad-block.jsonl") == b'{"event":"onroad_blocked"}\n'
    assert not any(name.endswith((".hevc", "/rlog.zst")) for name in archive.namelist())


def test_console_deletes_confirmed_selected_logs(monkeypatch, console_server, tmp_path):
  make_segment(tmp_path, "00000001--123456789a--0", 1_000_000, {"qlog.zst": b"selected"})
  selection = select_log_range(990_000, 1_070_000, tmp_path)
  monkeypatch.setattr(device_console, "require_offroad", lambda: None)
  monkeypatch.setattr(device_console, "select_log_range", lambda start, end, **kwargs: selection)
  monkeypatch.setattr(device_console, "delete_log_selection",
                      lambda selected: LogDeletion(2, 4, 8192, ()))
  payload = json.dumps({"start_ms": 1000, "end_ms": 2000, "confirm": True}).encode()
  request = urllib.request.Request(
    f"http://127.0.0.1:{console_server.server_port}/api/logs/delete",
    data=payload,
    headers={"Content-Type": "application/json"},
    method="POST",
  )

  with urllib.request.urlopen(request, timeout=2) as response:
    result = json.loads(response.read())

  assert result == {
    "ok": True,
    "segment_count": 2,
    "file_count": 4,
    "total_bytes": 8192,
    "skipped_files": [],
  }


def test_console_requires_explicit_log_delete_confirmation(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "require_offroad", lambda: None)
  payload = json.dumps({"start_ms": 1000, "end_ms": 2000}).encode()
  request = urllib.request.Request(
    f"http://127.0.0.1:{console_server.server_port}/api/logs/delete",
    data=payload,
    headers={"Content-Type": "application/json"},
    method="POST",
  )

  with pytest.raises(urllib.error.HTTPError) as exc_info:
    urllib.request.urlopen(request, timeout=2)

  assert exc_info.value.code == 400
  assert "明确确认" in json.loads(exc_info.value.read())["message"]


def test_console_blocks_log_delete_while_onroad(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "require_offroad",
                      lambda: (_ for _ in ()).throw(PermissionError("行驶中禁止执行该操作")))
  payload = json.dumps({"start_ms": 1000, "end_ms": 2000, "confirm": True}).encode()
  request = urllib.request.Request(
    f"http://127.0.0.1:{console_server.server_port}/api/logs/delete",
    data=payload,
    headers={"Content-Type": "application/json"},
    method="POST",
  )

  with pytest.raises(urllib.error.HTTPError) as exc_info:
    urllib.request.urlopen(request, timeout=2)

  assert exc_info.value.code == 403
  assert "行驶中禁止" in json.loads(exc_info.value.read())["message"]


def test_console_blocks_log_download_while_onroad(monkeypatch, console_server):
  monkeypatch.setattr(device_console, "require_offroad",
                      lambda: (_ for _ in ()).throw(PermissionError("行驶中禁止执行该操作")))
  url = f"http://127.0.0.1:{console_server.server_port}/api/logs/download?start_ms=1000&end_ms=2000"

  with pytest.raises(urllib.error.HTTPError) as exc_info:
    urllib.request.urlopen(url, timeout=2)

  assert exc_info.value.code == 403
  assert "行驶中禁止" in json.loads(exc_info.value.read())["message"]
