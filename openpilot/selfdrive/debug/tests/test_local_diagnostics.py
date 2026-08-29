from openpilot.cereal import messaging
from openpilot.selfdrive.debug.local_diagnostics import (
  LOCAL_DIAGNOSTIC_SERVICES,
  LocalDiagnosticWriter,
  scan_local_diagnostics,
)
from openpilot.tools.lib.logreader import LogReader


def _event(service: str) -> bytes:
  return messaging.new_message(service).to_bytes()


def test_diagnostic_scope_covers_local_features_without_raw_can_or_model_tensors():
  required = {
    "chestnutState",       # eGPU state
    "carStateSP",          # Tesla road/traffic state
    "carControlSP",        # Tesla/SP control output
    "selfdriveStateSP",    # MADS and longitudinal ownership
    "longitudinalPlanSP",  # selected planner and Traffic arbitration
    "trafficRadarState",   # virtual-radar Traffic state
  }

  assert required <= LOCAL_DIAGNOSTIC_SERVICES.keys()
  assert "can" not in LOCAL_DIAGNOSTIC_SERVICES
  assert "modelV2" not in LOCAL_DIAGNOSTIC_SERVICES
  assert "modelDataV2SP" not in LOCAL_DIAGNOSTIC_SERVICES


def test_writer_rotates_and_keeps_a_bounded_number_of_parseable_files(tmp_path):
  writer = LocalDiagnosticWriter(tmp_path, max_uncompressed_bytes=1, max_files=2,
                                 max_total_bytes=1024 * 1024)
  writer.write(_event("chestnutState"), wall_time_ms=1_000)
  writer.write(_event("carStateSP"), wall_time_ms=2_000)
  writer.write(_event("longitudinalPlanSP"), wall_time_ms=3_000)
  writer.close(wall_time_ms=4_000)

  files = scan_local_diagnostics(tmp_path)
  assert len(files) == 2
  assert not tuple(tmp_path.glob("*.partial"))
  assert [message.which() for file in files for message in LogReader(str(file.path))] == [
    "carStateSP", "longitudinalPlanSP",
  ]


def test_scan_rejects_symlinks_partial_and_unrelated_files(tmp_path):
  writer = LocalDiagnosticWriter(tmp_path)
  writer.write(_event("trafficRadarState"), wall_time_ms=10_000)
  writer.close(wall_time_ms=11_000)
  valid = next(tmp_path.glob("*.zst"))
  (tmp_path / "spdiag-0000000010000-0000000011000-999999.zst").symlink_to(valid)
  (tmp_path / "spdiag-0000000010000.partial").write_bytes(b"unfinished")
  (tmp_path / "notes.txt").write_text("not a diagnostic stream")

  files = scan_local_diagnostics(tmp_path)

  assert [file.path for file in files] == [valid.resolve()]
  assert [message.which() for message in LogReader(str(valid))] == ["trafficRadarState"]
