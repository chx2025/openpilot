from openpilot.selfdrive.ui.egpu_status import build_egpu_status


def test_egpu_status_explains_fallback_reason():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=False,
    model_alive=True, model_big=False, telemetry_valid=False, usb_speed_mbps=5000,
  )
  assert status.visible
  assert not status.healthy
  assert "回退小模型" in status.headline
  assert "5000" in status.details[0]


def test_egpu_status_reports_live_model_and_gpu_metrics():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True,
    usb_speed_mbps=5000, model_fps=19.8, power_w=72.0,
    temp_c=61.0, memory_temp_c=70.0, memory_used_mb=6144,
    memory_total_mb=8192, gpu_usage_percent=88, gpu_clock_mhz=2200,
    fan_speed_rpm=1450,
  )
  assert status.healthy
  assert "19.8 FPS" in status.headline
  assert "6.0/8.0 GB" in status.details[1]
  assert "88%" in status.details[2]
  assert "2200 MHz" in status.details[2]


def test_absent_egpu_has_no_panel():
  status = build_egpu_status(
    connected=False, compiled=False, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert not status.visible


def test_compiled_egpu_build_explains_disconnected_hardware():
  status = build_egpu_status(
    connected=False, compiled=True, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert status.visible
  assert "未连接" in status.headline
