from unittest.mock import Mock

import openpilot.system.manager.process_config as process_config
from openpilot.sunnypilot.hardware.profile import HardwareProfile


def test_c3xl_never_runs_route_video_encoder_even_with_stale_true_param(monkeypatch):
  params = Mock()
  params.get_bool.return_value = True
  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.C3XL)

  assert not process_config.record_route_video(True, params, Mock())


def test_standard_profile_still_follows_explicit_recording_param(monkeypatch):
  params = Mock()
  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.STANDARD)

  params.get_bool.side_effect = lambda key: {"RecordRoadVideo": True}[key]
  assert process_config.record_route_video(True, params, Mock())
  assert not process_config.record_route_video(False, params, Mock())


def test_local_diagnostics_runs_onroad_only_for_c3xl(monkeypatch):
  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.C3XL)
  assert process_config.c3xl_local_diagnostics(True, Mock(), Mock())
  assert not process_config.c3xl_local_diagnostics(False, Mock(), Mock())

  monkeypatch.setattr(process_config, "get_hardware_profile", lambda: HardwareProfile.STANDARD)
  assert not process_config.c3xl_local_diagnostics(True, Mock(), Mock())
