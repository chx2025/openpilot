from unittest.mock import Mock

import openpilot.system.manager.process_config as process_config


def test_route_video_recording_is_off_by_default_and_follows_one_explicit_toggle():
  params = Mock()
  params.get_bool.side_effect = lambda key: {"RecordRoadVideo": False}[key]
  assert not process_config.record_route_video(True, params, Mock())

  params.get_bool.side_effect = lambda key: {"RecordRoadVideo": True}[key]
  assert process_config.record_route_video(True, params, Mock())
  assert not process_config.record_route_video(False, params, Mock())
