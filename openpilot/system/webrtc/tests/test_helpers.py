from pathlib import Path

import pytest
import requests

from openpilot.system.webrtc import helpers
from openpilot.system.webrtc.helpers import StreamRequestBody


def test_stream_connection_error_tells_user_to_turn_ignition_off(mocker):
  mocker.patch.object(requests, "post", side_effect=requests.ConnectionError)
  body = StreamRequestBody("offer", ["wideRoad"], True)

  with pytest.raises(Exception, match="turn car ignition off to use livestreaming"):
    helpers.post_stream_request(body)


def test_stream_timeout_identifies_device_response(mocker):
  mocker.patch.object(requests, "post", side_effect=requests.ConnectTimeout)
  body = StreamRequestBody("offer", ["wideRoad"], True)

  with pytest.raises(Exception, match="device took too long to respond"):
    helpers.post_stream_request(body)


def test_webrtcd_wait_timeout_uses_livestream_name(mocker):
  mocker.patch.object(requests, "get", side_effect=requests.ConnectionError)
  mocker.patch.object(helpers.time, "sleep")

  with pytest.raises(TimeoutError, match="livestreaming service did not initialize in time"):
    helpers.wait_for_webrtcd(max_retries=1)


def test_webrtcd_does_not_upload_ice_candidates_or_cloud_logs():
  source = (Path(__file__).parents[1] / "webrtcd.py").read_text()

  assert "_ice_candidates" not in source
  assert "webrtcd.session.ice_candidates" not in source
  assert "cloudlog" not in source
