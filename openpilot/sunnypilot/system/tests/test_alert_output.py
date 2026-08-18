import os
from unittest.mock import Mock, call

import pytest

from openpilot.sunnypilot.system.alert_output import BEEP_GAP_SECONDS, BEEP_PULSE_SECONDS, Beepd
from openpilot.sunnypilot.hardware.profile import HardwareProfile


@pytest.fixture
def beepd():
  beep = Beepd.__new__(Beepd)
  beep.mads_enabled = None
  beep.dispatch_beep = Mock()
  return beep


def test_mads_initial_state_is_silent(beepd):
  beepd.update_mads(True)

  beepd.dispatch_beep.assert_not_called()


def test_mads_enable_and_disable_have_distinct_beeps(beepd):
  beepd.update_mads(False)
  beepd.update_mads(True)
  beepd.update_mads(True)
  beepd.update_mads(False)

  assert beepd.dispatch_beep.call_args_list == [call(beepd.engage), call(beepd.disengage)]


def test_mads_beeps_use_very_short_pulses(monkeypatch):
  assert BEEP_PULSE_SECONDS == pytest.approx(0.010)
  beep = Beepd.__new__(Beepd)
  beep._beep = Mock()
  sleep = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.time.sleep", sleep)

  beep.engage()
  assert beep._beep.call_args_list == [call(True), call(False)]
  sleep.assert_called_once_with(BEEP_PULSE_SECONDS)

  beep._beep.reset_mock()
  sleep.reset_mock()
  beep.disengage()
  assert beep._beep.call_args_list == [call(True), call(False), call(True), call(False)]
  assert sleep.call_args_list == [call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS), call(BEEP_PULSE_SECONDS)]

  beep._beep.reset_mock()
  sleep.reset_mock()
  beep.warning()
  assert beep._beep.call_args_list == [call(True), call(False)] * 3
  assert sleep.call_args_list == [
    call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS),
    call(BEEP_PULSE_SECONDS), call(BEEP_GAP_SECONDS),
    call(BEEP_PULSE_SECONDS),
  ]


def test_gpio_edges_use_persistent_fd_without_subprocess(monkeypatch):
  beep = Beepd.__new__(Beepd)
  beep.gpio_fd = 42
  run = Mock()
  seek = Mock()
  write = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.subprocess.run", run)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.os.lseek", seek)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.os.write", write)

  beep._beep(True)
  beep._beep(False)

  run.assert_not_called()
  assert seek.call_args_list == [call(42, 0, os.SEEK_SET), call(42, 0, os.SEEK_SET)]
  assert write.call_args_list == [call(42, b"1"), call(42, b"0")]


def test_standard_profile_never_probes_gpio42(monkeypatch):
  run = Mock()
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.get_hardware_profile",
                      lambda: HardwareProfile.STANDARD)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.subprocess.run", run)
  monkeypatch.setattr("openpilot.sunnypilot.system.alert_output.threading.Thread.start", lambda _: None)

  beep = Beepd()

  assert beep.gpio_fd is None
  run.assert_not_called()
