from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.traffic_control import planner_session_is_active


class FakeSubMaster:
  def __init__(self, *, seen=True, alive=True, valid=True, started=False):
    self.seen = {"deviceState": seen}
    self.alive = {"deviceState": alive}
    self.valid = {"deviceState": valid}
    self.device_state = SimpleNamespace(started=started)

  def __getitem__(self, name):
    assert name == "deviceState"
    return self.device_state


def test_planner_session_follows_raw_device_started_state():
  assert not planner_session_is_active(FakeSubMaster(started=False))
  assert planner_session_is_active(FakeSubMaster(started=True))


def test_unknown_device_state_fails_closed():
  assert planner_session_is_active(FakeSubMaster(seen=False))
  assert planner_session_is_active(FakeSubMaster(alive=False))
  assert planner_session_is_active(FakeSubMaster(valid=False))
