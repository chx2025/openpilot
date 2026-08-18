from openpilot.sunnypilot.selfdrive.car.tesla.control_profile import INITIALIZATION_KEYS, initialization_snapshot


class FakeParams:
  def __init__(self):
    self.requested = []

  def get(self, key, block=False, encoding=None, return_default=False):
    self.requested.append((key, return_default))
    return f"value:{key}"


def test_initialization_snapshot_is_complete_and_ordered():
  params = FakeParams()

  snapshot = initialization_snapshot(params)

  assert [next(iter(item)) for item in snapshot] == list(INITIALIZATION_KEYS)
  assert params.requested == [(key, True) for key in INITIALIZATION_KEYS]
  assert len(INITIALIZATION_KEYS) == len(set(INITIALIZATION_KEYS))
