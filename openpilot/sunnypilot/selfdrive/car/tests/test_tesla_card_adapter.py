from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.car.tesla.card_adapter import CONTEXT_STALE_S, TeslaCardAdapter, speed_limit_context


class FakeState:
  def __init__(self):
    self.templates = []
    self.longitudinal = []
    self.speed_limit = []

  def update_speed_button_template(self, data, mono_time):
    self.templates.append((data, mono_time))

  def update_longitudinal_context(self, *context):
    self.longitudinal.append(context)

  def update_speed_limit_target(self, *context):
    self.speed_limit.append(context)


class FakeSubMaster:
  def __init__(self, now=10.0):
    self.data = {
      "longitudinalPlanSP": SimpleNamespace(
        longitudinalPlanSource=SimpleNamespace(raw=3),
        speedLimit=SimpleNamespace(
          assist=SimpleNamespace(enabled=True),
          resolver=SimpleNamespace(speedLimitValid=True, speedLimitLastValid=False, speedLimitFinalLast=22.0),
        ),
      ),
      "carControl": SimpleNamespace(
        leftBlinker=True, rightBlinker=False, latActive=True, longActive=True,
        actuators=SimpleNamespace(accel=0.5),
      ),
      "selfdriveStateSP": SimpleNamespace(mads=SimpleNamespace(active=False)),
    }
    self.recv_time = dict.fromkeys(self.data, now)
    self.seen = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.updated = dict.fromkeys(self.data, True)

  def __getitem__(self, name):
    return self.data[name]


def test_adapter_observes_only_vehicle_speed_templates():
  state = FakeState()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.observe_can([(100, [(0x3C2, b"valid", 1), (0x3C2, b"wrong-bus", 0), (0x123, b"other", 1)])])

  assert state.templates == [(b"valid", 100)]


def test_adapter_supplies_fresh_context():
  state = FakeState()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.update_context(10.0)

  assert state.longitudinal[0][0] == 3
  assert state.longitudinal[0][2] is True
  assert state.speed_limit == [(22.0, True)]


def test_adapter_invalidates_stale_context():
  state = FakeState()
  sm = FakeSubMaster()
  adapter = TeslaCardAdapter("tesla", SimpleNamespace(CS=state), sm)

  adapter.update_context(10.0 + CONTEXT_STALE_S + 0.01)

  assert state.longitudinal[0][2] is False
  assert state.speed_limit == [(0.0, False)]


def test_configured_assist_keeps_resolved_target_while_runtime_state_is_inactive():
  sm = FakeSubMaster()
  sm.data["longitudinalPlanSP"].speedLimit.assist.enabled = False

  assert speed_limit_context(sm, 10.0, assist_configured=True) == (22.0, True)


def test_non_assist_mode_never_exposes_automatic_speed_target():
  sm = FakeSubMaster()

  assert speed_limit_context(sm, 10.0, assist_configured=False) == (0.0, False)


def test_non_tesla_adapter_is_inert():
  state = FakeState()
  adapter = TeslaCardAdapter("toyota", SimpleNamespace(CS=state), FakeSubMaster())

  adapter.observe_can([(100, [(0x3C2, b"ignored", 1)])])
  adapter.update_context(10.0)

  assert not state.templates
  assert not state.longitudinal
  assert not state.speed_limit
