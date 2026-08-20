import openpilot.cereal.messaging as messaging
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc as OfficialMpc,
  LongitudinalPlanSource,
  PARAM_DIM as OFFICIAL_PARAM_DIM,
  T_IDXS,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.legacy_mpc.long_mpc import (
  PARAM_DIM as LEGACY_PARAM_DIM,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.experimental.long_mpc import LongitudinalMpc as ExperimentalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tn_no_dec.long_mpc import LongitudinalMpc as TnMpc
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.tuning import LongitudinalTuning
from openpilot.sunnypilot.selfdrive.traffic_control.target import TrafficMpcTarget
from openpilot.sunnypilot.selfdrive.traffic_control import trafficcontrold


def test_independent_traffic_radar_service_uses_the_existing_twenty_hz_slot():
  message = messaging.new_message("trafficRadarState")
  target = message.trafficRadarState
  target.targetPresent = True
  target.controlAllowed = True
  target.distanceToStopPoint = 42.0

  assert message.which() == "trafficRadarState"
  assert target.targetPresent and target.controlAllowed
  assert target.distanceToStopPoint == 42.0
  assert SERVICE_LIST["trafficRadarState"].should_log
  assert SERVICE_LIST["trafficRadarState"].frequency == 20.0
  assert SERVICE_LIST["trafficRadarState"].decimation == 5


def test_trafficcontrold_publishes_only_the_independent_traffic_radar_service(monkeypatch):
  published = {}

  class FakeSubMaster:
    updated = {"modelV2": True}
    logMonoTime = {"modelV2": 123}

    def __init__(self, services, **kwargs):
      published["subscriptions"] = tuple(services)
      published["submaster_options"] = kwargs
      self.update_count = 0

    def update(self):
      if self.update_count:
        raise StopIteration
      self.update_count += 1

  class FakePubMaster:
    def __init__(self, services):
      published["services"] = tuple(services)

    def send(self, service, message):
      published["sent"] = (service, message)

  message = object()
  source = SimpleNamespace(update=lambda sm, now_ns: message)
  monkeypatch.setattr(trafficcontrold, "config_realtime_process", lambda *args: None)
  monkeypatch.setattr(trafficcontrold, "Params", lambda: object())
  monkeypatch.setattr(trafficcontrold, "build_source", lambda params: source)
  monkeypatch.setattr(trafficcontrold.messaging, "SubMaster", FakeSubMaster)
  monkeypatch.setattr(trafficcontrold.messaging, "PubMaster", FakePubMaster)

  with pytest.raises(StopIteration):
    trafficcontrold.main()

  assert published["services"] == ("trafficRadarState",)
  assert published["sent"] == ("trafficRadarState", message)
  assert {"radarState", "modelV2"} <= set(published["subscriptions"])
  assert not ({"radarState", "modelV2", "can", "sendcan"} & set(published["services"]))


@pytest.mark.parametrize(
  "mpc_type,param_dim,is_legacy",
  (
    (OfficialMpc, OFFICIAL_PARAM_DIM, False),
    (ExperimentalMpc, LEGACY_PARAM_DIM, True),
    (TnMpc, LEGACY_PARAM_DIM, True),
  ),
  ids=("official", "experimental", "tn-no-dec"),
)
def test_each_backend_consumes_traffic_as_an_independent_candidate_without_touching_physical_leads(
  mpc_type, param_dim, is_legacy,
):
  mpc = mpc_type.__new__(mpc_type)
  mpc.x0 = np.array([0.0, 10.0, 0.0])
  mpc.params = np.zeros((len(T_IDXS), param_dim))
  mpc.a_prev = np.zeros(len(T_IDXS))
  mpc.x_sol = np.zeros((len(T_IDXS), 3))
  mpc.yref = np.zeros((len(T_IDXS), 6))
  mpc.runtime_tuning = LongitudinalTuning()
  mpc.crash_cnt = 0
  mpc.solver = SimpleNamespace(set=lambda *args, **kwargs: None)
  mpc.run = lambda: None
  if is_legacy:
    mpc._apply_legacy_backend_params = lambda: None
    mpc._legacy_cruise_accel_max = lambda stock_accel_max: stock_accel_max
  else:
    mpc._apply_backend_params = lambda: None
  physical_lead_one = SimpleNamespace(present=False, modelProb=0.0)
  physical_lead_two = SimpleNamespace(present=False, modelProb=0.0)
  radar_state = SimpleNamespace(leadOne=physical_lead_one, leadTwo=physical_lead_two)
  far_lead = np.column_stack((np.full(len(T_IDXS), 200.0), np.full(len(T_IDXS), 10.0)))
  mpc.process_lead = lambda _lead: far_lead

  mpc.set_traffic_target(TrafficMpcTarget(event_id=7, distance_to_stop_point=30.0, should_stop=True))
  if is_legacy:
    mpc.update(radar_state, v_cruise=30.0)
  else:
    mpc.update(radar_state)

  assert mpc.source == LongitudinalPlanSource.lead2
  assert np.allclose(mpc.params[:, 2], 36.0)
  assert radar_state.leadOne is physical_lead_one
  assert radar_state.leadTwo is physical_lead_two
  assert not radar_state.leadOne.present and not radar_state.leadTwo.present
