from opendbc.can import CANPacker

from openpilot.sunnypilot.selfdrive.traffic_control.tesla_observer import TeslaTrafficControlObserver


def _frame(values, bus=2):
  return CANPacker("tesla_modely_hw4_perception").make_can_msg("APP_trafficControl", bus, values)


def test_observer_accepts_party_traffic_light_and_rejects_veh_collision():
  observer = TeslaTrafficControlObserver()
  values = {
    "APP_tcFeatureState": 3,
    "APP_tcStateMachine": 4,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 80,
    "APP_tcControlLightState": 1,
    "APP_tcVisionLight": 1,
  }

  address, data, _ = _frame(values)
  observer.update([(1_000_000_000, [(address, data, 1)])], 1_000_000_000)
  assert not observer.snapshot(1_000_000_000).available

  observer.update([(1_050_000_000, [(address, data, 2)])], 1_050_000_000)
  observation = observer.snapshot(1_050_000_000)
  assert observation.available
  assert observation.valid_for_control
  assert observation.source_bus == 2
  assert observation.distance == 80
  assert observation.light_state == 1
  assert observation.quality == 2


def test_observer_treats_255_as_no_target_and_supports_shortened_dlc():
  observer = TeslaTrafficControlObserver()
  address, data, _ = _frame({
    "APP_tcFeatureState": 0,
    "APP_tcStateMachine": 6,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 255,
    "APP_tcControlLightState": 1,
  })
  observer.update([(2_000_000_000, [(address, data[:6], 2)])], 2_000_000_000)
  observation = observer.snapshot(2_000_000_000)
  assert observation.available
  assert observation.dlc == 6
  assert observation.distance == 255
  assert not observation.valid_for_control


def test_observer_accepts_logged_red_aware_state_when_feature_flag_is_disabled():
  observer = TeslaTrafficControlObserver()
  address, data, _ = _frame({
    "APP_tcFeatureState": 0,
    "APP_tcStateMachine": 2,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 19,
    "APP_tcControlLightState": 1,
    "APP_tcUnavailableReason": 1,
    "APP_tcVisionLight": 1,
  })
  observer.update([(3_000_000_000, [(address, data, 2)])], 3_000_000_000)
  observation = observer.snapshot(3_000_000_000)
  assert observation.valid_for_control
  assert observation.feature_state == 0
  assert observation.control_type == 3
  assert observation.light_state == 1
  assert observation.distance == 19
  assert observation.state_machine == 2
  assert observation.quality == 2


def test_observer_accepts_logged_feature_zero_yellow_transition():
  observer = TeslaTrafficControlObserver()
  address, data, _ = _frame({
    "APP_tcFeatureState": 0,
    "APP_tcStateMachine": 6,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 49,
    "APP_tcControlLightState": 3,
    "APP_tcContinuationReason": 5,
    "APP_tcUnavailableReason": 1,
    "APP_tcVisionLight": 1,
  })

  observer.update([(3_050_000_000, [(address, data, 2)])], 3_050_000_000)
  observation = observer.snapshot(3_050_000_000)

  assert observation.valid_for_control
  assert observation.feature_state == 0
  assert observation.state_machine == 6
  assert observation.control_source == 3
  assert observation.light_state == 3
  assert observation.continuation_reason == 5
  assert observation.distance == 49


def test_observer_ignores_internal_availability_fields_for_color_distance_control():
  observer = TeslaTrafficControlObserver()
  address, data, _ = _frame({
    "APP_tcFeatureState": 0,
    "APP_tcStateMachine": 6,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 19,
    "APP_tcControlLightState": 1,
    "APP_tcUnavailableReason": 1,
    "APP_tcVisionLight": 1,
  })
  observer.update([(3_100_000_000, [(address, data, 2)])], 3_100_000_000)
  assert observer.snapshot(3_100_000_000).valid_for_control


def test_observer_uses_only_ap_party_and_never_falls_back_to_party():
  observer = TeslaTrafficControlObserver()
  values = {
    "APP_tcFeatureState": 3,
    "APP_tcStateMachine": 4,
    "APP_tcControlSource": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 60,
    "APP_tcControlLightState": 1,
    "APP_tcVisionLight": 1,
  }
  address, data, _ = _frame(values)
  observer.update([(5_000_000_000, [(address, data, 2)])], 5_000_000_000)

  party_values = dict(values, APP_tcControlDistance=20, APP_tcControlLightState=2)
  _, party_data, _ = _frame(party_values, bus=0)
  observer.update([(5_050_000_000, [(address, party_data, 0)])], 5_050_000_000)

  observation = observer.snapshot(5_050_000_000)
  assert observation.source_bus == 2
  assert observation.light_state == 1
  assert observation.distance == 60

  expired = observer.snapshot(5_800_000_001)
  assert expired.source_bus == 2
  assert not expired.available
  assert expired.distance == 60


def test_observer_accepts_two_hz_frames_but_expires_after_750ms():
  observer = TeslaTrafficControlObserver()
  address, data, _ = _frame({
    "APP_tcFeatureState": 3,
    "APP_tcControlSource": 2,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 50,
    "APP_tcControlLightState": 1,
  })
  observer.update([(4_000_000_000, [(address, data, 2)])], 4_000_000_000)
  assert observer.snapshot(4_200_000_000).available
  assert observer.snapshot(4_700_000_000).available
  assert not observer.snapshot(4_750_000_001).available


def test_observer_accepts_200m_boundary_and_rejects_201m_for_control():
  observer = TeslaTrafficControlObserver()
  for timestamp, distance in ((6_000_000_000, 200), (6_100_000_000, 201)):
    address, data, _ = _frame({
      "APP_tcControlType": 3,
      "APP_tcControlDistance": distance,
      "APP_tcControlLightState": 1,
    })
    observer.update([(timestamp, [(address, data, 2)])], timestamp)
    assert observer.snapshot(timestamp).valid_for_control == (distance == 200)
