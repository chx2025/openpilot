from opendbc.can import CANPacker

from openpilot.selfdrive.debug.tesla_can_visualization import TeslaCanVisualization


def _frame(packer, message, bus, values):
  return packer.make_can_msg(message, bus, values)


def test_tesla_can_visualization_decodes_party_lateral_and_cruise_commands():
  packer = CANPacker("tesla_model3_party")
  frames = [
    _frame(packer, "DAS_steeringControl", 2, {
      "DAS_steeringAngleRequest": 12.5,
      "DAS_steeringControlType": 1,
      "DAS_steeringHapticRequest": 0,
    }),
    _frame(packer, "DAS_control", 2, {
      "DAS_setSpeed": 88.0,
      "DAS_accelMin": -1.2,
      "DAS_accelMax": 1.6,
      "DAS_jerkMin": -2.0,
      "DAS_jerkMax": 2.5,
      "DAS_accState": 4,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  commands = visualization.snapshot(1_100_000_000)["actuation_commands"]

  assert commands["steering"] == {
    "available": True,
    "address": "0x488",
    "bus": "AP-PARTY",
    "angle_request_deg": 12.45,
    "control_type": 1,
    "haptic_request": False,
  }
  assert commands["cruise"] == {
    "available": True,
    "address": "0x2B9",
    "bus": "AP-PARTY",
    "set_speed_kph": 88.0,
    "accel_min_mps2": -1.2,
    "accel_max_mps2": 1.6,
    "jerk_min_mps3": -2.01,
    "jerk_max_mps3": 2.52,
    "acc_state": 4,
  }
  stale = visualization.snapshot(4_000_000_000)["actuation_commands"]
  assert not stale["steering"]["available"]
  assert not stale["cruise"]["available"]


def test_party_control_addresses_are_not_decoded_from_vehicle_bus() -> None:
  packer = CANPacker("tesla_model3_party")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [
    _frame(packer, "DAS_steeringControl", 1, {"DAS_steeringAngleRequest": 10.0}),
    _frame(packer, "DAS_control", 1, {"DAS_setSpeed": 80.0}),
  ])])

  commands = visualization.snapshot(1_100_000_000)["actuation_commands"]

  assert not commands["steering"]["available"]
  assert not commands["cruise"]["available"]


def test_tesla_can_visualization_builds_scene_from_multiple_buses():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "UI_driverAssistMapData", 1, {
      "UI_navRouteActive": 1,
      "UI_gpsRoadMatch": 1,
      "UI_mapSpeedUnits": 1,
      "UI_mapSpeedLimit": 13,
      "UI_nextBranchDist": 120,
      "UI_nextBranchRightOffRamp": 1,
      "UI_parallelAutoparkEnabled": 1,
      "UI_inSuperchargerGeofence": 1,
      "UI_rejectNav": 1,
    }),
    _frame(packer, "DAS_lanes", 4, {
      "DAS_leftLaneExists": 1,
      "DAS_rightLaneExists": 1,
      "DAS_virtualLaneWidth": 3.5,
      "DAS_virtualLaneViewRange": 80,
      "DAS_virtualLaneC0": 0,
      "DAS_virtualLaneC1": 0,
      "DAS_virtualLaneC2": 0,
      "DAS_virtualLaneC3": 0,
      "DAS_leftLineUsage": 2,
      "DAS_rightLineUsage": 2,
    }),
    _frame(packer, "APP_trafficControl", 2, {
      "APP_tcFeatureState": 3,
      "APP_tcStateMachine": 4,
      "APP_tcControlSource": 3,
      "APP_tcControlType": 3,
      "APP_tcControlDistance": 42,
      "APP_tcControlLightState": 1,
      "APP_tcVisionLight": 1,
      "APP_tcVisionLine": 1,
    }),
    _frame(packer, "DAS_object", 4, {
      "DAS_objectId": 0,
      "DAS_leadVehType": 2,
      "DAS_leadVehRelevantForControl": 1,
      "DAS_leadVehDx": 25,
      "DAS_leadVehVxRel": -2,
      "DAS_leadVehDy": 0,
      "DAS_leadVehId": 7,
    }),
  ]
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)

  assert scene["available"]
  assert scene["buses"] == ["AP-PARTY", "CH", "VEH"]
  assert scene["navigation"]["route_active"]
  assert scene["navigation"]["next_branch_distance_m"] == 120
  assert scene["navigation"]["speed_limit"] == 60
  assert scene["navigation"]["parallel_autopark_enabled"]
  assert scene["navigation"]["in_supercharger_geofence"]
  assert scene["navigation"]["reject_navigation"]
  assert scene["lanes"]["left_usage"] == "fused"
  assert scene["lanes"]["right_usage"] == "fused"
  assert scene["traffic"]["light_state"] == "red"
  assert scene["traffic"]["control_available"]
  assert not scene["traffic"]["road_sign_available"]
  assert scene["traffic"]["control_distance_m"] == 42
  assert scene["vehicles"] == [{
    "category": "lead", "index": 1, "track_id": 7, "type": "car", "x_m": 25.0, "y_m": -0.0,
    "relative_speed": -2.0, "relevant_for_control": True, "heading_rad": None,
  }]


def test_tesla_can_visualization_hides_stale_optional_data():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "APP_trafficControl", 2, {
    "APP_tcFeatureState": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 20,
    "APP_tcControlLightState": 2,
  })
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [frame])])

  assert visualization.snapshot(1_100_000_000)["traffic"]["available"]
  assert not visualization.snapshot(4_000_000_000)["traffic"]["available"]
  assert not visualization.snapshot(4_000_000_000)["available"]


def test_tesla_can_visualization_reset_discards_cached_vehicle_data():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "DAS_object", 4, {
    "DAS_objectId": 0,
    "DAS_leadVehType": 2,
    "DAS_leadVehDx": 15,
    "DAS_leadVehId": 4,
  })
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, [frame])])
  assert visualization.snapshot(1_100_000_000)["vehicles"]

  visualization.reset()
  assert not visualization.snapshot(1_100_000_000)["available"]


def test_tesla_can_visualization_hides_idle_traffic_control_without_light():
  """A fresh control/sign frame is not proof of a traffic light: idle/SNA
  values must not render as a green light a few meters ahead."""
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "APP_trafficControl", 2, {
      "APP_tcFeatureState": 0,
      "APP_tcStateMachine": 0,
      "APP_tcControlType": 1,
      "APP_tcControlDistance": 1,
      "APP_tcControlLightState": 2,
    }),
    _frame(packer, "DAS_object", 4, {
      "DAS_objectId": 4,
      "DAS_roadSignId": 255,
      "DAS_roadSignStopLineDist": 1.0,
      "DAS_roadSignControlActive": 0,
      "DAS_roadSignSource": 0,
    }),
  ]
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, frames)])

  traffic = visualization.snapshot(1_100_000_000)["traffic"]
  assert traffic["control_frame_fresh"]
  assert traffic["sign_frame_fresh"]
  assert not traffic["available"]
  assert not traffic["control_available"]
  assert not traffic["road_sign_available"]
  assert traffic["light_state"] == "unknown"
  assert traffic["control_distance_m"] is None
  assert traffic["stop_line_distance_m"] is None


def test_tesla_can_visualization_surfaces_party_visual_light_when_feature_is_disabled():
  """ESP32-S3 PARTY capture: feature=disabled, but type/source/light/distance
  still form a coherent visual traffic-light observation."""
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [(0x25D, bytes.fromhex("18 1E 03 39 40 24"), 2)])])

  traffic = visualization.snapshot(1_100_000_000)["traffic"]
  assert traffic["control_frame_fresh"]
  assert traffic["available"]
  assert not traffic["control_available"]
  assert traffic["light_observation_available"]
  assert traffic["feature_state"] == "disabled"
  assert traffic["feature_state_code"] == 0
  assert traffic["state_machine_code"] == 6
  assert traffic["control_source_code"] == 3
  assert traffic["control_type_code"] == 3
  assert traffic["control_source"] == "map_and_vision"
  assert traffic["control_type"] == "traffic_light"
  assert traffic["light_state"] == "red"
  assert traffic["control_distance_m"] == 3.0
  assert traffic["sources"] == ["AP-PARTY"]


def test_tesla_can_visualization_uses_only_ap_party_for_traffic_control():
  sample = (0x25D, bytes.fromhex("18 1E 28 2A 40 04"))

  for bus, ch_bus in ((0, None), (1, None), (4, 4)):
    visualization = TeslaCanVisualization(ch_bus=ch_bus)
    visualization.update([(1_000_000_000, [(*sample, bus)])])
    traffic = visualization.snapshot(1_100_000_000)["traffic"]
    assert not traffic["control_frame_fresh"]
    assert not traffic["available"]

  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [(*sample, 2)])])
  traffic = visualization.snapshot(1_100_000_000)["traffic"]
  assert traffic["light_observation_available"]
  assert traffic["light_state"] == "green"
  assert traffic["control_distance_m"] == 40.0
  assert traffic["sources"] == ["AP-PARTY"]


def test_tesla_can_visualization_traffic_light_sign_gates_stop_line_and_arrow():
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization(ch_bus=4)

  def sign_traffic(sign_values):
    visualization.reset()
    visualization.update([(1_000_000_000, [_frame(packer, "DAS_object", 4, {"DAS_objectId": 4, **sign_values})])])
    return visualization.snapshot(1_100_000_000)["traffic"]

  invalid = sign_traffic({
    "DAS_roadSignId": 255,
    "DAS_roadSignStopLineDist": 30,
    "DAS_roadSignSource": 0,
  })
  assert not invalid["road_sign_available"]
  assert invalid["stop_line_distance_m"] is None

  valid = sign_traffic({
    "DAS_roadSignId": 1,
    "DAS_roadSignStopLineDist": 30,
    "DAS_roadSignColor": 1,
    "DAS_roadSignArrow": 1,
    "DAS_roadSignSource": 2,
    "DAS_roadSignControlActive": 1,
  })
  assert valid["road_sign_available"]
  assert valid["stop_line_distance_m"] == 30
  assert valid["road_sign_arrow"] == "left"
  assert valid["road_sign_color"] == "red"


def test_tesla_can_visualization_rear_uses_live_flags_not_trip_latches():
  packer = CANPacker("tesla_modely_hw4_perception")

  def rear_snapshot(**flags):
    visualization = TeslaCanVisualization(ch_bus=4)
    visualization.update([(1_000_000_000, [_frame(packer, "DAS_visualDebug", 4, flags)])])
    return visualization.snapshot(1_100_000_000)["rear_vehicles"]

  only_trip = rear_snapshot(DAS_rearLeftVehDetectedTrip=1, DAS_rearRightVehDetectedTrip=1)
  assert not only_trip["left_live"]
  assert not only_trip["right_live"]

  left_now = rear_snapshot(DAS_rearLeftVehDetectedCurrent=1)
  assert left_now["left_live"]
  assert not left_now["right_live"]

  right_now = rear_snapshot(DAS_rearVehDetectedThisCycle=1, DAS_rearRightVehDetectedTrip=1)
  assert right_now["right_live"]
  assert not right_now["left_live"]


def test_tesla_can_visualization_decodes_road_sign_pedestrian_blind_spot_and_front_safety():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "UI_driverAssistRoadSign", 4, {
      "UI_roadSign": 1,
      "UI_stopSignStopLineDist": 12.0,
      "UI_stopSignStopLineConf": 100,
    }),
    _frame(packer, "UI_driverAssistRoadSign", 4, {
      "UI_roadSign": 2,
      "UI_trafficLightStopLineDist": 30.0,
      "UI_trafficLightStopLineConf": 90,
    }),
    _frame(packer, "APP_pedestrianDetection", 1, {
      "APP_pedestrianDetectedFrontMain": 1,
      "APP_pedestrianDetectedBackup": 1,
      "APP_closestPedestrian1dX": 3.2,
      "APP_closestPedestrian1dY": -1.6,
      "APP_closestPedestrian2dX": 5.0,
    }),
    _frame(packer, "DAS_status", 2, {
      "DAS_blindSpotRearLeft": 2,
      "DAS_blindSpotRearRight": 1,
      "DAS_sideCollisionWarning": 1,
      "DAS_forwardCollisionWarning": 1,
    }),
    _frame(packer, "DAS_integratedSafetyFront", 2, {
      "DAS_targetDistanceFront": 12.0,
      "DAS_targetDistanceFrontQF": 1,
      "DAS_relativeVelocityFront": -4.0,
      "DAS_relativeVelocityFrontQF": 1,
      "DAS_timeToImpactFront": 30,
      "DAS_timeToImpactFrontQF": 1,
      "DAS_predictedImpactOvrlapFront": 62.5,
      "DAS_predictedImpactOvrlapFrontQF": 1,
    }),
  ]
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)
  road_sign = scene["road_sign"]
  assert road_sign["available"]
  assert road_sign["stop_sign_stop_line_distance_m"] == 12.0
  assert road_sign["stop_sign_stop_line_confidence"] == 100
  assert road_sign["traffic_light_stop_line_distance_m"] == 30.0
  assert road_sign["traffic_light_stop_line_confidence"] == 90

  pedestrians = scene["pedestrian_detection"]
  assert pedestrians["available"]
  assert pedestrians["front_main"]
  assert pedestrians["backup"]
  assert pedestrians["camera_mask"] == 0x81
  assert pedestrians["simultaneous_front_rear"]
  assert pedestrians["coordinate_slots"][0]["dx_scaled"] == 3.2
  assert pedestrians["coordinate_slots"][0]["dy_scaled"] == -1.6
  assert not pedestrians["position_available"]

  blind_spot = scene["blind_spot"]
  assert blind_spot["available"]
  assert blind_spot["left_level"] == 2
  assert blind_spot["right_level"] == 1
  assert blind_spot["left_live"]
  assert blind_spot["right_live"]
  assert blind_spot["side_collision_warning_level"] == 1
  assert blind_spot["forward_collision_warning_level"] == 1

  front_safety = scene["front_safety"]
  assert front_safety["available"]
  assert front_safety["target_distance_m"] == 12.0
  assert front_safety["relative_velocity_mps"] == -4.0
  assert front_safety["time_to_impact_s"] == 30.0
  assert front_safety["predicted_impact_overlap_pct"] == 62.5


def test_tesla_can_visualization_gates_road_sign_stop_line_sna():
  """Idle/SNA road sign frames must not produce a bogus stop line distance."""
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, [_frame(packer, "UI_driverAssistRoadSign", 4, {
    "UI_roadSign": 2,
    "UI_trafficLightStopLineDist": -8.0,
    "UI_trafficLightStopLineConf": 0,
  })])])

  road_sign = visualization.snapshot(1_100_000_000)["road_sign"]
  assert road_sign["available"]
  assert road_sign["traffic_light_stop_line_distance_m"] is None
  assert road_sign["traffic_light_stop_line_confidence"] is None


def test_tesla_can_visualization_rejects_same_address_from_wrong_physical_bus():
  """0x30A on PARTY is not the CH DAS_object message."""
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [_frame(packer, "DAS_object", 0, {
    "DAS_objectId": 0,
    "DAS_leadVehType": 2,
    "DAS_leadVehDx": 8.0,
    "DAS_leadVehId": 7,
  })])])

  scene = visualization.snapshot(1_100_000_000)
  assert scene["vehicles"] == []
  assert "PARTY" not in scene["buses"]


def test_tesla_can_visualization_hides_pedestrian_coordinates_without_detection_flag():
  """Coordinate slots without a camera detection flag are not actionable."""
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [_frame(packer, "APP_pedestrianDetection", 1, {
    "APP_closestPedestrian1dX": 12.4,
    "APP_closestPedestrian1dY": 12.4,
    "APP_closestPedestrian2dX": 12.4,
    "APP_closestPedestrian2dY": 12.4,
    "APP_closestPedestrian3dX": 12.4,
    "APP_closestPedestrian3dY": 12.4,
  })])])

  pedestrians = visualization.snapshot(1_100_000_000)["pedestrian_detection"]
  assert pedestrians["available"]
  assert not pedestrians["detected_any"]
  assert pedestrians["coordinate_slots"] == []
  assert "closest" not in pedestrians


def test_tesla_can_visualization_keeps_pedestrian_slot_validity_unverified():
  """T-CAN gives the 0x400 coordinate fields no unit, confidence, track id,
  or per-slot validity bit. They are diagnostics, not positioned objects."""
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [_frame(packer, "APP_pedestrianDetection", 1, {
    "APP_pedestrianDetectedFrontMain": 1,
    "APP_closestPedestrian1dX": 3.2,
    "APP_closestPedestrian1dY": -1.6,
    "APP_closestPedestrian2dX": 5.2,
    "APP_closestPedestrian2dY": 4.0,
  })])])

  pedestrians = visualization.snapshot(1_100_000_000)["pedestrian_detection"]
  assert pedestrians["detected_any"]
  assert pedestrians["active_cameras"] == ["front_main"]
  assert not pedestrians["position_available"]
  assert pedestrians["positioned_objects"] == []
  assert pedestrians["coordinate_unit"] == "m"
  assert pedestrians["coordinate_slots"] == [
    {"index": 1, "dx_scaled": 3.2, "dy_scaled": -1.6, "validity": "unknown"},
    {"index": 2, "dx_scaled": 5.2, "dy_scaled": 4.0, "validity": "unknown"},
    {"index": 3, "dx_scaled": 0.0, "dy_scaled": 0.0, "validity": "unknown"},
  ]
  assert "closest" not in pedestrians


def test_tesla_can_visualization_expires_pedestrian_flags_quickly_and_surfaces_collision_warning():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "APP_pedestrianDetection", 1, {"APP_pedestrianDetectedFrontMain": 1}),
    _frame(packer, "DAS_status2", 2, {"DAS_longCollisionWarning": 2}),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  fresh = visualization.snapshot(1_100_000_000)["pedestrian_detection"]
  assert fresh["detected_any"]
  assert fresh["collision_warning"]
  assert fresh["evidence_tier"] == "collision_warning"

  stale = visualization.snapshot(1_800_000_000)["pedestrian_detection"]
  assert not stale["available"]
  assert not stale["detected_any"]
  assert stale["collision_warning"]


def test_tesla_can_visualization_requires_front_safety_quality_flags():
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [_frame(packer, "DAS_integratedSafetyFront", 2, {
    "DAS_targetDistanceFront": 12.0,
    "DAS_relativeVelocityFront": -4.0,
    "DAS_timeToImpactFront": 30.0,
    "DAS_predictedImpactOvrlapFront": 62.5,
    "DAS_imminentCollisionFront": 1,
  })])])

  front_safety = visualization.snapshot(1_100_000_000)["front_safety"]
  assert front_safety["available"]
  assert front_safety["target_distance_m"] is None
  assert front_safety["relative_velocity_mps"] is None
  assert front_safety["time_to_impact_s"] is None
  assert front_safety["predicted_impact_overlap_pct"] is None
  assert not front_safety["imminent_collision"]


def test_tesla_can_visualization_decodes_read_only_longitudinal_shadow():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "DAS_longControl", 2, {
      "DAS_longControlStack": 2,
      "DAS_torqueProfiler_accelMinPed": -3.0,
      "DAS_torqueProfiler_accelMaxPed": 1.0,
      "DAS_torqueProfiler_targetSpeedPed": 80.0,
    }),
    _frame(packer, "DAS_longControl", 2, {
      "DAS_longControlStack": 4,
      "DAS_aebControl_active": 2,
      "DAS_aebControl_targetAccelDis": -3.0,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  shadow = visualization.snapshot(1_100_000_000)["longitudinal_shadow"]
  assert shadow["available"]
  assert shadow["read_only"]
  assert shadow["current_stack"] == "aeb_control"
  assert shadow["torque_profiler"]["target_speed_kph"] == 80.0
  assert shadow["aeb"]["active"]
  assert shadow["aeb"]["target_accel_mps2"] == -3.0


def test_tesla_can_visualization_decodes_valid_parking_obstacle_and_pmm_status():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "PARK_oocStatus", 1, {
      "PARK_oocDistance": 180,
      "PARK_oocConfidence": 90,
      "PARK_oocVehicleX": 50,
      "PARK_oocVehicleY": -20,
      "PARK_oocCollisionSide": 1,
      "PARK_oocUntrackedTime": 0.2,
    }),
    _frame(packer, "DAS_status2", 2, {
      "DAS_pmmObstacleSeverity": 3,
      "DAS_longCollisionWarning": 2,
      "DAS_pmmUltrasonicsFaultReason": 0,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)
  obstacle = scene["parking_obstacle"]
  assert obstacle["available"]
  assert obstacle["valid_obstacle"]
  assert obstacle["distance_m"] == 1.8
  assert obstacle["x_m"] == 0.5
  assert obstacle["y_m"] == -0.2
  assert obstacle["collision_side"] == "right"

  proximity = scene["proximity_safety"]
  assert proximity["available"]
  assert proximity["read_only"]
  assert proximity["obstacle_severity"] == 3
  assert proximity["long_collision_warning"] == 2


def test_tesla_can_visualization_exposes_ch_objects_only_when_ch_source_is_configured():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "DAS_object", 4, {
    "DAS_objectId": 2,
    "DAS_rightVehType": 2,
    "DAS_rightVehDx": 8.0,
    "DAS_rightVehDy": -2.0,
    "DAS_rightVehId": 12,
  })
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, [frame])])

  scene = visualization.snapshot(1_100_000_000)
  assert scene["capabilities"]["ch_bus_configured"]
  assert scene["capabilities"]["oem_object_list_available"]
  assert not scene["capabilities"]["control_integration_enabled"]
  assert scene["vehicles"][0]["category"] == "right"
  assert scene["vehicles"][0]["x_m"] == 8.0
  assert scene["vehicles"][0]["y_m"] == -2.1
  assert scene["vehicles"][0]["track_id"] == 12


def test_tesla_can_visualization_uses_ch_pedestrian_object_as_positioned_evidence():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "DAS_object", 4, {
    "DAS_objectId": 0,
    "DAS_leadVehType": 5,
    "DAS_leadVehDx": 16.0,
    "DAS_leadVehDy": 1.0,
    "DAS_leadVehId": 23,
  })
  visualization = TeslaCanVisualization(ch_bus=4)
  visualization.update([(1_000_000_000, [frame])])

  pedestrian = visualization.snapshot(1_100_000_000)["pedestrian_detection"]
  assert not pedestrian["available"]
  assert not pedestrian["detected_any"]
  assert pedestrian["evidence_present"]
  assert pedestrian["evidence_tier"] == "positioned_object"
  assert pedestrian["position_available"]
  positioned = pedestrian["positioned_objects"]
  assert len(positioned) == 1
  assert positioned[0]["category"] == "lead"
  assert positioned[0]["track_id"] == 23
  assert positioned[0]["type"] == "pedestrian"
  assert positioned[0]["x_m"] == 16.0
  assert positioned[0]["y_m"] == 1.05


def test_tesla_can_visualization_decodes_detailed_vehicle_bus_diagnostics():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "APP_roadDisturbance", 1, {
      "APP_roadDisturbanceIndex": 2, "APP_roadDisturbanceHeight": 0.12,
      "APP_roadDisturbanceX0": 10.0, "APP_roadDisturbanceX1": 15.0,
      "APP_roadDisturbanceY0": -1.0, "APP_roadDisturbanceY1": 1.0,
      "APP_suspensionLevelRequest": 3,
    }),
    _frame(packer, "BMS_hvBusStatus", 1, {
      "BMS_dcLinkVoltage": 402.5, "BMS_packCurrent": -80.0, "BMS_currentUnfiltered": -79.5,
    }),
    _frame(packer, "BMS_status", 1, {
      "BMS_hvacPowerRequest": 1, "BMS_preconditionAllowed": 1, "BMS_contactorState": 4,
      "BMS_userChargeStatus": 3, "BMS_batteryInputPower": 50.0, "BMS_chargeRequest": 1,
      "BMS_state": 1, "BMS_chgPowerAvailable": 120.0, "BMS_conditioningRequest": 1,
      "BMS_smStateRequest": 1, "BMS_hvState": 3,
    }),
    _frame(packer, "VCSEC_TPMSData", 1, {
      "VCSEC_TPMSDataIndex": 0, "VCSEC_TPMSPressure0": 2.5, "VCSEC_TPMSTemperature0": 30,
      "VCSEC_TPMSBatVoltage0": 2.8, "VCSEC_TPMSLocation0": 0,
      "VCSEC_TPMSTemperatureCompensatedPressure0": 2.45, "VCSEC_TPMSPressureRateOfChange0": -0.02,
      "VCSEC_TPMSCapabilityPressureInAdv0": 1, "VCSEC_TPMSCapabilityConfigurablePressure0": 1,
    }),
    _frame(packer, "VCSEC_TPMSData", 1, {
      "VCSEC_TPMSDataIndex": 4, "VCSEC_TPMSRecommendedColdPressureFront": 2.9,
      "VCSEC_TPMSRecommendedColdPressureRear": 3.0, "VCSEC_TPMSFeature0": 5,
      "VCSEC_TPMSFeature1": 4, "VCSEC_TPMSFeature0Count": 3, "VCSEC_TPMSFeature0TimeS": 12,
    }),
    _frame(packer, "VCSEC_TPMSData", 1, {
      "VCSEC_TPMSDataIndex": 5, "VCSEC_TPMSAutonomyStatus": 0,
      "VCSEC_TPMSLastKnownPressureFL": 2.5, "VCSEC_TPMSLastKnownPressureFR": 2.55,
      "VCSEC_TPMSLastKnownPressureRL": 2.6, "VCSEC_TPMSLastKnownPressureRR": 2.65,
    }),
    _frame(packer, "VCSEC_TPMSDisplay", 1, {
      "VCSEC_TPMSDisplayPressureFL": 2.5, "VCSEC_TPMSDisplayPressureFR": 2.55,
      "VCSEC_TPMSDisplayPressureRL": 2.6, "VCSEC_TPMSDisplayPressureRR": 2.65,
      "VCSEC_TPMSTellTale": 1, "VCSEC_TPMSDisplaySoftWarningIndicationFL": 1,
    }),
    _frame(packer, "TPMS_data", 1, {
      "TPMS_pressureFL": 2.5, "TPMS_temperatureFL": 30,
      "TPMS_pressureFR": 2.55, "TPMS_temperatureFR": 31,
      "TPMS_pressureRL": 2.6, "TPMS_temperatureRL": 32,
      "TPMS_pressureRR": 2.65, "TPMS_temperatureRR": 33,
    }),
    _frame(packer, "DIR_power", 1, {
      "DIR_elecPower": -10, "DIR_heatPowerOptimal": 2, "DIR_heatPowerMax": 4,
      "DIR_heatPowerActual": 3, "DIR_excessHeatCommand": 1, "DIR_drivePowerMax": 250,
    }),
    _frame(packer, "DIF_power", 1, {
      "DIF_elecPower": 20, "DIF_heatPowerOptimal": 2, "DIF_heatPowerMax": 4,
      "DIF_heatPowerActual": 3, "DIF_excessHeatCommand": 1, "DIF_drivePowerMax": 240,
    }),
    _frame(packer, "DIR_temperature", 1, {
      "DIR_tempIndex": 0, "DIR_inverterTQF": 2, "DIR_pcbT": 45, "DIR_inverterT": 50,
      "DIR_statorT": 55, "DIR_dcCapT": 42, "DIR_heatsinkT": 44,
      "DIR_inverterTpct": 40, "DIR_statorTpct": 44,
    }),
    _frame(packer, "DIR_temperature", 1, {
      "DIR_tempIndex": 1, "DIR_heatsink1Temp": 41, "DIR_heatsink2Temp": 42,
      "DIR_heatsink3Temp": 43, "DIR_pcbTemp2": 44, "DIR_junctionTemp": 48,
      "DIR_TPak1Temp": 46, "DIR_TPak2Temp": 47,
    }),
    _frame(packer, "DIR_temperature", 1, {
      "DIR_tempIndex": 2, "DIR_fluidInTemp": 35,
      "DIR_normalFetBurnIn": 1.526, "DIR_additionalFetBurnIn": 0.763,
    }),
    _frame(packer, "DIF_temperature", 1, {
      "DIF_tempIndex": 0, "DIF_inverterTQF": 2, "DIF_pcbT": 43, "DIF_inverterT": 48,
      "DIF_statorT": 53, "DIF_dcCapT": 40, "DIF_heatsinkT": 42,
    }),
    _frame(packer, "DI_odometerStatus", 1, {"DI_odometer": 12345.678, "DI_obdDriveCycleStatus": 1}),
    _frame(packer, "BMS_kwhCounter", 1, {"BMS_kwhDischargeTotal": 120.5, "BMS_kwhChargeTotal": 100.25}),
    _frame(packer, "DI_estimatedBrakeTemp", 1, {
      "DI_brakeFLTemp": 85, "DI_brakeFRTemp": 86, "DI_brakeRLTemp": 65, "DI_brakeRRTemp": 66,
      "DI_mcpIndex": 1.0, "DI_mcpIndexPrimeFilt": 1.2,
    }),
    _frame(packer, "UI_ambientLightingCtrls", 1, {
      "UI_ambientLightPowerOverride": 1, "UI_rgbEnableState": 2, "UI_rgbEffectType": 4,
      "UI_rgbLightingColorHexRed": 18, "UI_rgbLightingColorHexGreen": 52,
      "UI_rgbLightingColorHexBlue": 86, "UI_rgbBrightnessLevel": 60,
      "UI_audioVisualizerState": 1, "UI_rgbTargetDOORFL": 1, "UI_rgbTargetIPFR": 1,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)
  assert scene["road_disturbance"]["longitudinal_span_m"] == 5.0
  assert scene["battery_diagnostics"]["dc_link_voltage_v"] == 402.5
  assert scene["battery_diagnostics"]["contactor_state"] == "closed"
  assert scene["battery_diagnostics"]["hv_state"] == "up_for_drive"
  assert scene["tpms"]["wheels"]["front_left"]["display_pressure_bar"] == 2.5
  assert scene["tpms"]["sensors"][0]["location"] == "front_left"
  assert scene["tpms"]["feature_state"] == "active"
  assert scene["drive_power"]["front"]["electrical_power_kw"] == 20.0
  assert scene["drive_power"]["rear"]["electrical_power_kw"] == -10.0
  assert scene["drive_temperatures"]["rear"]["received_pages"] == [0, 1, 2]
  assert scene["drive_temperatures"]["rear"]["operating_c"]["stator"] == 55.0
  assert scene["drive_temperatures"]["rear"]["operating_percent"]["inverter"] == 40.0
  assert scene["drive_temperatures"]["rear"]["fet_burn_in"]["normal"] == 1.53
  assert scene["vehicle_totals"]["odometer_km"] == 12345.678
  assert scene["vehicle_totals"]["brake_temperature_c"]["front_right"] == 86.0
  assert scene["ambient_lighting"]["hex_color"] == "#123456"
  assert scene["ambient_lighting"]["effect_duration_ms"] == 1000
  assert scene["ambient_lighting"]["targets"] == ["front_left_door", "instrument_panel_right"]


def test_tesla_can_visualization_expires_vehicle_mux_pages():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "DIR_temperature", 1, {
    "DIR_tempIndex": 0, "DIR_inverterTQF": 2, "DIR_inverterT": 50,
  })
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [frame])])

  assert visualization.snapshot(1_100_000_000)["drive_temperatures"]["rear"]["available"]
  assert not visualization.snapshot(7_000_000_000)["drive_temperatures"]["rear"]["available"]
