from opendbc.can import CANPacker

from openpilot.selfdrive.debug.tesla_can_visualization import TeslaCanVisualization


def _frame(packer, message, bus, values):
  return packer.make_can_msg(message, bus, values)


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
