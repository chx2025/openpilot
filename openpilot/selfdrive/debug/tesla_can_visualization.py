"""Read-only Tesla Model Y HW4 CAN context for the local web visualization.

This parser is deliberately owned by the debug web process. It is not part of
CarInterface, CAN validity, safety, or any control decision.
"""
from __future__ import annotations

import math
import time
from typing import Any

from opendbc.can import CANParser


DBC_NAME = "tesla_modely_hw4_perception"
VEH_MESSAGES = (
  "UI_driverAssistMapData",
  "PARK_oocStatus",
  "APP_pedestrianDetection",
)
PARTY_MESSAGES = (
  "APP_trafficControl",
  "DAS_longControl",
  "DAS_status2",
  "DAS_status",
  "DAS_integratedSafetyFront",
)
CH_MESSAGES = (
  "APP_trafficControl",
  "DAS_visualDebug",
  "DAS_lanes",
  "UI_driverAssistRoadSign",
  "DAS_statusCH",
  "DAS_object",
)
MESSAGE_NAMES = (*VEH_MESSAGES, *PARTY_MESSAGES, *CH_MESSAGES)
BUS_NAMES = {0: "PARTY", 1: "VEH", 2: "AP-PARTY"}
FRAME_STALE_NS = 2_000_000_000
NAV_STALE_NS = 5_000_000_000
PEDESTRIAN_STALE_NS = 750_000_000

USAGE = {0: "rejected", 1: "available", 2: "fused", 3: "blacklisted"}
VEHICLE_TYPES = {0: "unknown", 1: "truck", 2: "car", 3: "motorcycle", 4: "bicycle", 5: "pedestrian", 6: "ipso"}
ROAD_CLASSES = {0: "unknown", 1: "class_1_major", 2: "class_2", 3: "class_3", 4: "class_4", 5: "class_5", 6: "class_6_minor"}
SPEED_LIMIT_TYPES = {1: "regular", 2: "advisory", 3: "dependent", 4: "bumps", 7: "unknown"}
SPEED_LIMIT_VALUES = {
  1: 5, 2: 7, 3: 10, 4: 15, 5: 20, 6: 25, 7: 30, 8: 35, 9: 40, 10: 45,
  11: 50, 12: 55, 13: 60, 14: 65, 15: 70, 16: 75, 17: 80, 18: 85, 19: 90,
  20: 95, 21: 100, 22: 105, 23: 110, 24: 115, 25: 120, 26: 130, 27: 140,
  28: 150, 29: 160,
}
TRAFFIC_FEATURE_STATES = {0: "disabled", 1: "unavailable", 2: "available", 3: "active"}
TRAFFIC_MACHINE_STATES = {0: "disabled", 1: "standby", 2: "aware", 3: "warning", 4: "stopping", 5: "stopped", 6: "continuing"}
TRAFFIC_SOURCES = {0: "none", 1: "map", 2: "vision", 3: "map_and_vision"}
TRAFFIC_TYPES = {
  0: "invalid", 1: "unknown", 2: "stop_sign", 3: "traffic_light", 4: "yield", 5: "crosswalk",
  6: "keep_clear_enter", 7: "keep_clear_exit", 8: "suicide_left", 9: "pedestrian_crossing",
  10: "ramp_meter", 11: "speed_bump", 12: "speed_hump", 13: "traffic_rule",
  14: "all_way_stop_sign", 15: "bike_merge_from_left", 16: "bike_merge_from_right",
  17: "no_stop", 18: "t_implicit", 19: "t_implicit_by_name", 20: "t_implicit_by_geometry",
  21: "bev_junction", 22: "t_arm",
}
LIGHT_STATES = {0: "none", 1: "red", 2: "green", 3: "yellow", 4: "off", 5: "white", 6: "other"}
ROAD_SIGN_COLORS = {0: "none", 1: "red", 2: "yellow", 3: "green", 4: "red_yellow"}
ROAD_SIGN_TYPES = {0: "stop_sign", 1: "traffic_light", 255: "unavailable"}
ROAD_SIGN_SOURCES = {0: "none", 1: "navigation", 2: "vision"}
ROAD_SIGN_ARROWS = {0: "circle", 1: "left", 2: "right", 3: "straight", 4: "unknown"}
PLANNER_STATES = {
  0: "disabled", 1: "virtual_lane", 2: "follow", 3: "lane_change_requested", 4: "lane_change_in_progress",
  5: "waiting_side_obstacle", 6: "waiting_forward_obstacle", 7: "lane_change_abort",
}
BEHAVIOR_TYPES = {0: "invalid", 1: "in_lane", 2: "lane_change_left", 3: "lane_change_right"}
HEALTH_STATES = {0: "unavailable", 1: "nominal", 2: "degraded", 3: "severely_degraded", 4: "aborting", 5: "fault"}
ROAD_SURFACES = {0: "unknown", 1: "normal", 2: "enhanced"}
LONG_CONTROL_STACKS = {
  0: "none", 1: "reserved", 2: "torque_profiler", 3: "velocity_profile",
  4: "aeb_control", 5: "pedal_control", 6: "torque_control", 7: "sna",
}
COLLISION_SIDES = {0: "none", 1: "right", 2: "left", 3: "front", 4: "rear", 5: "unknown", 7: "sna"}


def _round(value: float, digits: int = 2) -> float:
  return round(float(value), digits)


def _int(values: dict[str, float], name: str) -> int:
  return int(values.get(name, 0.0))


def _bool(values: dict[str, float], name: str) -> bool:
  return bool(_int(values, name))


class TeslaCanVisualization:
  """Decode optional OEM context without participating in vehicle control."""

  def __init__(self, buses: tuple[int, ...] = (0, 1, 2), ch_bus: int | None = None) -> None:
    if ch_bus in (1, 2):
      raise ValueError("CH must use a dedicated CAN source, not VEH or AP-PARTY")

    # The harness exposes VEH on source 1 and the Autopilot side of PARTY on
    # source 2; source 0 may also expose the other PARTY side. CH is not
    # present on the stock three-bus Panda connection. A
    # DBC must never be applied by address alone across these networks: e.g.
    # PARTY also carries 0x30A, but it is not the CH DAS_object message.
    self.ch_bus = ch_bus
    enabled_buses = set(buses)
    message_buses: dict[int, tuple[str, ...]] = {}
    if 0 in enabled_buses:
      message_buses[0] = PARTY_MESSAGES
    if 1 in enabled_buses:
      message_buses[1] = VEH_MESSAGES
    if 2 in enabled_buses:
      message_buses[2] = PARTY_MESSAGES
    if ch_bus is not None:
      message_buses[ch_bus] = CH_MESSAGES

    self.parser_messages = message_buses
    self.bus_names = {**BUS_NAMES, **({ch_bus: "CH"} if ch_bus is not None else {})}
    self.parsers = {
      bus: CANParser(DBC_NAME, [(name, math.nan) for name in names], bus)
      for bus, names in message_buses.items()
    }
    self.frames: dict[str, tuple[dict[str, float], int, int]] = {}
    self.object_frames: dict[int, tuple[dict[str, float], int, int]] = {}
    self.road_sign_frames: dict[int, tuple[dict[str, float], int, int]] = {}
    self.long_control_frames: dict[int, tuple[dict[str, float], int, int]] = {}
    self.latest_long_control_frame: tuple[dict[str, float], int, int] | None = None

  def reset(self) -> None:
    self.frames.clear()
    self.object_frames.clear()
    self.road_sign_frames.clear()
    self.long_control_frames.clear()
    self.latest_long_control_frame = None

  def update(self, can_packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    if not can_packets:
      return
    for bus, parser in self.parsers.items():
      updated = parser.update(can_packets)
      message_names = self.parser_messages[bus]
      for name in message_names:
        if name in ("UI_driverAssistRoadSign", "DAS_object", "DAS_longControl"):
          continue
        # 0x25D is bus-dependent. Match the control observer and surface the
        # APP traffic-control message only from the verified AP-PARTY source.
        if name == "APP_trafficControl" and bus != 2:
          continue
        message = parser.dbc.name_to_msg[name]
        if message.address not in updated:
          continue
        timestamp = max(parser.ts_nanos[name].values(), default=0)
        previous = self.frames.get(name)
        if timestamp and (previous is None or timestamp >= previous[1]):
          self.frames[name] = (dict(parser.vl[name]), timestamp, bus)

      # UI_driverAssistRoadSign is multiplexed on UI_roadSign (1=stop sign
      # group, 2=traffic light group, 3/4=map/fleet speed groups, 5=spline id).
      if "UI_driverAssistRoadSign" in message_names:
        road_sign_address = parser.dbc.name_to_msg["UI_driverAssistRoadSign"].address
      else:
        road_sign_address = -1
      if road_sign_address in updated:
        all_values = parser.vl_all["UI_driverAssistRoadSign"]
        mux_ids = all_values.get("UI_roadSign", [])
        timestamp = max(parser.ts_nanos["UI_driverAssistRoadSign"].values(), default=0)
        for index, mux_value in enumerate(mux_ids):
          mux_id = int(mux_value)
          if mux_id not in (1, 2, 3, 4, 5):
            continue
          values = {name: samples[index] for name, samples in all_values.items() if index < len(samples)}
          previous = self.road_sign_frames.get(mux_id)
          if timestamp and (previous is None or timestamp >= previous[1]):
            self.road_sign_frames[mux_id] = (values, timestamp, bus)

      if "DAS_object" not in message_names:
        object_address = -1
      else:
        object_address = parser.dbc.name_to_msg["DAS_object"].address
      if object_address not in updated:
        pass
      else:
        all_values = parser.vl_all["DAS_object"]
        object_ids = all_values.get("DAS_objectId", [])
        timestamp = max(parser.ts_nanos["DAS_object"].values(), default=0)
        for index, object_id_value in enumerate(object_ids):
          object_id = int(object_id_value)
          if not 0 <= object_id <= 5:
            continue
          values = {name: samples[index] for name, samples in all_values.items() if index < len(samples)}
          previous = self.object_frames.get(object_id)
          if previous is None or timestamp >= previous[1]:
            self.object_frames[object_id] = (values, timestamp, bus)

      if "DAS_longControl" not in message_names:
        long_control_address = -1
      else:
        long_control_address = parser.dbc.name_to_msg["DAS_longControl"].address
      if long_control_address in updated:
        all_values = parser.vl_all["DAS_longControl"]
        stack_ids = all_values.get("DAS_longControlStack", [])
        timestamp = max(parser.ts_nanos["DAS_longControl"].values(), default=0)
        for index, stack_value in enumerate(stack_ids):
          stack = int(stack_value)
          if not 0 <= stack <= 7:
            continue
          values = {name: samples[index] for name, samples in all_values.items() if index < len(samples)}
          previous = self.long_control_frames.get(stack)
          if timestamp and (previous is None or timestamp >= previous[1]):
            frame = (values, timestamp, bus)
            self.long_control_frames[stack] = frame
            # Samples inside one cereal CAN event share a timestamp. Preserve
            # their wire order so the last multiplexed sample is current.
            self.latest_long_control_frame = frame

  @staticmethod
  def _fresh(frame: tuple[dict[str, float], int, int] | None, now_ns: int, stale_ns: int = FRAME_STALE_NS) -> bool:
    return frame is not None and frame[1] > 0 and 0 <= now_ns - frame[1] <= stale_ns

  def _frame(self, name: str, now_ns: int, stale_ns: int = FRAME_STALE_NS) -> tuple[dict[str, float], int, int] | None:
    frame = self.frames.get(name)
    return frame if self._fresh(frame, now_ns, stale_ns) else None

  def _object_frame(self, mux: int, now_ns: int) -> tuple[dict[str, float], int, int] | None:
    frame = self.object_frames.get(mux)
    return frame if self._fresh(frame, now_ns) else None

  def _long_control_frame(self, mux: int, now_ns: int) -> tuple[dict[str, float], int, int] | None:
    frame = self.long_control_frames.get(mux)
    return frame if self._fresh(frame, now_ns) else None

  def _bus(self, frame: tuple[dict[str, float], int, int] | None) -> str | None:
    return self.bus_names.get(frame[2], str(frame[2])) if frame else None

  def _navigation(self, now_ns: int) -> dict[str, Any]:
    map_frame = self._frame("UI_driverAssistMapData", now_ns, NAV_STALE_NS)
    debug_frame = self._frame("DAS_visualDebug", now_ns, NAV_STALE_NS)
    map_values = map_frame[0] if map_frame else {}
    debug_values = debug_frame[0] if debug_frame else {}
    speed_code = _int(map_values, "UI_mapSpeedLimit")
    speed_unit = "kph" if _int(map_values, "UI_mapSpeedUnits") else "mph"
    branch_distance = float(map_values.get("UI_nextBranchDist", 310.0))
    nav_distance = float(debug_values.get("DAS_navDistance", 25500.0))
    return {
      "available": bool(map_frame or debug_frame),
      "route_active": _bool(map_values, "UI_navRouteActive"),
      "gps_road_match": _bool(map_values, "UI_gpsRoadMatch"),
      "parallel_autopark_enabled": _bool(map_values, "UI_parallelAutoparkEnabled"),
      "perpendicular_autopark_enabled": _bool(map_values, "UI_perpendicularAutoparkEnabled"),
      "nav_available": _bool(debug_values, "DAS_navAvailable"),
      "nav_distance_m": _round(nav_distance, 0) if nav_distance < 25500.0 else None,
      "next_branch_distance_m": _round(branch_distance, 0) if branch_distance < 310.0 else None,
      "next_branch_left_off_ramp": _bool(map_values, "UI_nextBranchLeftOffRamp"),
      "next_branch_right_off_ramp": _bool(map_values, "UI_nextBranchRightOffRamp"),
      "controlled_access": _bool(map_values, "UI_controlledAccess"),
      "road_class": ROAD_CLASSES.get(_int(map_values, "UI_roadClass"), "unknown"),
      "country_code": _int(map_values, "UI_countryCode"),
      "street_count": _int(map_values, "UI_streetCount"),
      "speed_limit": SPEED_LIMIT_VALUES.get(speed_code),
      "speed_limit_unlimited": speed_code == 30,
      "speed_limit_unit": speed_unit,
      "speed_limit_type": SPEED_LIMIT_TYPES.get(_int(map_values, "UI_mapSpeedLimitType"), "unknown"),
      "speed_limit_dependency": _int(map_values, "UI_mapSpeedLimitDependency"),
      "in_supercharger_geofence": _bool(map_values, "UI_inSuperchargerGeofence"),
      "autosteer_navigation_usage": USAGE.get(_int(debug_values, "DAS_autosteerNavigationUsage"), "unknown") if debug_frame else "unknown",
      "reject_navigation": _bool(map_values, "UI_rejectNav"),
      "reject_hpp": _bool(map_values, "UI_rejectHPP"),
      "reject_left_lane": _bool(map_values, "UI_rejectLeftLane"),
      "reject_right_lane": _bool(map_values, "UI_rejectRightLane"),
      "reject_left_free_space": _bool(map_values, "UI_rejectLeftFreeSpace"),
      "reject_right_free_space": _bool(map_values, "UI_rejectRightFreeSpace"),
      "reject_autosteer": _bool(map_values, "UI_rejectAutosteer"),
      "reject_hands_on": _bool(map_values, "UI_rejectHandsOn"),
      "accept_botts_dots": _bool(map_values, "UI_acceptBottsDots"),
      "autosteer_restricted": _bool(map_values, "UI_autosteerRestricted"),
      "pmm_enabled": _bool(map_values, "UI_pmmEnabled"),
      "sca_enabled": _bool(map_values, "UI_scaEnabled"),
      "sources": sorted(filter(None, (self._bus(map_frame), self._bus(debug_frame)))),
    }

  def _lanes(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_lanes", now_ns)
    if not frame:
      return {"available": False, "center": [], "left": [], "right": []}
    values = frame[0]
    view_range = min(max(float(values["DAS_virtualLaneViewRange"]), 0.0), 100.0)
    width = float(values["DAS_virtualLaneWidth"])
    coefficients = [float(values[f"DAS_virtualLaneC{i}"]) for i in range(4)]
    center = []
    left = []
    right = []
    for x in range(0, int(view_range) + 1, 4):
      y = sum(coefficient * x ** power for power, coefficient in enumerate(coefficients))
      center.append([x, _round(y)])
      if _bool(values, "DAS_leftLaneExists"):
        left.append([x, _round(y + width / 2.0)])
      if _bool(values, "DAS_rightLaneExists"):
        right.append([x, _round(y - width / 2.0)])
    return {
      "available": True,
      "bus": self._bus(frame),
      "left_exists": _bool(values, "DAS_leftLaneExists"),
      "right_exists": _bool(values, "DAS_rightLaneExists"),
      "width_m": _round(width),
      "view_range_m": _round(view_range, 0),
      "coefficients": [_round(value, 7) for value in coefficients],
      "left_usage": USAGE.get(_int(values, "DAS_leftLineUsage"), "unknown"),
      "right_usage": USAGE.get(_int(values, "DAS_rightLineUsage"), "unknown"),
      "left_fork": _int(values, "DAS_leftFork"),
      "right_fork": _int(values, "DAS_rightFork"),
      "center": center,
      "left": left,
      "right": right,
    }

  @staticmethod
  def _vehicle(values: dict[str, float], prefix: str, category: str, index: int) -> dict[str, Any] | None:
    suffix = "" if index == 1 else "2"
    dx = float(values.get(f"DAS_{prefix}Veh{suffix}Dx", 127.5))
    track_id = _int(values, f"DAS_{prefix}Veh{suffix}Id")
    id_unavailable = track_id == (127 if index == 1 else 0)
    if dx >= 127.5 or id_unavailable:
      return None
    type_code = _int(values, f"DAS_{prefix}Veh{suffix}Type")
    return {
      "category": category,
      "index": index,
      "track_id": track_id,
      "type": VEHICLE_TYPES.get(type_code, "unknown"),
      "x_m": _round(dx),
      "y_m": _round(values.get(f"DAS_{prefix}Veh{suffix}Dy", 0.0)),
      # T-CAN publishes the scale for VxRel but does not declare a physical unit.
      "relative_speed": _round(values.get(f"DAS_{prefix}Veh{suffix}VxRel", 0.0)),
      "relevant_for_control": _bool(values, f"DAS_{prefix}Veh{suffix}RelevantForControl"),
    }

  def _vehicles(self, now_ns: int) -> tuple[list[dict[str, Any]], dict[str, bool], list[str]]:
    vehicles: list[dict[str, Any]] = []
    sources: set[str] = set()
    headings_frame = self._object_frame(5, now_ns)
    headings = headings_frame[0] if headings_frame else {}
    if headings_frame:
      sources.add(self._bus(headings_frame) or "")
    for mux, prefix, category in ((0, "lead", "lead"), (1, "left", "left"), (2, "right", "right")):
      frame = self._object_frame(mux, now_ns)
      if not frame:
        continue
      sources.add(self._bus(frame) or "")
      for index in (1, 2):
        vehicle = self._vehicle(frame[0], prefix, category, index)
        if vehicle:
          suffix = "" if index == 1 else "2"
          heading_name = f"DAS_{prefix}Veh{suffix}Heading"
          vehicle["heading_rad"] = _round(headings[heading_name], 3) if heading_name in headings else None
          vehicles.append(vehicle)
    cutin_frame = self._object_frame(3, now_ns)
    if cutin_frame:
      sources.add(self._bus(cutin_frame) or "")
      vehicle = self._vehicle(cutin_frame[0], "cutin", "cutin", 1)
      if vehicle:
        vehicle["heading_rad"] = _round(cutin_frame[0].get("DAS_cutinVehHeading", 0.0), 3)
        vehicles.append(vehicle)

    debug_frame = self._frame("DAS_visualDebug", now_ns)
    debug = debug_frame[0] if debug_frame else {}
    left_current = _bool(debug, "DAS_rearLeftVehDetectedCurrent")
    detected_this_cycle = _bool(debug, "DAS_rearVehDetectedThisCycle")
    rear = {
      "detected_this_cycle": detected_this_cycle,
      "left_current": left_current,
      "left_trip": _bool(debug, "DAS_rearLeftVehDetectedTrip"),
      "right_trip": _bool(debug, "DAS_rearRightVehDetectedTrip"),
      # The *_trip bits latch for the whole trip and must never drive a live
      # display. The DBC has no right-side "current" bit, so the right side is
      # inferred from the shared per-cycle bit when the left side is not set.
      "left_live": left_current,
      "right_live": detected_this_cycle and not left_current,
    }
    return vehicles, rear, sorted(source for source in sources if source)

  def _traffic(self, now_ns: int) -> dict[str, Any]:
    control_frame = self._frame("APP_trafficControl", now_ns)
    sign_frame = self._object_frame(4, now_ns)
    control = control_frame[0] if control_frame else {}
    sign = sign_frame[0] if sign_frame else {}

    # A fresh frame is not proof of a traffic light: the OEM broadcasts these
    # messages continuously with idle/SNA values when no light is relevant, so
    # only surface data when the control actually describes a light control.
    feature_code = _int(control, "APP_tcFeatureState")
    control_type_code = _int(control, "APP_tcControlType")
    control_light = bool(control_frame) and control_type_code == 3 and feature_code in (2, 3)
    light_code = _int(control, "APP_tcControlLightState")
    source_code = _int(control, "APP_tcControlSource")
    control_distance = float(control.get("APP_tcControlDistance", 255.0))
    # The ESP32-S3 PARTY capture contains coherent visual light observations
    # while the separate traffic-control feature is disabled. Surface the
    # observation without claiming that Tesla control is available/active.
    light_observation = (bool(control_frame) and control_type_code == 3 and source_code in (1, 2, 3)
                         and light_code in LIGHT_STATES and light_code != 0 and control_distance < 255.0)
    crosswalk_active = bool(control_frame) and control_type_code in (5, 9) and feature_code in (2, 3)
    control_available = control_light or crosswalk_active

    sign_type_code = _int(sign, "DAS_roadSignId")
    sign_valid = bool(sign_frame) and sign_type_code in (0, 1) and _int(sign, "DAS_roadSignSource") != 0

    stop_line_distance = float(sign.get("DAS_roadSignStopLineDist", 184.6))
    return {
      "available": bool(control_available or light_observation or sign_valid),
      "control_available": bool(control_available),
      "light_observation_available": bool(light_observation),
      "road_sign_available": bool(sign_valid),
      "control_frame_fresh": bool(control_frame),
      "sign_frame_fresh": bool(sign_frame),
      "feature_state": TRAFFIC_FEATURE_STATES.get(feature_code, "unknown"),
      "feature_state_code": feature_code,
      "state_machine": TRAFFIC_MACHINE_STATES.get(_int(control, "APP_tcStateMachine"), "unknown"),
      "state_machine_code": _int(control, "APP_tcStateMachine"),
      "control_source": TRAFFIC_SOURCES.get(_int(control, "APP_tcControlSource"), "unknown"),
      "control_source_code": source_code,
      "control_type": TRAFFIC_TYPES.get(control_type_code, "unknown"),
      "control_type_code": control_type_code,
      "control_distance_m": _round(control_distance) if control_available or light_observation else None,
      "light_state": LIGHT_STATES.get(light_code, "unknown") if control_available or light_observation else "unknown",
      "continuation_reason": _int(control, "APP_tcContinuationReason"),
      "confirmation_type": _int(control, "APP_tcConfirmationType"),
      "warning_suppression_reason": _int(control, "APP_tcWarningSuppressionReason"),
      "unavailable_reason": _int(control, "APP_tcUnavailableReason"),
      "vision_light": _bool(control, "APP_tcVisionLight"),
      "vision_sign": _bool(control, "APP_tcVisionSign"),
      "vision_road_marking": _bool(control, "APP_tcVisionRoadMarking"),
      "vision_line": _bool(control, "APP_tcVisionLine"),
      "road_sign_type": ROAD_SIGN_TYPES.get(sign_type_code, "unknown") if sign_valid else "unknown",
      "road_sign_color": ROAD_SIGN_COLORS.get(_int(sign, "DAS_roadSignColor"), "unknown") if sign_valid else "unknown",
      "stop_line_distance_m": _round(stop_line_distance) if sign_valid and stop_line_distance < 184.6 else None,
      "road_sign_active": _bool(sign, "DAS_roadSignControlActive"),
      "road_sign_source": ROAD_SIGN_SOURCES.get(_int(sign, "DAS_roadSignSource"), "unknown") if sign_valid else "unknown",
      "road_sign_arrow": ROAD_SIGN_ARROWS.get(_int(sign, "DAS_roadSignArrow"), "unknown") if sign_valid else "unknown",
      "road_sign_orientation": _int(sign, "DAS_roadSignOrientation"),
      "sources": sorted(filter(None, (self._bus(control_frame), self._bus(sign_frame)))),
    }

  def _driver_assist(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_visualDebug", now_ns)
    values = frame[0] if frame else {}
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "planner_state": PLANNER_STATES.get(_int(values, "DAS_plannerState"), "unknown"),
      "behavior": BEHAVIOR_TYPES.get(_int(values, "DAS_behaviorType"), "unknown"),
      "health": HEALTH_STATES.get(_int(values, "DAS_autosteerHealthState"), "unknown"),
      "health_anomaly_level": _int(values, "DAS_autosteerHealthAnomalyLevel"),
      "road_surface": ROAD_SURFACES.get(_int(values, "DAS_roadSurfaceType"), "unknown"),
      "vehicles_usage": USAGE.get(_int(values, "DAS_autosteerVehiclesUsage"), "unknown"),
      "hpp_usage": USAGE.get(_int(values, "DAS_autosteerHPPUsage"), "unknown"),
      "model_usage": USAGE.get(_int(values, "DAS_autosteerModelUsage"), "unknown"),
      "botts_dots_usage": USAGE.get(_int(values, "DAS_autosteerBottsDotsUsage"), "unknown"),
      "offset_side": _int(values, "DAS_offsetSide"),
      "last_line_preference_reason": _int(values, "DAS_lastLinePreferenceReason"),
      "last_abort_reason": _int(values, "DAS_lastAutosteerAbortReason"),
      "developer_app_interface_enabled": _bool(values, "DAS_devAppInterfaceEnabled"),
      "smart_speed_active": _bool(values, "DAS_accSmartSpeedActive"),
      "smart_speed_state": _int(values, "DAS_accSmartSpeedState"),
      "traffic_aware_set_speed": _bool(values, "DAS_trafficAwareSetSpeedInUse"),
      "isa_state": _int(values, "DAS_isaSystemState"),
      "ulc_in_progress": _bool(values, "DAS_ulcInProgress"),
      "ulc_type": _int(values, "DAS_ulcType"),
    }

  def _road_sign_mux_frame(self, mux: int, now_ns: int) -> tuple[dict[str, float], int, int] | None:
    frame = self.road_sign_frames.get(mux)
    return frame if self._fresh(frame, now_ns) else None

  def _road_sign(self, now_ns: int) -> dict[str, Any]:
    any_frame = None
    for frame in self.road_sign_frames.values():
      if self._fresh(frame, now_ns) and (any_frame is None or frame[1] >= any_frame[1]):
        any_frame = frame
    stop_frame = self._road_sign_mux_frame(1, now_ns)
    light_frame = self._road_sign_mux_frame(2, now_ns)
    map_frame = self._road_sign_mux_frame(3, now_ns)
    fleet_frame = self._road_sign_mux_frame(4, now_ns)
    values = any_frame[0] if any_frame else {}

    def _stop_line(frame, name):
      if not frame:
        return None
      value = float(frame[0].get(name, -8.0))
      return _round(value) if 0.0 <= value < 200.0 else None

    def _confidence(frame, name):
      if not frame:
        return None
      confidence = _int(frame[0], name)
      return confidence if 0 < confidence < 127 else None

    def _speed(frame, name):
      if not frame:
        return None
      value = float(frame[0].get(name, 0.0))
      return _round(value) if value > 0.0 else None

    return {
      "available": bool(any_frame),
      "bus": self._bus(any_frame),
      "road_sign_id": _int(values, "UI_roadSign") if any_frame else None,
      "stop_sign_stop_line_distance_m": _stop_line(stop_frame, "UI_stopSignStopLineDist"),
      "stop_sign_stop_line_confidence": _confidence(stop_frame, "UI_stopSignStopLineConf"),
      "traffic_light_stop_line_distance_m": _stop_line(light_frame, "UI_trafficLightStopLineDist"),
      "traffic_light_stop_line_confidence": _confidence(light_frame, "UI_trafficLightStopLineConf"),
      "base_map_speed_limit_mps": _speed(map_frame, "UI_baseMapSpeedLimitMPS"),
      "bottom_quartile_fleet_speed_mps": _speed(map_frame, "UI_bottomQrtlFleetSpeedMPS"),
      "top_quartile_fleet_speed_mps": _speed(map_frame, "UI_topQrtlFleetSpeedMPS"),
      "mean_fleet_spline_speed_mps": _speed(fleet_frame, "UI_meanFleetSplineSpeedMPS"),
      "median_fleet_spline_speed_mps": _speed(fleet_frame, "UI_medianFleetSplineSpeedMPS"),
      "ramp_type": _int(fleet_frame[0], "UI_rampType") if fleet_frame else None,
      "spline_location_confidence": _int(values, "UI_splineLocConfidence") if any_frame else None,
    }

  def _pedestrian_detection(self, now_ns: int, positioned_objects: list[dict[str, Any]], collision_warning: bool) -> dict[str, Any]:
    frame = self._frame("APP_pedestrianDetection", now_ns, PEDESTRIAN_STALE_NS)
    values = frame[0] if frame else {}
    flags = {
      "front_main": _bool(values, "APP_pedestrianDetectedFrontMain"),
      "front_fisheye": _bool(values, "APP_pedestrianDetectedFrontFisheye"),
      "front_narrow": _bool(values, "APP_pedestrianDetectedFrontNarrow"),
      "left_pillar": _bool(values, "APP_pedestrianDetectedLeftPillar"),
      "left_repeater": _bool(values, "APP_pedestrianDetectedLeftRepeater"),
      "right_pillar": _bool(values, "APP_pedestrianDetectedRightPillar"),
      "right_repeater": _bool(values, "APP_pedestrianDetectedRightRepeater"),
      "backup": _bool(values, "APP_pedestrianDetectedBackup"),
    }
    detected_any = bool(frame) and any(flags.values())
    active_cameras = [name for name, active in flags.items() if active]
    camera_mask = sum((1 << index) for index, active in enumerate(flags.values()) if active)
    front_detected = any(flags[name] for name in ("front_main", "front_fisheye", "front_narrow"))
    coordinate_slots = []
    if detected_any:
      for index in (1, 2, 3):
        # T-CAN documents signed 6-bit values with scale 0.4. Vehicle-position
        # validation supports metres, but no per-slot valid bit, confidence,
        # track id, or camera-to-slot mapping is documented.
        coordinate_slots.append({
          "index": index,
          "dx_scaled": _round(values.get(f"APP_closestPedestrian{index}dX", 0.0)),
          "dy_scaled": _round(values.get(f"APP_closestPedestrian{index}dY", 0.0)),
          "validity": "unknown",
        })
    evidence_tier = ("collision_warning" if collision_warning else
                     "positioned_object" if positioned_objects else
                     "camera_detection" if detected_any else "none")
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      **flags,
      "detected_any": detected_any,
      "active_cameras": active_cameras,
      "camera_mask": camera_mask,
      "simultaneous_front_rear": bool(front_detected and flags["backup"]),
      "collision_warning": collision_warning,
      "evidence_present": bool(detected_any or positioned_objects or collision_warning),
      "evidence_tier": evidence_tier,
      "position_available": bool(positioned_objects),
      "positioned_objects": positioned_objects,
      "coordinate_slots": coordinate_slots,
      "coordinate_unit": "m",
      "coordinate_validity": "undocumented",
    }

  def _blind_spot(self, now_ns: int) -> dict[str, Any]:
    frames = []
    for name in ("DAS_status", "DAS_statusCH"):
      frame = self._frame(name, now_ns)
      if frame is not None:
        frames.append(frame)
    frame = max(frames, key=lambda item: item[1]) if frames else None
    values = frame[0] if frame else {}
    left = _int(values, "DAS_blindSpotRearLeft")
    right = _int(values, "DAS_blindSpotRearRight")
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "sources": sorted(source for source in (self._bus(item) for item in frames) if source),
      "left_level": left if frame else None,
      "right_level": right if frame else None,
      "left_live": left in (1, 2) if frame else False,
      "right_live": right in (1, 2) if frame else False,
      "side_collision_avoid_level": _int(values, "DAS_sideCollisionAvoid") if frame else None,
      "side_collision_warning_level": _int(values, "DAS_sideCollisionWarning") if frame else None,
      "side_collision_inhibit": _bool(values, "DAS_sideCollisionInhibit") if frame else False,
      "forward_collision_warning_level": _int(values, "DAS_forwardCollisionWarning") if frame else None,
      "lane_departure_warning_level": _int(values, "DAS_laneDepartureWarning") if frame else None,
      "autopilot_state": _int(values, "DAS_autopilotState") if frame else None,
      "fused_speed_limit_kph": _round(float(values.get("DAS_fusedSpeedLimit", 0.0))) if frame else None,
    }

  def _front_safety(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_integratedSafetyFront", now_ns)
    values = frame[0] if frame else {}
    distance = float(values.get("DAS_targetDistanceFront", 0.0))
    distance_valid = bool(frame) and _bool(values, "DAS_targetDistanceFrontQF")
    target_present = distance_valid and 0.0 < distance < 12.7
    relative_velocity = float(values.get("DAS_relativeVelocityFront", 0.0))
    relative_accel = float(values.get("DAS_relativeAccelerationFront", 0.0))
    time_to_impact = float(values.get("DAS_timeToImpactFront", 0.0))
    impact_velocity = float(values.get("DAS_predictedImpactVelFront", 0.0))
    impact_overlap = float(values.get("DAS_predictedImpactOvrlapFront", 0.0))
    tti_valid = target_present and _bool(values, "DAS_timeToImpactFrontQF") and 0.0 < time_to_impact < 511.0
    velocity_valid = target_present and _bool(values, "DAS_relativeVelocityFrontQF") and relative_velocity > -32.0
    accel_valid = target_present and _bool(values, "DAS_relativeAccelerationFrontQF") and relative_accel > -12.8
    impact_velocity_valid = target_present and _bool(values, "DAS_predictedImpactVelFrontQF")
    impact_overlap_valid = target_present and _bool(values, "DAS_predictedImpactOvrlapFrontQF")
    imminent_valid = bool(frame) and _bool(values, "DAS_imminentCollisionFrontQF")
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "valid_target": target_present,
      "target_distance_m": _round(distance) if target_present else None,
      "target_distance_quality": distance_valid,
      "relative_velocity_mps": _round(relative_velocity) if velocity_valid else None,
      "relative_velocity_quality": _bool(values, "DAS_relativeVelocityFrontQF") if frame else False,
      "relative_acceleration_mps2": _round(relative_accel) if accel_valid else None,
      "time_to_impact_s": _round(time_to_impact) if tti_valid else None,
      "imminent_collision": imminent_valid and _bool(values, "DAS_imminentCollisionFront"),
      "predicted_impact_velocity_mps": _round(impact_velocity) if impact_velocity_valid else None,
      "predicted_impact_overlap_pct": _round(impact_overlap) if impact_overlap_valid else None,
      "idf_enabled": _bool(values, "DAS_idfEnableFlag") if frame else False,
    }

  def _longitudinal_shadow(self, now_ns: int) -> dict[str, Any]:
    latest = self.latest_long_control_frame if self._fresh(self.latest_long_control_frame, now_ns) else None
    latest_values = latest[0] if latest else {}
    current_stack = _int(latest_values, "DAS_longControlStack") if latest else None

    torque_profile = self._long_control_frame(2, now_ns)
    velocity_profile = self._long_control_frame(3, now_ns)
    aeb = self._long_control_frame(4, now_ns)
    pedal = self._long_control_frame(5, now_ns)
    torque = self._long_control_frame(6, now_ns)
    tp = torque_profile[0] if torque_profile else {}
    vp = velocity_profile[0] if velocity_profile else {}
    aeb_values = aeb[0] if aeb else {}
    pedal_values = pedal[0] if pedal else {}
    torque_values = torque[0] if torque else {}
    aeb_state = _int(aeb_values, "DAS_aebControl_active") if aeb else None

    return {
      "available": bool(latest),
      "bus": self._bus(latest),
      "read_only": True,
      "current_stack": LONG_CONTROL_STACKS.get(current_stack, "unknown") if current_stack is not None else None,
      "current_stack_code": current_stack,
      "gear_request": _int(latest_values, "DAS_gearRequest") if latest else None,
      "torque_profiler": {
        "available": bool(torque_profile),
        "accel_min_mps2": _round(tp["DAS_torqueProfiler_accelMinPed"]) if "DAS_torqueProfiler_accelMinPed" in tp else None,
        "accel_max_mps2": _round(tp["DAS_torqueProfiler_accelMaxPed"]) if "DAS_torqueProfiler_accelMaxPed" in tp else None,
        "target_speed_kph": _round(tp["DAS_torqueProfiler_targetSpeedPed"]) if "DAS_torqueProfiler_targetSpeedPed" in tp else None,
      },
      "velocity_profile": {
        "available": bool(velocity_profile),
        "accel_mps2": _round(vp["DAS_velocityProfile_accelFwd_t0"]) if "DAS_velocityProfile_accelFwd_t0" in vp else None,
        "future_target_speed_kph": _round(vp["DAS_velocityProfile_futureTargetSpeedFwd"]) if "DAS_velocityProfile_futureTargetSpeedFwd" in vp else None,
        "calculation_delay_s": _round(vp["DAS_velocityProfile_calcDelay"], 3) if "DAS_velocityProfile_calcDelay" in vp else None,
      },
      "aeb": {
        "available": bool(aeb) and aeb_state in (0, 1, 2),
        "state": aeb_state,
        "active": aeb_state == 2,
        "target_accel_mps2": _round(aeb_values["DAS_aebControl_targetAccelDis"]) if aeb and aeb_state in (0, 1, 2) else None,
      },
      "pedal_control": {
        "available": bool(pedal),
        "accelerator_pct": _round(pedal_values["DAS_pedalControl_accelPedalPos"]) if "DAS_pedalControl_accelPedalPos" in pedal_values else None,
        "brake_torque_nm": _round(pedal_values["DAS_pedalControl_brakeTorqueCommand"]) if "DAS_pedalControl_brakeTorqueCommand" in pedal_values else None,
      },
      "torque_control": {
        "available": bool(torque),
        "system_torque_nm": (_round(torque_values["DAS_torqueControl_sysTorqueCommandFwd"])
                             if "DAS_torqueControl_sysTorqueCommandFwd" in torque_values else None),
        "standstill_request": _bool(torque_values, "DAS_torqueControl_standstillRequest") if torque else False,
      },
    }

  def _proximity_safety(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_status2", now_ns)
    values = frame[0] if frame else {}
    severity = _int(values, "DAS_pmmObstacleSeverity") if frame else None
    long_warning = _int(values, "DAS_longCollisionWarning") if frame else None
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "read_only": True,
      "obstacle_severity": severity if severity is not None and severity < 7 else None,
      "long_collision_warning": long_warning if long_warning is not None and long_warning < 15 else None,
      "ultrasonics_fault_reason": _int(values, "DAS_pmmUltrasonicsFaultReason") if frame else None,
      "radar_fault_reason": _int(values, "DAS_pmmRadarFaultReason") if frame else None,
      "camera_fault_reason": _int(values, "DAS_pmmCameraFaultReason") if frame else None,
      "system_fault_reason": _int(values, "DAS_pmmSysFaultReason") if frame else None,
      "activation_failure_status": _int(values, "DAS_activationFailureStatus") if frame else None,
    }

  def _parking_obstacle(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("PARK_oocStatus", now_ns)
    values = frame[0] if frame else {}
    distance_cm = float(values.get("PARK_oocDistance", 511.0))
    confidence = _int(values, "PARK_oocConfidence")
    x_cm = float(values.get("PARK_oocVehicleX", 394.0))
    y_cm = float(values.get("PARK_oocVehicleY", 126.0))
    valid = bool(frame) and 0.0 <= distance_cm < 500.0 and 0 < confidence < 127
    side_code = _int(values, "PARK_oocCollisionSide") if frame else None
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "valid_obstacle": valid,
      "distance_m": _round(distance_cm / 100.0) if valid else None,
      "confidence": confidence if valid else None,
      "x_m": _round(x_cm / 100.0) if valid and x_cm != 394.0 else None,
      "y_m": _round(y_cm / 100.0) if valid and y_cm != 126.0 else None,
      "collision_side": COLLISION_SIDES.get(side_code, "unknown") if valid else None,
      "direct_echo_only": _bool(values, "PARK_oocDirectEchoOnly") if valid else False,
      "untracked_time_s": _round(values["PARK_oocUntrackedTime"]) if valid else None,
    }

  def snapshot(self, now_ns: int | None = None) -> dict[str, Any]:
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    navigation = self._navigation(now_ns)
    lanes = self._lanes(now_ns)
    vehicles, rear, vehicle_sources = self._vehicles(now_ns)
    traffic = self._traffic(now_ns)
    driver_assist = self._driver_assist(now_ns)
    road_sign = self._road_sign(now_ns)
    blind_spot = self._blind_spot(now_ns)
    front_safety = self._front_safety(now_ns)
    longitudinal_shadow = self._longitudinal_shadow(now_ns)
    proximity_safety = self._proximity_safety(now_ns)
    parking_obstacle = self._parking_obstacle(now_ns)
    pedestrians = [vehicle for vehicle in vehicles if vehicle["type"] == "pedestrian"]
    pedestrian_detection = self._pedestrian_detection(
      now_ns, pedestrians, proximity_safety.get("long_collision_warning") == 2,
    )
    buses = sorted({
      *(navigation.get("sources") or []),
      *(traffic.get("sources") or []),
      *vehicle_sources,
      *([lanes["bus"]] if lanes.get("bus") else []),
      *([driver_assist["bus"]] if driver_assist.get("bus") else []),
      *([road_sign["bus"]] if road_sign.get("bus") else []),
      *([pedestrian_detection["bus"]] if pedestrian_detection.get("bus") else []),
      *([blind_spot["bus"]] if blind_spot.get("bus") else []),
      *([front_safety["bus"]] if front_safety.get("bus") else []),
      *([longitudinal_shadow["bus"]] if longitudinal_shadow.get("bus") else []),
      *([proximity_safety["bus"]] if proximity_safety.get("bus") else []),
      *([parking_obstacle["bus"]] if parking_obstacle.get("bus") else []),
    })
    return {
      "available": bool(navigation["available"] or lanes["available"] or vehicles or traffic["available"] or driver_assist["available"]
                        or road_sign["available"] or pedestrian_detection["available"] or blind_spot["available"] or front_safety["available"]
                        or longitudinal_shadow["available"] or proximity_safety["available"] or parking_obstacle["available"]),
      "dbc": DBC_NAME,
      "buses": buses,
      "capabilities": {
        "read_only": True,
        "ch_bus_configured": self.ch_bus is not None,
        "oem_object_list_available": bool(self.object_frames) and self.ch_bus is not None,
        "control_integration_enabled": False,
      },
      "navigation": navigation,
      "lanes": lanes,
      "vehicles": vehicles,
      "rear_vehicles": rear,
      "traffic": traffic,
      "driver_assist": driver_assist,
      "road_sign": road_sign,
      "pedestrian_detection": pedestrian_detection,
      "blind_spot": blind_spot,
      "front_safety": front_safety,
      "longitudinal_shadow": longitudinal_shadow,
      "proximity_safety": proximity_safety,
      "parking_obstacle": parking_obstacle,
      "pedestrians": pedestrians,
      "cyclists": [vehicle for vehicle in vehicles if vehicle["type"] in ("bicycle", "motorcycle")],
    }
