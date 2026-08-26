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
  "APP_roadDisturbance",
  "BMS_hvBusStatus",
  "BMS_status",
  "VCSEC_TPMSData",
  "VCSEC_TPMSDisplay",
  "TPMS_data",
  "DIR_power",
  "DIF_power",
  "DIR_temperature",
  "DIF_temperature",
  "DI_odometerStatus",
  "BMS_kwhCounter",
  "DI_estimatedBrakeTemp",
  "UI_ambientLightingCtrls",
)
VEH_MUX_MESSAGES = {
  "VCSEC_TPMSData": "VCSEC_TPMSDataIndex",
  "DIR_temperature": "DIR_tempIndex",
  "DIF_temperature": "DIF_tempIndex",
}
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
VEH_DIAGNOSTIC_STALE_NS = 5_000_000_000

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
BMS_CONTACTOR_STATES = {0: "sna", 1: "open", 2: "opening", 3: "closing", 4: "closed", 5: "welded", 6: "blocked"}
BMS_CHARGE_STATES = {
  0: "disconnected", 1: "no_power", 2: "about_to_charge", 3: "charging", 4: "charge_complete",
  5: "charge_stopped", 6: "calibrating",
}
BMS_STATES = {
  0: "standby", 1: "drive", 2: "support", 3: "charge", 4: "feim", 5: "clear_fault",
  6: "fault", 7: "weld", 8: "test", 9: "sna", 10: "diagnostic",
}
BMS_HV_STATES = {0: "down", 1: "coming_up", 2: "going_down", 3: "up_for_drive", 4: "up_for_charge", 5: "up_for_dc_charge", 6: "up"}
TPMS_LOCATIONS = {0: "front_left", 1: "front_right", 2: "rear_left", 3: "rear_right", 4: "unknown"}
TPMS_TELLTALES = {0: "off", 1: "solid", 2: "flashing"}
TPMS_FEATURE_STATES = {0: "not_supported", 1: "unavailable_1", 2: "unavailable_2", 3: "wait_for_stationary", 4: "ready", 5: "active", 6: "blocked"}
TPMS_PROXIMITY_STATES = {0: "none", 1: "reached", 2: "incorrect", 3: "far_away", 4: "medium", 5: "close"}
INVERTER_QUALITY = {0: "initializing", 1: "irrational", 2: "rational", 3: "unknown"}
AMBIENT_ENABLE_STATES = {0: "off", 1: "on", 2: "auto"}
PARK_OBSTACLE_HEIGHTS = {0: "no_object", 1: "low", 2: "high", 3: "unknown"}


def _round(value: float, digits: int = 2) -> float:
  return round(float(value), digits)


def _int(values: dict[str, float], name: str) -> int:
  return int(values.get(name, 0.0))


def _bool(values: dict[str, float], name: str) -> bool:
  return bool(_int(values, name))


def _value(values: dict[str, float], name: str, invalid: tuple[float, ...] = ()) -> float | None:
  if name not in values:
    return None
  value = float(values[name])
  return None if any(math.isclose(value, sentinel, abs_tol=1e-6) for sentinel in invalid) else value


def _measurement(values: dict[str, float], name: str, invalid: tuple[float, ...] = (), digits: int = 2) -> float | None:
  value = _value(values, name, invalid)
  return _round(value, digits) if value is not None else None


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
    self.veh_mux_frames: dict[str, dict[int, tuple[dict[str, float], int, int]]] = {
      name: {} for name in VEH_MUX_MESSAGES
    }
    self.latest_long_control_frame: tuple[dict[str, float], int, int] | None = None

  def reset(self) -> None:
    self.frames.clear()
    self.object_frames.clear()
    self.road_sign_frames.clear()
    self.long_control_frames.clear()
    for frames in self.veh_mux_frames.values():
      frames.clear()
    self.latest_long_control_frame = None

  def update(self, can_packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    if not can_packets:
      return
    for bus, parser in self.parsers.items():
      updated = parser.update(can_packets)
      message_names = self.parser_messages[bus]
      for name in message_names:
        if name in ("UI_driverAssistRoadSign", "DAS_object", "DAS_longControl") or name in VEH_MUX_MESSAGES:
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

      for name, mux_name in VEH_MUX_MESSAGES.items():
        if name not in message_names:
          continue
        address = parser.dbc.name_to_msg[name].address
        if address not in updated:
          continue
        all_values = parser.vl_all[name]
        mux_values = all_values.get(mux_name, [])
        timestamp = max(parser.ts_nanos[name].values(), default=0)
        for index, mux_value in enumerate(mux_values):
          mux = int(mux_value)
          values = {signal: samples[index] for signal, samples in all_values.items() if index < len(samples)}
          previous = self.veh_mux_frames[name].get(mux)
          if timestamp and (previous is None or timestamp >= previous[1]):
            self.veh_mux_frames[name][mux] = (values, timestamp, bus)

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

  def _veh_mux_frame(self, name: str, mux: int, now_ns: int) -> tuple[dict[str, float], int, int] | None:
    frame = self.veh_mux_frames.get(name, {}).get(mux)
    return frame if self._fresh(frame, now_ns, VEH_DIAGNOSTIC_STALE_NS) else None

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

  def _road_disturbance(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("APP_roadDisturbance", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    values = frame[0] if frame else {}
    x0 = _measurement(values, "APP_roadDisturbanceX0")
    x1 = _measurement(values, "APP_roadDisturbanceX1")
    y0 = _measurement(values, "APP_roadDisturbanceY0")
    y1 = _measurement(values, "APP_roadDisturbanceY1")
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "index": _int(values, "APP_roadDisturbanceIndex") if frame else None,
      "height_m": _measurement(values, "APP_roadDisturbanceHeight"),
      "x0_m": x0,
      "x1_m": x1,
      "y0_m": y0,
      "y1_m": y1,
      "longitudinal_span_m": _round(abs(x1 - x0)) if x0 is not None and x1 is not None else None,
      "lateral_span_m": _round(abs(y1 - y0)) if y0 is not None and y1 is not None else None,
      "suspension_level_request": _int(values, "APP_suspensionLevelRequest") if frame else None,
    }

  def _battery_diagnostics(self, now_ns: int) -> dict[str, Any]:
    bus_frame = self._frame("BMS_hvBusStatus", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    status_frame = self._frame("BMS_status", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    bus_values = bus_frame[0] if bus_frame else {}
    status = status_frame[0] if status_frame else {}
    contactor_code = _int(status, "BMS_contactorState") if status_frame else None
    charge_code = _int(status, "BMS_userChargeStatus") if status_frame else None
    state_code = _int(status, "BMS_state") if status_frame else None
    requested_state_code = _int(status, "BMS_smStateRequest") if status_frame else None
    hv_code = _int(status, "BMS_hvState") if status_frame else None
    return {
      "available": bool(bus_frame or status_frame),
      "sources": sorted(filter(None, (self._bus(bus_frame), self._bus(status_frame)))),
      "dc_link_voltage_v": _measurement(bus_values, "BMS_dcLinkVoltage"),
      "pack_current_a": _measurement(bus_values, "BMS_packCurrent", (-3276.8,)),
      "current_unfiltered_a": _measurement(bus_values, "BMS_currentUnfiltered", (-2460.4,)),
      "contactor_state": BMS_CONTACTOR_STATES.get(contactor_code, "unknown") if contactor_code is not None else None,
      "contactor_state_code": contactor_code,
      "charge_status": BMS_CHARGE_STATES.get(charge_code, "unknown") if charge_code is not None else None,
      "charge_status_code": charge_code,
      "state": BMS_STATES.get(state_code, "unknown") if state_code is not None else None,
      "state_code": state_code,
      "requested_state": BMS_STATES.get(requested_state_code, "unknown") if requested_state_code is not None else None,
      "requested_state_code": requested_state_code,
      "hv_state": BMS_HV_STATES.get(hv_code, "unknown") if hv_code is not None else None,
      "hv_state_code": hv_code,
      "battery_input_power_kw": _measurement(status, "BMS_batteryInputPower", (3276.75,)),
      "charge_power_available_kw": _measurement(status, "BMS_chgPowerAvailable", (511.875,)),
      "hvac_power_request": _bool(status, "BMS_hvacPowerRequest") if status_frame else False,
      "not_enough_power_for_drive": _bool(status, "BMS_notEnoughPowerForDrive") if status_frame else False,
      "not_enough_power_for_support": _bool(status, "BMS_notEnoughPowerForSupport") if status_frame else False,
      "precondition_allowed": _bool(status, "BMS_preconditionAllowed") if status_frame else False,
      "update_allowed": _bool(status, "BMS_updateAllowed") if status_frame else False,
      "charge_port_missing_on_hv_system": _bool(status, "BMS_cpMiaOnHvs") if status_frame else False,
      "charge_request": _bool(status, "BMS_chargeRequest") if status_frame else False,
      "conditioning_request": _bool(status, "BMS_conditioningRequest") if status_frame else False,
      "pcs_pwm_enabled": _bool(status, "BMS_pcsPwmEnabled") if status_frame else False,
      "charge_retry_count": _int(status, "BMS_chargeRetryCount") if status_frame else None,
      "limp_request": _bool(status, "BMS_diLimpRequest") if status_frame else False,
      "ok_to_ship_by_air": _bool(status, "BMS_okToShipByAir") if status_frame else False,
      "ok_to_ship_by_land": _bool(status, "BMS_okToShipByLand") if status_frame else False,
    }

  def _tpms_diagnostics(self, now_ns: int) -> dict[str, Any]:
    display_frame = self._frame("VCSEC_TPMSDisplay", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    direct_frame = self._frame("TPMS_data", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    display = display_frame[0] if display_frame else {}
    direct = direct_frame[0] if direct_frame else {}
    sensors = []
    for index in range(4):
      frame = self._veh_mux_frame("VCSEC_TPMSData", index, now_ns)
      if not frame:
        continue
      values = frame[0]
      location_code = _int(values, f"VCSEC_TPMSLocation{index}")
      sensors.append({
        "sensor_index": index,
        "location": TPMS_LOCATIONS.get(location_code, "unknown"),
        "location_code": location_code,
        "pressure_bar": _measurement(values, f"VCSEC_TPMSPressure{index}", (12.75, 12.775)),
        "temperature_c": _measurement(values, f"VCSEC_TPMSTemperature{index}", (215.0,)),
        "temperature_compensated_pressure_bar": _measurement(values, f"VCSEC_TPMSTemperatureCompensatedPressure{index}", (12.75, 12.775)),
        "pressure_rate": _measurement(values, f"VCSEC_TPMSPressureRateOfChange{index}"),
        "battery_voltage_v": _measurement(values, f"VCSEC_TPMSBatVoltage{index}", (4.05,)),
        "pressure_in_advertisement": _bool(values, f"VCSEC_TPMSCapabilityPressureInAdv{index}"),
        "configurable_pressure": _bool(values, f"VCSEC_TPMSCapabilityConfigurablePressure{index}"),
      })

    feature_frame = self._veh_mux_frame("VCSEC_TPMSData", 4, now_ns)
    autonomy_frame = self._veh_mux_frame("VCSEC_TPMSData", 5, now_ns)
    feature = feature_frame[0] if feature_frame else {}
    autonomy = autonomy_frame[0] if autonomy_frame else {}
    telltale_code = _int(display, "VCSEC_TPMSTellTale") if display_frame else None
    wheels = {}
    for suffix, key in (("FL", "front_left"), ("FR", "front_right"), ("RL", "rear_left"), ("RR", "rear_right")):
      wheels[key] = {
        "display_pressure_bar": _measurement(display, f"VCSEC_TPMSDisplayPressure{suffix}", (6.35, 6.375)),
        "direct_pressure_bar": _measurement(direct, f"TPMS_pressure{suffix}"),
        "direct_temperature_c": _measurement(direct, f"TPMS_temperature{suffix}"),
        "last_known_pressure_bar": _measurement(autonomy, f"VCSEC_TPMSLastKnownPressure{suffix}", (6.35, 6.375)),
        "soft_warning": _bool(display, f"VCSEC_TPMSDisplaySoftWarningIndication{suffix}") if display_frame else False,
        "hard_warning": _bool(display, f"VCSEC_TPMSDisplayHardWarningIndication{suffix}") if display_frame else False,
      }
    feature0 = _int(feature, "VCSEC_TPMSFeature0") if feature_frame else None
    feature1 = _int(feature, "VCSEC_TPMSFeature1") if feature_frame else None
    autonomy_code = _int(autonomy, "VCSEC_TPMSAutonomyStatus") if autonomy_frame else None
    sources = {self._bus(frame) for frame in (display_frame, direct_frame, feature_frame, autonomy_frame) if frame}
    sources.update(self._bus(self._veh_mux_frame("VCSEC_TPMSData", index, now_ns)) for index in range(4))
    return {
      "available": bool(display_frame or direct_frame or sensors or feature_frame or autonomy_frame),
      "sources": sorted(source for source in sources if source),
      "telltale": TPMS_TELLTALES.get(telltale_code, "unknown") if telltale_code is not None else None,
      "telltale_code": telltale_code,
      "wheels": wheels,
      "sensors": sensors,
      "recommended_cold_pressure_front_bar": _measurement(feature, "VCSEC_TPMSRecommendedColdPressureFront", (6.35, 6.375)),
      "recommended_cold_pressure_rear_bar": _measurement(feature, "VCSEC_TPMSRecommendedColdPressureRear", (6.35, 6.375)),
      "feature_state": TPMS_FEATURE_STATES.get(feature0, "unknown") if feature0 is not None else None,
      "feature_state_code": feature0,
      "proximity_state": TPMS_PROXIMITY_STATES.get(feature1, "unknown") if feature1 is not None else None,
      "proximity_state_code": feature1,
      "feature_count": _int(feature, "VCSEC_TPMSFeature0Count") if feature_frame else None,
      "feature_time_s": _measurement(feature, "VCSEC_TPMSFeature0TimeS"),
      "autonomy_status": {0: "normal", 1: "mia", 2: "reset"}.get(autonomy_code, "unknown") if autonomy_code is not None else None,
      "autonomy_status_code": autonomy_code,
      "autonomy_mia_time_s": _measurement(autonomy, "VCSEC_TPMSAutonomyStatusMIATimeS"),
    }

  def _drive_power(self, now_ns: int) -> dict[str, Any]:
    result: dict[str, Any] = {"available": False, "units_inferred": True}
    sources = set()
    for message, key, prefix in (("DIR_power", "rear", "DIR"), ("DIF_power", "front", "DIF")):
      frame = self._frame(message, now_ns, VEH_DIAGNOSTIC_STALE_NS)
      values = frame[0] if frame else {}
      if frame:
        result["available"] = True
        sources.add(self._bus(frame))
      result[key] = {
        "available": bool(frame),
        "electrical_power_kw": _measurement(values, f"{prefix}_elecPower", (-512.0,)),
        "heat_power_optimal_kw": _measurement(values, f"{prefix}_heatPowerOptimal"),
        "heat_power_max_kw": _measurement(values, f"{prefix}_heatPowerMax"),
        "heat_power_actual_kw": _measurement(values, f"{prefix}_heatPowerActual"),
        "excess_heat_command_kw": _measurement(values, f"{prefix}_excessHeatCommand"),
        "drive_power_max_kw": _measurement(values, f"{prefix}_drivePowerMax", (511.0,)),
      }
    result["sources"] = sorted(source for source in sources if source)
    return result

  def _drive_temperature_side(self, message: str, prefix: str, now_ns: int) -> dict[str, Any]:
    pages = {mux: self._veh_mux_frame(message, mux, now_ns) for mux in range(5)}
    values = {mux: (frame[0] if frame else {}) for mux, frame in pages.items()}
    p0, p1, p2, p3, p4 = (values[mux] for mux in range(5))
    return {
      "available": any(pages.values()),
      "received_pages": [mux for mux, frame in pages.items() if frame],
      "quality": INVERTER_QUALITY.get(_int(p0, f"{prefix}_inverterTQF"), "unknown") if pages[0] else None,
      "operating_c": {
        "pcb": _measurement(p0, f"{prefix}_pcbT", (-40.0,)),
        "inverter": _measurement(p0, f"{prefix}_inverterT", (-40.0,)),
        "stator": _measurement(p0, f"{prefix}_statorT", (-40.0,)),
        "dc_capacitor": _measurement(p0, f"{prefix}_dcCapT", (-40.0,)),
        "heatsink": _measurement(p0, f"{prefix}_heatsinkT", (-40.0,)),
      },
      "operating_percent": {
        "inverter": _measurement(p0, f"{prefix}_inverterTpct"),
        "stator": _measurement(p0, f"{prefix}_statorTpct"),
      },
      "heatsink_and_pack_c": {
        "heatsink_1": _measurement(p1, f"{prefix}_heatsink1Temp"),
        "heatsink_2": _measurement(p1, f"{prefix}_heatsink2Temp"),
        "heatsink_3": _measurement(p1, f"{prefix}_heatsink3Temp"),
        "pcb_2": _measurement(p1, f"{prefix}_pcbTemp2"),
        "junction": _measurement(p1, f"{prefix}_junctionTemp"),
        "t_pak_1": _measurement(p1, f"{prefix}_TPak1Temp"),
        "t_pak_2": _measurement(p1, f"{prefix}_TPak2Temp"),
      },
      "fluid_in_c": _measurement(p2, f"{prefix}_fluidInTemp", (-40.0,)),
      "fet_burn_in": {
        "normal": _measurement(p2, f"{prefix}_normalFetBurnIn", (500.03205,)),
        "additional": _measurement(p2, f"{prefix}_additionalFetBurnIn", (500.03205,)),
      },
      "life_estimates": {
        "current_weibull_miles": _measurement(p3, f"{prefix}_currentWeibullMiles", (262136.0,), 0),
        "end_of_service_weibull_miles": _measurement(p3, f"{prefix}_endOfServiceWeibullMiles", (262136.0,), 0),
        "burn_in_damage_ratio": _measurement(p3, f"{prefix}_burnInDamageRatio", (5.11,)),
        "sensor_estimate_c": _measurement(p3, f"{prefix}_inverterSensorEst"),
        "heatsink_1_estimate_c": _measurement(p3, f"{prefix}_inverterHS1Est"),
      },
      "estimated_c": {
        "heatsink_2": _measurement(p4, f"{prefix}_inverterHS2Est"),
        "inlet": _measurement(p4, f"{prefix}_inverterInletEst"),
        "stator_housing": _measurement(p4, f"{prefix}_statorHousingTemp", (-40.0,)),
      },
      "initial_burn_in_odometer": _measurement(p4, f"{prefix}_initialBurnInVehicleOdometer", (4294967.295,), 3),
    }

  def _drive_temperatures(self, now_ns: int) -> dict[str, Any]:
    front = self._drive_temperature_side("DIF_temperature", "DIF", now_ns)
    rear = self._drive_temperature_side("DIR_temperature", "DIR", now_ns)
    return {"available": front["available"] or rear["available"], "bus": "VEH" if front["available"] or rear["available"] else None,
            "front": front, "rear": rear}

  def _vehicle_totals(self, now_ns: int) -> dict[str, Any]:
    odometer_frame = self._frame("DI_odometerStatus", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    energy_frame = self._frame("BMS_kwhCounter", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    brake_frame = self._frame("DI_estimatedBrakeTemp", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    odometer = odometer_frame[0] if odometer_frame else {}
    energy = energy_frame[0] if energy_frame else {}
    brakes = brake_frame[0] if brake_frame else {}
    return {
      "available": bool(odometer_frame or energy_frame or brake_frame),
      "sources": sorted(filter(None, (self._bus(odometer_frame), self._bus(energy_frame), self._bus(brake_frame)))),
      "odometer_km": _measurement(odometer, "DI_odometer", (4294967.295,), 3),
      "obd_drive_cycle_active": _bool(odometer, "DI_obdDriveCycleStatus") if odometer_frame else False,
      "discharge_total_kwh": _measurement(energy, "BMS_kwhDischargeTotal", (0.0, 4294967.295), 3),
      "charge_total_kwh": _measurement(energy, "BMS_kwhChargeTotal", (0.0, 4294967.295), 3),
      "brake_temperature_c": {
        "front_left": _measurement(brakes, "DI_brakeFLTemp", (983.0,)),
        "front_right": _measurement(brakes, "DI_brakeFRTemp", (983.0,)),
        "rear_left": _measurement(brakes, "DI_brakeRLTemp", (983.0,)),
        "rear_right": _measurement(brakes, "DI_brakeRRTemp", (983.0,)),
      },
      "mcp_index": _measurement(brakes, "DI_mcpIndex"),
      "mcp_index_filtered": _measurement(brakes, "DI_mcpIndexPrimeFilt"),
    }

  def _ambient_lighting(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("UI_ambientLightingCtrls", now_ns, VEH_DIAGNOSTIC_STALE_NS)
    values = frame[0] if frame else {}
    enable_code = _int(values, "UI_rgbEnableState") if frame else None
    brightness = _int(values, "UI_rgbBrightnessLevel") if frame else None
    red = _int(values, "UI_rgbLightingColorHexRed") if frame else 0
    green = _int(values, "UI_rgbLightingColorHexGreen") if frame else 0
    blue = _int(values, "UI_rgbLightingColorHexBlue") if frame else 0
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "power_override": _bool(values, "UI_ambientLightPowerOverride") if frame else False,
      "enable_state": AMBIENT_ENABLE_STATES.get(enable_code, "unknown") if enable_code is not None else None,
      "enable_state_code": enable_code,
      "effect_code": _int(values, "UI_rgbEffectType") if frame else None,
      "effect_duration_ms": _int(values, "UI_rgbEffectType") * 250 if frame else None,
      "rgb": {"red": red, "green": green, "blue": blue},
      "hex_color": f"#{red:02X}{green:02X}{blue:02X}" if frame else None,
      "brightness": None if brightness == 127 else brightness,
      "audio_visualizer": _bool(values, "UI_audioVisualizerState") if frame else False,
      "targets": [name for name, signal in (
        ("front_left_door", "UI_rgbTargetDOORFL"), ("front_right_door", "UI_rgbTargetDOORFR"),
        ("rear_left_door", "UI_rgbTargetDOORRL"), ("rear_right_door", "UI_rgbTargetDOORRR"),
        ("instrument_panel_left", "UI_rgbTargetIPFL"), ("instrument_panel_right", "UI_rgbTargetIPFR"),
      ) if frame and _bool(values, signal)],
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
      "height": PARK_OBSTACLE_HEIGHTS.get(_int(values, "PARK_oocHeight"), "unknown") if valid else None,
      "off_course": _int(values, "PARK_oocOffCourse") if valid else None,
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
    road_disturbance = self._road_disturbance(now_ns)
    battery_diagnostics = self._battery_diagnostics(now_ns)
    tpms = self._tpms_diagnostics(now_ns)
    drive_power = self._drive_power(now_ns)
    drive_temperatures = self._drive_temperatures(now_ns)
    vehicle_totals = self._vehicle_totals(now_ns)
    ambient_lighting = self._ambient_lighting(now_ns)
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
      *([road_disturbance["bus"]] if road_disturbance.get("bus") else []),
      *(battery_diagnostics.get("sources") or []),
      *(tpms.get("sources") or []),
      *(drive_power.get("sources") or []),
      *([drive_temperatures["bus"]] if drive_temperatures.get("bus") else []),
      *(vehicle_totals.get("sources") or []),
      *([ambient_lighting["bus"]] if ambient_lighting.get("bus") else []),
    })
    return {
      "available": bool(navigation["available"] or lanes["available"] or vehicles or traffic["available"] or driver_assist["available"]
                        or road_sign["available"] or pedestrian_detection["available"] or blind_spot["available"] or front_safety["available"]
                        or longitudinal_shadow["available"] or proximity_safety["available"] or parking_obstacle["available"]
                        or road_disturbance["available"] or battery_diagnostics["available"] or tpms["available"]
                        or drive_power["available"] or drive_temperatures["available"] or vehicle_totals["available"]
                        or ambient_lighting["available"]),
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
      "road_disturbance": road_disturbance,
      "battery_diagnostics": battery_diagnostics,
      "tpms": tpms,
      "drive_power": drive_power,
      "drive_temperatures": drive_temperatures,
      "vehicle_totals": vehicle_totals,
      "ambient_lighting": ambient_lighting,
      "pedestrians": pedestrians,
      "cyclists": [vehicle for vehicle in vehicles if vehicle["type"] in ("bicycle", "motorcycle")],
    }
