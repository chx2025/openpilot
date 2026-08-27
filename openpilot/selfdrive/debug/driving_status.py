"""Read-only live driving status for the local settings web UI."""
import threading
import time

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.debug.tesla_can_visualization import TeslaCanVisualization
from openpilot.selfdrive.debug.unknown_can_observer import unknown_can_snapshot


SERVICES = ("carState", "carStateSP", "carControl", "carOutput", "controlsState", "selfdriveState", "selfdriveStateSP",
            "longitudinalPlan", "modelV2")
COMPARISON_SERVICES = ("carState", "carControl", "carOutput", "controlsState", "longitudinalPlan", "modelV2")
MAX_TRAJECTORY_DISTANCE_M = 100.0
TRAJECTORY_STRIDE = 3
MAX_CAN_EVENTS_PER_SNAPSHOT = 250
TESLA_CH_LANE_ADDRESS = 0x239


def _number(value: object, digits: int = 1) -> float:
  return round(float(value), digits)


def _set_speed_kph(v_cruise_cluster: float, fallback_v_cruise: float) -> float:
  """Mirror the on-device HUD: vCruiseCluster is already in display units."""
  return fallback_v_cruise if v_cruise_cluster == 0.0 else v_cruise_cluster


def _is_tesla_model_y(car_params: bytes | None) -> bool:
  if not car_params:
    return False
  try:
    with car.CarParams.from_bytes(car_params) as cp:
      return cp.brand == "tesla" and cp.carFingerprint == "TESLA_MODEL_Y"
  except Exception:
    return False


def discover_ch_bus(packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> int | None:
  """Discover a separately exposed CH source without reinterpreting PARTY/VEH by address alone."""
  for _, frames in reversed(packets):
    for address, _, source in frames:
      if address == TESLA_CH_LANE_ADDRESS and source not in (0, 1, 2):
        return source
  return None


def comparison_services_available(alive: dict[str, bool]) -> bool:
  return all(bool(alive.get(service, False)) for service in COMPARISON_SERVICES)


def _line_points(line: object) -> list[list[float]]:
  """Compact model coordinates for browser-side rendering."""
  xs, ys = list(line.x), list(line.y)
  points = []
  for index in range(0, min(len(xs), len(ys)), TRAJECTORY_STRIDE):
    x = float(xs[index])
    if 0.0 <= x <= MAX_TRAJECTORY_DISTANCE_M:
      points.append([_number(x), _number(ys[index], 2)])
  return points


def _point_offset(points: list[list[float]], lookahead_m: float) -> float | None:
  if not points:
    return None
  before = points[0]
  for after in points[1:]:
    if after[0] >= lookahead_m:
      if after[0] == before[0]:
        return _number(after[1], 2)
      ratio = (lookahead_m - before[0]) / (after[0] - before[0])
      return _number(before[1] + ratio * (after[1] - before[1]), 2)
    before = after
  return _number(points[-1][1], 2) if points[-1][0] >= lookahead_m * 0.75 else None


def control_comparison(car_state: object, car_control: object, car_output: object, controls_state: object,
                       longitudinal_plan: object, geometry: dict[str, object]) -> dict[str, object]:
  oem_can = geometry.get("oem_can", {})
  commands = oem_can.get("actuation_commands", {})
  steering = commands.get("steering", {})
  cruise = commands.get("cruise", {})
  fsd = oem_can.get("longitudinal_shadow", {})
  velocity_profile = fsd.get("velocity_profile", {})
  torque_profiler = fsd.get("torque_profiler", {})

  sp_lanes = geometry.get("lanes", [])
  sp_left = sp_lanes[0] if len(sp_lanes) > 0 else []
  sp_right = sp_lanes[1] if len(sp_lanes) > 1 else []
  oem_lanes = oem_can.get("lanes", {})
  lookahead_m = 20.0
  sp_left_offset = _point_offset(sp_left, lookahead_m)
  sp_right_offset = _point_offset(sp_right, lookahead_m)

  speeds = list(longitudinal_plan.speeds)
  accels = list(longitudinal_plan.accels)
  jerks = list(longitudinal_plan.jerks)
  return {
    "lateral": {
      "actual_angle_deg": _number(car_state.steeringAngleDeg, 2),
      "sp_request_angle_deg": _number(car_control.actuators.steeringAngleDeg, 2),
      "sp_output_angle_deg": _number(car_output.actuatorsOutput.steeringAngleDeg, 2),
      "sp_desired_curvature": _number(controls_state.desiredCurvature, 6),
      "oem_0x488_raw_angle_deg": steering.get("angle_request_deg") if steering.get("available") else None,
      # Tesla PARTY encodes the rack request with the opposite sign from the
      # CarState/SP steering-angle convention (see TeslaCAN.create_steering_control).
      "oem_0x488_angle_deg": (-steering["angle_request_deg"]
                              if steering.get("available") and steering.get("angle_request_deg") is not None else None),
    },
    "longitudinal": {
      "actual_accel_mps2": _number(car_state.aEgo, 2),
      "sp_command_accel_mps2": _number(car_control.actuators.accel, 2),
      "sp_plan_accel_mps2": _number(longitudinal_plan.aTarget, 2),
      "actual_speed_kph": _number(car_state.vEgo * 3.6, 2),
      "sp_set_speed_kph": _number(car_state.vCruiseCluster, 2),
      "sp_plan_speed_kph": _number(speeds[0] * 3.6, 2) if speeds else None,
      "sp_plan_jerk_mps3": _number(jerks[0], 2) if jerks else None,
      "sp_trajectory_accel_mps2": _number(accels[0], 2) if accels else None,
      "plan_source": str(longitudinal_plan.longitudinalPlanSource),
      "oem_0x2b9_set_speed_kph": cruise.get("set_speed_kph") if cruise.get("available") else None,
      "oem_0x2b9_accel_min_mps2": cruise.get("accel_min_mps2") if cruise.get("available") else None,
      "oem_0x2b9_accel_max_mps2": cruise.get("accel_max_mps2") if cruise.get("available") else None,
      "oem_0x2b9_jerk_min_mps3": cruise.get("jerk_min_mps3") if cruise.get("available") else None,
      "oem_0x2b9_jerk_max_mps3": cruise.get("jerk_max_mps3") if cruise.get("available") else None,
      "fsd_0x209_accel_mps2": velocity_profile.get("accel_mps2") if velocity_profile.get("available") else None,
      "fsd_0x209_target_speed_kph": (velocity_profile.get("future_target_speed_kph") if velocity_profile.get("available")
                                     else torque_profiler.get("target_speed_kph") if torque_profiler.get("available") else None),
      "fsd_0x209_accel_min_mps2": torque_profiler.get("accel_min_mps2") if torque_profiler.get("available") else None,
      "fsd_0x209_accel_max_mps2": torque_profiler.get("accel_max_mps2") if torque_profiler.get("available") else None,
    },
    "lanes": {
      "lookahead_m": lookahead_m,
      "sp_path_offset_m": _point_offset(geometry.get("path", []), lookahead_m),
      "sp_left_offset_m": sp_left_offset,
      "sp_right_offset_m": sp_right_offset,
      "sp_width_m": _number(sp_left_offset - sp_right_offset, 2) if sp_left_offset is not None and sp_right_offset is not None else None,
      "oem_center_offset_m": _point_offset(oem_lanes.get("center", []), lookahead_m) if oem_lanes.get("available") else None,
      "oem_left_offset_m": _point_offset(oem_lanes.get("left", []), lookahead_m) if oem_lanes.get("available") else None,
      "oem_right_offset_m": _point_offset(oem_lanes.get("right", []), lookahead_m) if oem_lanes.get("available") else None,
    },
  }


def _model_geometry(model: object, car_state_sp: object, oem_can: dict[str, object]) -> dict[str, object]:
  leads = []
  for lead in model.leadsV3:
    if lead.prob >= 0.5 and len(lead.x) and len(lead.y):
      leads.append({"x": _number(lead.x[0]), "y": _number(lead.y[0], 2), "velocity_mps": _number(lead.v[0]), "probability": _number(lead.prob, 2)})
  return {
    "path": _line_points(model.position),
    # The inner pair is the current-lane boundary and is the clearest signal on a compact display.
    "lanes": [_line_points(line) for line in list(model.laneLines)[1:3]],
    "edges": [_line_points(line) for line in model.roadEdges],
    "leads": leads,
    "lane_change": str(model.meta.laneChangeState),
    "lane_change_direction": str(model.meta.laneChangeDirection),
    "hard_brake_predicted": bool(model.meta.hardBrakePredicted),
    "oem_traffic": {
      "available": bool(car_state_sp.teslaRoadContext.available),
      "light_color": int(car_state_sp.teslaRoadContext.trafficLightColor),
      "stop_line_distance": _number(car_state_sp.teslaRoadContext.stopLineDistance),
    },
    "oem_can": oem_can,
  }


class DrivingStatus:
  def __init__(self) -> None:
    self.params = Params()
    self.sm = messaging.SubMaster(SERVICES)
    self.can_sock = messaging.sub_sock("can", conflate=False)
    self.tesla_can = TeslaCanVisualization()
    self.lock = threading.Lock()

  def _is_tesla_model_y(self) -> bool:
    car_params = self.params.get("CarParams") or self.params.get("CarParamsPersistent")
    return _is_tesla_model_y(car_params)

  def _update_tesla_can(self) -> dict[str, object]:
    packets = []
    events = messaging.drain_sock(self.can_sock)
    for event in events[-MAX_CAN_EVENTS_PER_SNAPSHOT:]:
      packets.append((event.logMonoTime, [(frame.address, bytes(frame.dat), frame.src) for frame in event.can]))
    if self._is_tesla_model_y():
      if self.tesla_can.ch_bus is None and (ch_bus := discover_ch_bus(packets)) is not None:
        self.tesla_can = TeslaCanVisualization(ch_bus=ch_bus)
      self.tesla_can.update(packets)
    else:
      self.tesla_can.reset()
    return self.tesla_can.snapshot()

  def snapshot(self) -> dict[str, object]:
    with self.lock:
      self.sm.update(0)
      car_state = self.sm["carState"]
      car_state_sp = self.sm["carStateSP"]
      controls_state = self.sm["controlsState"]
      selfdrive_state = self.sm["selfdriveState"]
      sp_state = self.sm["selfdriveStateSP"]
      model = self.sm["modelV2"]
      oem_can = self._update_tesla_can()
      oem_can["unknown_frames"] = unknown_can_snapshot()
      geometry = _model_geometry(model, car_state_sp, oem_can)
      comparison = (control_comparison(car_state, self.sm["carControl"], self.sm["carOutput"], controls_state,
                                       self.sm["longitudinalPlan"], geometry)
                    if comparison_services_available(self.sm.alive) else None)

      alert = " ".join(text for text in (selfdrive_state.alertText1, selfdrive_state.alertText2) if text)
      cruise_speed = _set_speed_kph(float(car_state.vCruiseCluster), float(controls_state.deprecated.vCruise))
      return {
        "onroad": not self.params.get_bool("IsOffroad"),
        "connected": {service: self.sm.alive[service] for service in SERVICES},
        "speed_kph": _number(car_state.vEgo * 3.6),
        "set_speed_kph": _number(cruise_speed),
        "openpilot_enabled": bool(selfdrive_state.enabled),
        "mads_enabled": bool(sp_state.mads.enabled),
        "alert": alert,
        "geometry": geometry,
        "comparison": comparison,
        "updated_at": int(time.monotonic()),
      }


_STATUS: DrivingStatus | None = None
_STATUS_LOCK = threading.Lock()


def driving_status_enabled() -> bool:
  return True


def driving_status_snapshot() -> dict[str, object]:
  global _STATUS
  if not driving_status_enabled():
    with _STATUS_LOCK:
      _STATUS = None
    raise PermissionError("请先在设备的 Tesla 设置中开启“浏览器行驶信息”")

  with _STATUS_LOCK:
    if _STATUS is None:
      _STATUS = DrivingStatus()
    status = _STATUS
  return status.snapshot()
