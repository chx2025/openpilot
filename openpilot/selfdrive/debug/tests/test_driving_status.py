from types import SimpleNamespace as ns

from openpilot.selfdrive.debug.driving_status import control_comparison, discover_ch_bus, driving_status_enabled


def test_control_comparison_aligns_oem_sp_fsd_and_lane_signals() -> None:
  car_state = ns(steeringAngleDeg=3.0, aEgo=-0.2, vEgo=15.0, vCruiseCluster=90.0)
  car_control = ns(actuators=ns(steeringAngleDeg=5.0, accel=0.8, speed=20.0))
  car_output = ns(actuatorsOutput=ns(steeringAngleDeg=4.5, accel=0.7, speed=19.5))
  controls_state = ns(desiredCurvature=0.012)
  longitudinal_plan = ns(aTarget=0.6, speeds=[19.0], accels=[0.55], jerks=[0.1], longitudinalPlanSource="cruise")
  geometry = {
    "path": [[0.0, 0.0], [20.0, 0.25]],
    "lanes": [[[0.0, 1.8], [20.0, 1.9]], [[0.0, -1.8], [20.0, -1.7]]],
    "oem_can": {
      "actuation_commands": {
        "steering": {"available": True, "angle_request_deg": 6.0},
        "cruise": {"available": True, "set_speed_kph": 90.0, "accel_min_mps2": -1.0, "accel_max_mps2": 1.5},
      },
      "longitudinal_shadow": {
        "available": True,
        "velocity_profile": {"available": True, "accel_mps2": 0.4, "future_target_speed_kph": 88.0},
        "torque_profiler": {"available": True, "accel_min_mps2": -0.8, "accel_max_mps2": 1.2, "target_speed_kph": 87.0},
      },
      "lanes": {"available": True, "center": [[0, 0.1], [20, 0.2]], "left": [[0, 1.9], [20, 2.0]], "right": [[0, -1.7], [20, -1.6]]},
    },
  }

  comparison = control_comparison(car_state, car_control, car_output, controls_state, longitudinal_plan, geometry)

  assert comparison["lateral"] == {
    "actual_angle_deg": 3.0,
    "sp_request_angle_deg": 5.0,
    "sp_output_angle_deg": 4.5,
    "sp_desired_curvature": 0.012,
    "oem_0x488_raw_angle_deg": 6.0,
    "oem_0x488_angle_deg": -6.0,
  }
  assert comparison["longitudinal"]["sp_command_accel_mps2"] == 0.8
  assert comparison["longitudinal"]["actual_speed_kph"] == 54.0
  assert comparison["longitudinal"]["sp_set_speed_kph"] == 90.0
  assert comparison["longitudinal"]["oem_0x2b9_set_speed_kph"] == 90.0
  assert comparison["longitudinal"]["fsd_0x209_accel_mps2"] == 0.4
  assert comparison["longitudinal"]["fsd_0x209_target_speed_kph"] == 88.0
  assert comparison["lanes"] == {
    "lookahead_m": 20.0,
    "sp_path_offset_m": 0.25,
    "sp_left_offset_m": 1.9,
    "sp_right_offset_m": -1.7,
    "sp_width_m": 3.6,
    "oem_center_offset_m": 0.2,
    "oem_left_offset_m": 2.0,
    "oem_right_offset_m": -1.6,
  }


def test_ch_lane_bus_is_discovered_only_on_a_dedicated_source() -> None:
  packets = [(1_000_000_000, [(0x239, b"\x00" * 8, 2), (0x239, b"\x00" * 8, 4)])]

  assert discover_ch_bus(packets) == 4
  assert discover_ch_bus([(1_000_000_000, [(0x239, b"\x00" * 8, 2)])]) is None


def test_driving_status_is_always_available_without_a_toggle() -> None:
  assert driving_status_enabled()
