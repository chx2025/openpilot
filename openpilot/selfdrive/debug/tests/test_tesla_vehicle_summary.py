from opendbc.can import CANPacker

from openpilot.selfdrive.debug.tesla_can_visualization import TeslaCanVisualization
from openpilot.selfdrive.debug.tesla_vehicle_summary import VehicleSummary


def test_soc_range_and_expiry():
  decoder = TeslaCanVisualization()
  packer = CANPacker("tesla_modely_hw4_perception")
  decoder.update([(1_000_000_000, [packer.make_can_msg("BMS_socStatus", 1, {"BMS_socUI": 72.5}),
                                 packer.make_can_msg("UI_range", 1, {"UI_ratedRange": 250})])])
  summary = VehicleSummary().snapshot(decoder.snapshot(1_100_000_000), 1.1)
  assert summary["soc"] == "72.5 %"
  assert summary["range"] == "待核实单位"
  assert "250" in summary["range_note"]
  assert VehicleSummary().snapshot(decoder.snapshot(7_000_000_000), 7)["soc"] == "—"
  decoder.update([(8_000_000_000, [packer.make_can_msg("BMS_socStatus", 1, {"BMS_socUI": 102.3})])])
  assert VehicleSummary().snapshot(decoder.snapshot(8_100_000_000), 8.1)["soc"] == "—"


def test_pressure_fallback_warning_and_missing_value():
  data = {"tpms": {
    "wheels": {"front_left": {"display_pressure_bar": 2.5, "direct_pressure_bar": 2.7, "soft_warning": True},
               "rear_right": {"direct_pressure_bar": 2.6}},
    "sensors": [{"location": "front_left", "battery_voltage_v": 3.01},
                {"location": "rear_right", "battery_voltage_v": 2.99}],
  }}
  wheels = VehicleSummary().snapshot(data, 1)["pressure"]
  assert wheels[0]["text"] == "2.50 bar" and wheels[0]["warning"]
  assert wheels[0]["battery"] == "3.01 V"
  assert wheels[1]["text"] == "—"
  assert wheels[1]["battery"] == "—"
  assert wheels[3]["text"] == "2.60 bar" and wheels[3]["source"] == "传感器"
  assert wheels[3]["battery"] == "2.99 V"


def test_vehicle_totals_brakes_and_high_voltage_battery_are_human_readable():
  data = {
    "battery_diagnostics": {"dc_link_voltage_v": 358.8, "pack_current_a": -3.9, "contactor_state": "closed",
                            "hv_state": "up_for_drive", "charge_status": "disconnected", "state": "drive"},
    "vehicle_totals": {"odometer_km": 59845.053, "discharge_total_kwh": 13523.688, "charge_total_kwh": 14269.964,
                       "brake_temperature_c": {"front_left": 32, "front_right": 32, "rear_left": 31, "rear_right": 31}},
  }
  result = VehicleSummary().snapshot(data, 1)
  assert "consumption" not in result
  assert result["odometer"] == "59,845.1 km"
  assert result["discharge"] == "13,523.7 kWh"
  assert result["charge"] == "14,270.0 kWh"
  assert [item["text"] for item in result["brakes"]] == ["32 °C", "32 °C", "31 °C", "31 °C"]
  assert result["high_voltage"] == {
    "voltage": "358.8 V", "current": "-3.9 A", "power": "-1.4 kW",
    "contactor": "已闭合", "hv_state": "行驶高压已上电", "charge_status": "未连接", "state": "行驶",
  }
