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
  data = {"tpms": {"wheels": {"front_left": {"display_pressure_bar": 2.5, "direct_pressure_bar": 2.7, "soft_warning": True},
                              "rear_right": {"direct_pressure_bar": 2.6}}}}
  wheels = VehicleSummary().snapshot(data, 1)["pressure"]
  assert wheels[0]["text"] == "2.50 bar" and wheels[0]["warning"]
  assert wheels[1]["text"] == "—"
  assert wheels[3]["text"] == "2.60 bar" and wheels[3]["source"] == "传感器"


def test_net_consumption_requires_distance_and_resets_on_gap_or_counter_reset():
  summary = VehicleSummary()
  def sample(t, km, discharge, charge):
    return summary.snapshot({"vehicle_totals": {"odometer_km": km, "discharge_total_kwh": discharge, "charge_total_kwh": charge}}, t)
  assert sample(1, 100, 20, 10)["consumption"] == "—"
  assert sample(2, 100.5, 20.2, 10.1)["consumption"] == "—"
  assert sample(3, 102, 20.4, 10.1)["consumption"] == "15.0 kWh/100 km"
  assert sample(20, 110, 25, 11)["consumption"] == "—"
  assert sample(21, 111, 1, 1)["consumption"] == "—"
