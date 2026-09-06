"""Human-readable metrics shared by the device home card and local web UI."""
import math


WHEELS = (("front_left", "左前"), ("front_right", "右前"), ("rear_left", "左后"), ("rear_right", "右后"))


def measurement(value, unit: str, digits: int = 1) -> str:
  if not isinstance(value, (int, float)) or not math.isfinite(value):
    return "—"
  return f"{value:,.{digits}f} {unit}"


class VehicleSummary:
  def __init__(self):
    self.baseline = None
    self.last_time = None

  def snapshot(self, can: dict, now: float) -> dict:
    battery = can.get("battery_diagnostics", {})
    totals = can.get("vehicle_totals", {})
    wheels = can.get("tpms", {}).get("wheels", {})
    odometer, discharge, charge = (totals.get(k) for k in ("odometer_km", "discharge_total_kwh", "charge_total_kwh"))
    consumption = None
    valid = all(v is not None and math.isfinite(v) for v in (odometer, discharge, charge))
    if not valid or self.last_time is None or now - self.last_time > 10:
      self.baseline = None
    self.last_time = now
    if valid:
      current = (odometer, discharge, charge)
      if self.baseline is None or any(a < b for a, b in zip(current, self.baseline, strict=True)):
        self.baseline = current
      distance = odometer - self.baseline[0]
      if distance >= 1.0:
        consumption = ((discharge - self.baseline[1]) - (charge - self.baseline[2])) / distance * 100
    pressure = []
    for key, label in WHEELS:
      wheel = wheels.get(key, {})
      value = wheel.get("display_pressure_bar")
      source = "原车显示"
      if value is None:
        value, source = wheel.get("direct_pressure_bar"), "传感器"
      pressure.append({"label": label, "text": measurement(value, "bar", 2), "source": source,
                       "warning": bool(wheel.get("soft_warning") or wheel.get("hard_warning"))})
    raw_range = battery.get("rated_range_raw")
    return {
      "soc": measurement(battery.get("soc_percent"), "%"),
      "range": "待核实单位" if raw_range is not None else "—",
      "range_note": f"原车续航信号：{raw_range:.0f}（单位未标注）" if raw_range is not None else "等待原车续航信号",
      "odometer": measurement(odometer, "km"),
      "consumption": measurement(consumption, "kWh/100 km"),
      "consumption_note": "本次连续观测净电耗；至少行驶 1 km 后显示",
      "discharge": measurement(discharge, "kWh"),
      "charge": measurement(charge, "kWh"),
      "pressure": pressure,
      "ambient": can.get("ambient_lighting", {}),
    }
