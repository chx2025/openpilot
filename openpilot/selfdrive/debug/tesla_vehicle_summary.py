"""Human-readable metrics shared by the device home card and local web UI."""
import math


WHEELS = (("front_left", "左前"), ("front_right", "右前"), ("rear_left", "左后"), ("rear_right", "右后"))
CONTACTOR = {"open": "已断开", "opening": "正在断开", "closing": "正在闭合", "closed": "已闭合", "welded": "粘连", "blocked": "受阻"}
HV_STATES = {"down": "高压未上电", "coming_up": "正在上高压", "going_down": "正在下高压", "up_for_drive": "行驶高压已上电",
             "up_for_charge": "充电高压已上电", "up_for_dc_charge": "快充高压已上电", "up": "高压已上电"}
CHARGE_STATES = {"disconnected": "未连接", "no_power": "无充电功率", "about_to_charge": "准备充电", "charging": "充电中",
                 "charge_complete": "充电完成", "charge_stopped": "充电停止", "calibrating": "校准中"}
BATTERY_STATES = {"standby": "待机", "drive": "行驶", "support": "供电", "charge": "充电", "fault": "故障", "diagnostic": "诊断"}


def measurement(value, unit: str, digits: int = 1) -> str:
  if not isinstance(value, (int, float)) or not math.isfinite(value):
    return "—"
  return f"{value:,.{digits}f} {unit}"


class VehicleSummary:
  def snapshot(self, can: dict, now: float) -> dict:
    del now
    battery = can.get("battery_diagnostics", {})
    totals = can.get("vehicle_totals", {})
    tpms = can.get("tpms", {})
    wheels = tpms.get("wheels", {})
    sensors = {sensor.get("location"): sensor for sensor in tpms.get("sensors", [])}
    odometer, discharge, charge = (totals.get(k) for k in ("odometer_km", "discharge_total_kwh", "charge_total_kwh"))
    pressure = []
    for key, label in WHEELS:
      wheel = wheels.get(key, {})
      value = wheel.get("display_pressure_bar")
      source = "原车显示"
      if value is None:
        value, source = wheel.get("direct_pressure_bar"), "传感器"
      pressure.append({"label": label, "text": measurement(value, "bar", 2),
                       "battery": measurement(sensors.get(key, {}).get("battery_voltage_v"), "V", 2), "source": source,
                       "warning": bool(wheel.get("soft_warning") or wheel.get("hard_warning"))})
    brakes = totals.get("brake_temperature_c", {})
    voltage, current = battery.get("dc_link_voltage_v"), battery.get("pack_current_a")
    power = voltage * current / 1000 if all(isinstance(value, (int, float)) and math.isfinite(value) for value in (voltage, current)) else None
    raw_range = battery.get("rated_range_raw")
    return {
      "soc": measurement(battery.get("soc_percent"), "%"),
      "range": "待核实单位" if raw_range is not None else "—",
      "range_note": f"原车续航信号：{raw_range:.0f}（单位未标注）" if raw_range is not None else "等待原车续航信号",
      "odometer": measurement(odometer, "km"),
      "discharge": measurement(discharge, "kWh"),
      "charge": measurement(charge, "kWh"),
      "pressure": pressure,
      "brakes": [{"label": label, "text": measurement(brakes.get(key), "°C", 0)} for key, label in WHEELS],
      "high_voltage": {
        "voltage": measurement(voltage, "V"), "current": measurement(current, "A"), "power": measurement(power, "kW"),
        "contactor": CONTACTOR.get(battery.get("contactor_state"), "—"),
        "hv_state": HV_STATES.get(battery.get("hv_state"), "—"),
        "charge_status": CHARGE_STATES.get(battery.get("charge_status"), "—"),
        "state": BATTERY_STATES.get(battery.get("state"), "—"),
      },
    }
