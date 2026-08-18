"""Independent traffic-control observation and longitudinal constraint support."""
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlMode


MODE_PARAM = "TeslaTrafficControlMode"


def configured_mode(params) -> TrafficControlMode:
  try:
    return TrafficControlMode(int(params.get(MODE_PARAM, return_default=True)))
  except (TypeError, ValueError):
    return TrafficControlMode.off


def decorate_planner(planner, CP, params):
  """Attach the single decision/constraint pipeline when Tesla monitoring is enabled."""
  if CP.brand != "tesla" or configured_mode(params) == TrafficControlMode.off:
    return planner

  from openpilot.sunnypilot.selfdrive.traffic_control.planner_adapter import TrafficControlPlannerAdapter
  return TrafficControlPlannerAdapter(planner, CP, params)


__all__ = ("TrafficControlMode", "configured_mode", "decorate_planner")
