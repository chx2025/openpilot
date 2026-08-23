"""Independent traffic-control observation and longitudinal constraint support."""
from openpilot.sunnypilot.selfdrive.traffic_control.controller import TrafficControlMode


TRAFFIC_SIGNAL_CONTROL_PARAM = "TeslaTrafficSignalControlEnabled"


def planner_session_is_active(sm) -> bool:
  """Fail closed if the UI cannot prove plannerd is stopped."""
  known = bool(sm.seen["deviceState"] and sm.alive["deviceState"] and sm.valid["deviceState"])
  return not known or bool(sm["deviceState"].started)


__all__ = (
  "TRAFFIC_SIGNAL_CONTROL_PARAM", "TrafficControlMode", "planner_session_is_active",
)
