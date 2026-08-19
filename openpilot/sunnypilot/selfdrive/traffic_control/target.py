from dataclasses import dataclass


@dataclass(frozen=True)
class TrafficMpcTarget:
  """Planner-only geometry from the independent Traffic Radar service."""

  event_id: int
  distance_to_stop_point: float
  should_stop: bool
