from __future__ import annotations

from dataclasses import dataclass

from opendbc.can import CANParser


DBC_NAME = "tesla_modely_hw4_perception"
TRAFFIC_CONTROL_ADDRESS = 0x25D
# Vehicle logs and the web decoder agree that AP-PARTY is the authoritative
# source for this installation. Never substitute the same arbitration ID from
# another logical bus: identical IDs may carry different semantics.
TRAFFIC_CONTROL_BUSES = (2,)
TRAFFIC_CONTROL_MIN_DLC = 6
TRAFFIC_CONTROL_STALE_NS = 750_000_000
TRAFFIC_CONTROL_MAX_DISTANCE = 200.0


@dataclass(frozen=True)
class TeslaTrafficControlObservation:
  available: bool = False
  valid_for_control: bool = False
  source_bus: int = 0
  dlc: int = 0
  feature_state: int = 0
  state_machine: int = 0
  control_source: int = 0
  control_type: int = 0
  distance: float = 255.0
  light_state: int = 0
  continuation_reason: int = 0
  confirmation_type: int = 0
  warning_suppression_reason: int = 0
  unavailable_reason: int = 0
  vision_light: bool = False
  vision_sign: bool = False
  vision_road_marking: bool = False
  vision_line: bool = False
  frame_mono_time: int = 0
  quality: int = 0

  @classmethod
  def from_message(cls, msg) -> TeslaTrafficControlObservation:
    return cls(
      available=bool(msg.available), valid_for_control=bool(msg.validForControl),
      source_bus=int(msg.sourceBus), dlc=int(msg.dlc), feature_state=int(msg.featureState),
      state_machine=int(msg.stateMachine), control_source=int(msg.controlSource),
      control_type=int(msg.controlType), distance=float(msg.distance), light_state=int(msg.lightState),
      continuation_reason=int(msg.continuationReason), confirmation_type=int(msg.confirmationType),
      warning_suppression_reason=int(msg.warningSuppressionReason), unavailable_reason=int(msg.unavailableReason),
      vision_light=bool(msg.visionLight), vision_sign=bool(msg.visionSign),
      vision_road_marking=bool(msg.visionRoadMarking), vision_line=bool(msg.visionLine),
      frame_mono_time=int(msg.frameMonoTime), quality=int(msg.quality),
    )


class TeslaTrafficControlObserver:
  """Optional bus-aware parser that never participates in CAN validity."""

  def __init__(self) -> None:
    self.parsers = {
      bus: CANParser(DBC_NAME, [("APP_trafficControl", float("nan"))], bus)
      for bus in TRAFFIC_CONTROL_BUSES
    }
    self.latest_by_bus: dict[int, TeslaTrafficControlObservation] = {}

  @staticmethod
  def _control_eligible(values: dict[str, float], decoded: bool) -> bool:
    # Tesla's feature/state-machine/continuation fields describe its internal
    # UI/availability state, not a safe STOP/PASS decision. Keep publishing
    # them for diagnostics, but base control eligibility only on the displayed
    # traffic-light color and its bounded forward distance.
    distance = float(values["APP_tcControlDistance"])
    return bool(decoded and 0.0 <= distance <= TRAFFIC_CONTROL_MAX_DISTANCE)

  @staticmethod
  def _quality(decoded: bool, eligible: bool) -> int:
    if not decoded:
      return 0
    return 2 if eligible else 1

  @classmethod
  def _build(cls, values: dict[str, float], bus: int, dlc: int, timestamp_ns: int) -> TeslaTrafficControlObservation:
    control_type = int(values["APP_tcControlType"])
    control_source = int(values["APP_tcControlSource"])
    light_state = int(values["APP_tcControlLightState"])
    distance = float(values["APP_tcControlDistance"])
    decoded = control_type == 3 and light_state in (0, 1, 2, 3) and distance < 255.0
    eligible = cls._control_eligible(values, decoded)
    return TeslaTrafficControlObservation(
      available=True,
      valid_for_control=eligible,
      source_bus=bus,
      dlc=dlc,
      feature_state=int(values["APP_tcFeatureState"]),
      state_machine=int(values["APP_tcStateMachine"]),
      control_source=control_source,
      control_type=control_type,
      distance=distance,
      light_state=light_state,
      continuation_reason=int(values["APP_tcContinuationReason"]),
      confirmation_type=int(values["APP_tcConfirmationType"]),
      warning_suppression_reason=int(values["APP_tcWarningSuppressionReason"]),
      unavailable_reason=int(values["APP_tcUnavailableReason"]),
      vision_light=bool(values["APP_tcVisionLight"]),
      vision_sign=bool(values["APP_tcVisionSign"]),
      vision_road_marking=bool(values["APP_tcVisionRoadMarking"]),
      vision_line=bool(values["APP_tcVisionLine"]),
      frame_mono_time=timestamp_ns,
      quality=cls._quality(decoded, eligible),
    )

  def update(self, can_packets: list[tuple[int, list[tuple[int, bytes, int]]]], now_ns: int) -> None:
    del now_ns  # packet monotonic time is authoritative
    raw_latest: dict[int, tuple[int, int]] = {}
    for packet_mono_time, frames in can_packets:
      for address, data, source in frames:
        if source in self.parsers and address == TRAFFIC_CONTROL_ADDRESS and len(data) >= TRAFFIC_CONTROL_MIN_DLC:
          previous = raw_latest.get(source)
          if previous is None or packet_mono_time >= previous[0]:
            raw_latest[source] = (packet_mono_time, len(data))

    for bus, parser in self.parsers.items():
      parser.update(can_packets)
      raw = raw_latest.get(bus)
      if raw is None:
        continue
      timestamp_ns, dlc = raw
      observation = self._build(dict(parser.vl["APP_trafficControl"]), bus, dlc, timestamp_ns)
      previous = self.latest_by_bus.get(bus)
      if previous is None or observation.frame_mono_time >= previous.frame_mono_time:
        self.latest_by_bus[bus] = observation

  def snapshot(self, now_ns: int) -> TeslaTrafficControlObservation:
    for bus in TRAFFIC_CONTROL_BUSES:
      observation = self.latest_by_bus.get(bus)
      if observation is None:
        continue
      age_ns = now_ns - observation.frame_mono_time
      if 0 <= age_ns <= TRAFFIC_CONTROL_STALE_NS:
        return observation

    for bus in TRAFFIC_CONTROL_BUSES:
      observation = self.latest_by_bus.get(bus)
      if observation is not None:
        # Preserve the last raw tuple for diagnostics, while making it
        # impossible for a stale frame to advance any confirmation counter.
        return TeslaTrafficControlObservation(
          available=False,
          valid_for_control=False,
          source_bus=observation.source_bus,
          dlc=observation.dlc,
          feature_state=observation.feature_state,
          state_machine=observation.state_machine,
          control_source=observation.control_source,
          control_type=observation.control_type,
          distance=observation.distance,
          light_state=observation.light_state,
          continuation_reason=observation.continuation_reason,
          confirmation_type=observation.confirmation_type,
          warning_suppression_reason=observation.warning_suppression_reason,
          unavailable_reason=observation.unavailable_reason,
          vision_light=observation.vision_light,
          vision_sign=observation.vision_sign,
          vision_road_marking=observation.vision_road_marking,
          vision_line=observation.vision_line,
          frame_mono_time=observation.frame_mono_time,
          quality=observation.quality,
        )
    return TeslaTrafficControlObservation()


def publish_tesla_traffic_control(builder, observation: TeslaTrafficControlObservation) -> None:
  target = builder.teslaTrafficControl
  target.available = observation.available
  target.validForControl = observation.valid_for_control
  target.sourceBus = observation.source_bus
  target.dlc = observation.dlc
  target.featureState = observation.feature_state
  target.stateMachine = observation.state_machine
  target.controlSource = observation.control_source
  target.controlType = observation.control_type
  target.distance = observation.distance
  target.lightState = observation.light_state
  target.continuationReason = observation.continuation_reason
  target.confirmationType = observation.confirmation_type
  target.warningSuppressionReason = observation.warning_suppression_reason
  target.unavailableReason = observation.unavailable_reason
  target.visionLight = observation.vision_light
  target.visionSign = observation.vision_sign
  target.visionRoadMarking = observation.vision_road_marking
  target.visionLine = observation.vision_line
  target.frameMonoTime = observation.frame_mono_time
  target.quality = observation.quality
