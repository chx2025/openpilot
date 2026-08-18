#!/usr/bin/env python3
import argparse
from enum import StrEnum
import json
import os
import sys
import time
import uuid

import openpilot.cereal.messaging as messaging

from openpilot.common.params import Params
from openpilot.selfdrive.pandad import can_list_to_can_capnp
from opendbc.car.can_definitions import CanData


SWITCH_STATUS_ADDRESS = 0x3C2
VEHICLE_BUS = 1
SWITCH_STATUS_WHEEL_INDEX = 1
VALIDATION_LOG_PATH = "/data/tesla_speed_button_validation.log"
VALIDATION_LOG_PREFIX = "[TESLA-SPEED-BUTTON-VALIDATION-v2]"
MAX_LOG_BYTES = 2 * 1024 * 1024
# Keep the manual validator available, but do not create persistent logs on dev.
SPEED_BUTTON_VALIDATION_LOGGING_ENABLED = False
TX_OBSERVE_S = 0.25


class SpeedButtonAction(StrEnum):
  increase = "increase"
  decrease = "decrease"


def _decode_signed_6(raw_value: int) -> int:
  raw_value &= 0x3F
  return raw_value - 0x40 if raw_value & 0x20 else raw_value


def _encode_signed_6(value: int) -> int:
  if not -32 <= value <= 31:
    raise ValueError(f"right-scroll value outside signed 6-bit range: {value}")
  return value & 0x3F


def switch_status_index(data: bytes) -> int:
  if len(data) != 8:
    raise ValueError("0x3C2 VCLEFT_switchStatus must contain 8 bytes")
  return data[0] & 0x3


def decode_right_scroll_ticks(data: bytes) -> int:
  if len(data) != 8:
    raise ValueError("0x3C2 VCLEFT_switchStatus must contain 8 bytes")
  return _decode_signed_6(data[3])


def is_original_vehicle_speed_frame(address: int, source: int, data: bytes) -> bool:
  return (address == SWITCH_STATUS_ADDRESS and source == VEHICLE_BUS and len(data) == 8 and
          switch_status_index(data) == SWITCH_STATUS_WHEEL_INDEX)


def create_speed_button_frame(original_idle_frame: bytes, action: SpeedButtonAction) -> bytes:
  if not is_original_vehicle_speed_frame(SWITCH_STATUS_ADDRESS, VEHICLE_BUS, original_idle_frame):
    raise ValueError("original 0x3C2 RX template is not a wheel-status mux-1 frame")
  if decode_right_scroll_ticks(original_idle_frame) != 0:
    raise ValueError("original 0x3C2 RX template is not an idle right-scroll frame")

  data = bytearray(original_idle_frame)
  tick = 1 if action == SpeedButtonAction.increase else -1
  data[3] = (data[3] & 0xC0) | _encode_signed_6(tick)
  return bytes(data)


class ValidationRecorder:
  def __init__(self, action: SpeedButtonAction, log_path: str = VALIDATION_LOG_PATH):
    self.action = action
    self.log_path = log_path
    self.test_id = uuid.uuid4().hex[:12]
    if SPEED_BUTTON_VALIDATION_LOGGING_ENABLED:
      try:
        if os.path.exists(log_path) and os.path.getsize(log_path) > MAX_LOG_BYTES:
          os.replace(log_path, f"{log_path}.1")
      except OSError:
        pass

  def record(self, event: str, **values) -> None:
    if not SPEED_BUTTON_VALIDATION_LOGGING_ENABLED:
      return

    record = {
      "prefix": VALIDATION_LOG_PREFIX,
      "test_id": self.test_id,
      "wall_time_ns": time.time_ns(),
      "monotonic_ns": time.monotonic_ns(),
      "action": self.action.value,
      "event": event,
      **values,
    }
    try:
      with open(self.log_path, "a", encoding="utf-8") as log_file:
        log_file.write(json.dumps(record, sort_keys=True, default=str) + "\n")
        log_file.flush()
    except OSError:
      pass


def wait_for_original_idle_template(can_sock: messaging.SubSocket, recorder: ValidationRecorder,
                                    timeout_s: float = 5.0) -> bytes:
  deadline = time.monotonic() + timeout_s
  while time.monotonic() < deadline:
    event = messaging.recv_one_or_none(can_sock)
    if event is None:
      continue
    for frame in event.can:
      data = bytes(frame.dat)
      if not is_original_vehicle_speed_frame(int(frame.address), int(frame.src), data):
        continue
      right_ticks = decode_right_scroll_ticks(data)
      recorder.record("original_rx_observation", source=int(frame.src), bus=VEHICLE_BUS, data=data.hex(),
                      switch_status_index=switch_status_index(data), right_scroll_ticks=right_ticks)
      if right_ticks == 0:
        recorder.record("original_rx_template_selected", source=int(frame.src), bus=VEHICLE_BUS, data=data.hex())
        return data
  raise RuntimeError("no fresh idle 0x3C2 wheel-status mux-1 frame received on bus 1")


def latest_cruise_speed(car_state_sock: messaging.SubSocket, duration_s: float) -> float | None:
  latest = None
  deadline = time.monotonic() + duration_s
  while time.monotonic() < deadline:
    event = messaging.recv_one_or_none(car_state_sock)
    if event is not None:
      latest = float(event.carState.cruiseState.speedCluster)
  return latest


def observe_transmission(can_sock: messaging.SubSocket, recorder: ValidationRecorder,
                         duration_s: float) -> tuple[bool, bool, bool]:
  tx_echo = False
  rejected = False
  original_idle_returned = False
  deadline = time.monotonic() + duration_s
  while time.monotonic() < deadline:
    event = messaging.recv_one_or_none(can_sock)
    if event is None:
      continue
    for frame in event.can:
      if int(frame.address) != SWITCH_STATUS_ADDRESS or len(frame.dat) != 8:
        continue
      source = int(frame.src)
      data = bytes(frame.dat)
      if source == VEHICLE_BUS:
        right_ticks = decode_right_scroll_ticks(data) if switch_status_index(data) == SWITCH_STATUS_WHEEL_INDEX else None
        original_idle_returned |= right_ticks == 0
        recorder.record("original_rx_observation", source=source, bus=VEHICLE_BUS, data=data.hex(),
                        switch_status_index=switch_status_index(data), right_scroll_ticks=right_ticks)
      elif source == VEHICLE_BUS + 0x80:
        tx_echo = True
        recorder.record("transmission_echo", source=source, bus=VEHICLE_BUS, data=data.hex(),
                        right_scroll_ticks=decode_right_scroll_ticks(data))
      elif source == VEHICLE_BUS + 0xC0:
        rejected = True
        recorder.record("transmission_rejected", source=source, bus=VEHICLE_BUS, data=data.hex())
  return tx_echo, rejected, original_idle_returned


def run_validation(action: SpeedButtonAction, log_path: str = VALIDATION_LOG_PATH) -> int:
  recorder = ValidationRecorder(action, log_path)
  recorder.record("test_started", address=hex(SWITCH_STATUS_ADDRESS), analysis_source="fresh_original_rx_template")
  try:
    if not Params().get_bool("TeslaSpeedButtonValidation"):
      raise RuntimeError("TeslaSpeedButtonValidation is disabled; enable it offroad and restart")

    can_sock = messaging.sub_sock("can", timeout=10)
    car_state_sock = messaging.sub_sock("carState", timeout=10)
    sendcan = messaging.pub_sock("sendcan")

    original_idle = wait_for_original_idle_template(can_sock, recorder)
    before_speed = latest_cruise_speed(car_state_sock, 0.3)
    action_data = create_speed_button_frame(original_idle, action)
    recorder.record("frame_submitted", bus=VEHICLE_BUS, data=action_data.hex(),
                    right_scroll_ticks=decode_right_scroll_ticks(action_data))
    sendcan.send(can_list_to_can_capnp([CanData(SWITCH_STATUS_ADDRESS, action_data, VEHICLE_BUS)], msgtype="sendcan"))
    tx_echo, rejected, original_idle_returned = observe_transmission(can_sock, recorder, TX_OBSERVE_S)

    if rejected and not tx_echo:
      recorder.record("test_finished", result="PANDA_REJECTED", before_speed=before_speed)
      print(f"FAIL: Panda rejected 0x3C2 frame; log={log_path}; test_id={recorder.test_id}")
      return 2
    if not tx_echo:
      recorder.record("test_finished", result="NO_TX_ECHO", before_speed=before_speed)
      print(f"FAIL: no Panda transmission echo; log={log_path}; test_id={recorder.test_id}")
      return 2

    after_speed = latest_cruise_speed(car_state_sock, 2.0)
    changed_correctly = None
    if before_speed is not None and after_speed is not None:
      delta = after_speed - before_speed
      changed_correctly = delta > 0.1 if action == SpeedButtonAction.increase else delta < -0.1
      if abs(delta) > 0.1 and not changed_correctly:
        recorder.record("test_finished", result="WRONG_DIRECTION", before_speed=before_speed,
                        after_speed=after_speed, delta=delta, original_idle_returned=original_idle_returned)
        print(f"FAIL: set speed changed in wrong direction ({delta * 3.6:+.1f} km/h); log={log_path}; test_id={recorder.test_id}")
        return 2

    result = "PASS" if changed_correctly else "SENT_CHECK_VEHICLE_UI"
    recorder.record("test_finished", result=result, before_speed=before_speed, after_speed=after_speed,
                    original_idle_returned=original_idle_returned)
    if changed_correctly:
      print(f"PASS: set speed changed from {before_speed * 3.6:.1f} to {after_speed * 3.6:.1f} km/h; log={log_path}; test_id={recorder.test_id}")
      return 0

    sent_message = "SENT: inspect the vehicle set-speed display; 0x3C2 was transmitted but no reliable speed edge was observed; "
    print(sent_message + f"log={log_path}; test_id={recorder.test_id}")
    return 3
  except (RuntimeError, ValueError) as error:
    recorder.record("test_finished", result="BLOCKED", error=str(error))
    print(f"BLOCKED: {error}", file=sys.stderr)
    return 1


def main() -> int:
  parser = argparse.ArgumentParser(description="Tesla right-scroll CAN validation")
  parser.add_argument("action", choices=(SpeedButtonAction.increase.value, SpeedButtonAction.decrease.value))
  args = parser.parse_args()
  return run_validation(SpeedButtonAction(args.action))


if __name__ == "__main__":
  raise SystemExit(main())
