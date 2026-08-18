from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import (
  DAS_BODY_CONTROLS_ADDRESS,
  TeslaTurnSignalRealtimeController,
  create_body_control_frame,
  decode_body_controls,
  is_original_body_controls_frame,
  tesla_body_controls_checksum,
)


def idle_body_controls(counter: int = 4) -> bytes:
  data = bytearray([0xA5, 0x8C, 0x61, 0xB4, 0x5A, 0xC3, (counter << 4) | 0x07, 0])
  data[7] = tesla_body_controls_checksum(data)
  return bytes(data)


def test_turn_frame_changes_only_owned_fields_and_recomputes_checksum():
  original = idle_body_controls()
  sent = create_body_control_frame(original, "left", 5)

  assert is_original_body_controls_frame(DAS_BODY_CONTROLS_ADDRESS, 1, original)
  assert decode_body_controls(sent)["turn_request"] == 1
  assert decode_body_controls(sent)["turn_request_reason"] == 8
  assert decode_body_controls(sent)["counter"] == 5
  assert sent[0] == original[0]
  assert sent[3:6] == original[3:6]
  assert sent[7] == tesla_body_controls_checksum(sent)


def test_disabled_controller_fails_closed_without_can_send():
  controller = TeslaTurnSignalRealtimeController(configured=False)
  assert not controller.submit_request("test", "left", 100)
  assert controller.take_can_sends(100) == []
  completed = controller.drain_completed()
  assert completed[0][0]["result"] == "BLOCKED"


def test_enabled_controller_requires_fresh_original_template():
  controller = TeslaTurnSignalRealtimeController(configured=True)
  assert controller.submit_request("test", "right", 100)
  assert controller.take_can_sends(100) == []

  original = idle_body_controls(9)
  controller.observe_frame(200, DAS_BODY_CONTROLS_ADDRESS, original, 1)
  sends = controller.take_can_sends(200)

  assert len(sends) == 1
  assert sends[0].address == DAS_BODY_CONTROLS_ADDRESS
  assert sends[0].src == 1
  assert decode_body_controls(sends[0].dat)["turn_request"] == 2
  assert decode_body_controls(sends[0].dat)["counter"] == 10
