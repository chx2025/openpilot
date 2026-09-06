import json
import pytest
from opendbc.can import CANPacker, CANParser

from openpilot.sunnypilot.selfdrive.car.tesla.ambient_lighting import AmbientLightingController, red_frame, REQUEST_PARAM

TEMPLATE = bytes.fromhex("0cffd5aa00f801")  # Captured from the vehicle at 0x679@VEH.


@pytest.mark.parametrize("side,targets", [("left", [1, 0, 1, 0, 1, 0]), ("right", [0, 1, 0, 1, 0, 1])])
def test_red_only_and_preserves_unrelated_template_bits(side, targets):
  data = red_frame(TEMPLATE, side)
  parser = CANParser("tesla_modely_hw4_perception", [("UI_ambientLightingCtrls", 0)], 1)
  parser.update([(1_000_000_000, [(0x679, data, 1)])])
  v = parser.vl["UI_ambientLightingCtrls"]
  assert [v["UI_rgbTarget" + suffix] for suffix in ("DOORFL", "DOORFR", "DOORRL", "DOORRR", "IPFL", "IPFR")] == targets
  assert [v["UI_rgbLightingColorHex" + c] for c in ("Red", "Green", "Blue")] == [255, 0, 0]
  assert v["UI_rgbBrightnessLevel"] == 100
  assert v["UI_rgbEnableState"] == 1 and v["UI_rgbEffectType"] == 0 and v["UI_audioVisualizerState"] == 0
  assert data[6] & 0xFE == TEMPLATE[6] & 0xFE


@pytest.mark.parametrize("side,targets", [("left", [1, 0, 1, 0, 1, 0]), ("right", [0, 1, 0, 1, 0, 1]),
                                          ("both", [1, 1, 1, 1, 1, 1])])
def test_blindspot_frame_supports_flash_off_and_both_sides(side, targets):
  data = red_frame(TEMPLATE, side, brightness=0)
  parser = CANParser("tesla_modely_hw4_perception", [("UI_ambientLightingCtrls", 0)], 1)
  parser.update([(1_000_000_000, [(0x679, data, 1)])])
  values = parser.vl["UI_ambientLightingCtrls"]
  assert values["UI_rgbBrightnessLevel"] == 0
  assert [values["UI_rgbTarget" + suffix] for suffix in ("DOORFL", "DOORFR", "DOORRL", "DOORRR", "IPFL", "IPFR")] == targets


@pytest.mark.parametrize("data,side", [(TEMPLATE, "invalid"), (b"", "left"), (b"\0" * 8, "left")])
def test_invalid_request_or_length(data, side):
  with pytest.raises(ValueError):
    red_frame(data, side)


def ready_controller(gear=1, speed=0):
  c = AmbientLightingController()
  c.pending = {"id": "a", "side": "left", "created_ns": 1_000_000_000}
  packer = CANPacker("tesla_model3_party")
  for a, d, b in [packer.make_can_msg("DI_systemStatus", 0, {"DI_gear": gear}),
                  packer.make_can_msg("DI_speed", 0, {"DI_vehicleSpeed": speed}), (0x679, TEMPLATE, 1)]:
    c.observe_frame(1_000_000_000, a, d, b)
  return c


def refresh(c, timestamp, gear=1, speed=0):
  packer = CANPacker("tesla_model3_party")
  for a, d, b in [packer.make_can_msg("DI_systemStatus", 0, {"DI_gear": gear}),
                  packer.make_can_msg("DI_speed", 0, {"DI_vehicleSpeed": speed}), (0x679, TEMPLATE, 1)]:
    c.observe_frame(timestamp, a, d, b)


def test_three_seconds_30_frames_exact_echo_and_no_repeat():
  c = ready_controller()
  frames = []
  for index in range(30):
    now = 1_100_000_000 + index * 100_000_000
    refresh(c, now)
    sent = c.take_can_sends(now)
    assert len(sent) == 1
    frames.extend(sent)
    c.observe_frame(now, 0x679, TEMPLATE, 0x81)
    assert c.active["echoes"] == index
    c.observe_frame(now, 0x679, sent[0][1], 0x81)
    assert c.take_can_sends(now + 50_000_000) == []
  assert c.take_can_sends(4_100_000_000) == []
  assert c.status["state"] == "sent" and c.status["submitted"] == c.status["echoes"] == 30
  assert c.take_can_sends(5_000_000_000) == []


def test_delayed_loop_does_not_burst_to_catch_up():
  c = ready_controller()
  assert len(c.take_can_sends(1_100_000_000)) == 1
  refresh(c, 2_000_000_000)
  assert len(c.take_can_sends(2_000_000_000)) == 1
  assert c.take_can_sends(2_000_000_001) == []


@pytest.mark.parametrize("left,right,side,target_bytes", [
  (True, False, "left", (0xA8, 0)), (False, True, "right", (0x50, 1)), (True, True, "both", (0xF8, 1)),
])
def test_blindspot_flashes_requested_side_at_five_hz(left, right, side, target_bytes):
  c = AmbientLightingController()
  c.observe_frame(1_000_000_000, 0x679, TEMPLATE, 1)
  c.update_blindspot(left, right, 1_100_000_000)
  frames = []
  for index in range(4):
    now = 1_100_000_000 + index * 100_000_000
    refresh(c, now)
    frames.extend(c.take_can_sends(now))
  assert c.blindspot_side == side
  assert [frame[1][4] & 0x7F for frame in frames] == [100, 0, 100, 0]
  assert all((frame[1][5] & 0xF8, frame[1][6] & 1) == target_bytes for frame in frames)
  c.update_blindspot(False, False, 1_500_000_000)
  assert c.take_can_sends(1_500_000_000) == []


def test_blindspot_alert_is_bounded_to_fifteen_seconds_and_rearms_after_clear():
  c = AmbientLightingController()
  c.update_blindspot(True, False, 1_000_000_000)
  sent = []
  for index in range(151):
    now = 1_000_000_000 + index * 100_000_000
    c.observe_frame(now, 0x679, TEMPLATE, 1)
    sent.extend(c.take_can_sends(now))
  assert len(sent) == 150
  c.update_blindspot(False, False, 16_100_000_000)
  c.update_blindspot(False, True, 17_100_000_000)
  c.observe_frame(17_100_000_000, 0x679, TEMPLATE, 1)
  assert len(c.take_can_sends(17_100_000_000)) == 1


@pytest.mark.parametrize("gear,speed", [(4, 0), (1, 1)])
def test_test_is_independent_of_gear_and_speed(gear, speed):
  c = ready_controller()
  c.take_can_sends(1_100_000_000)
  refresh(c, 1_200_000_000, gear, speed)
  assert len(c.take_can_sends(1_200_000_000)) == 1


def test_blocks_stale_ambient_template():
  c = ready_controller()
  assert c.take_can_sends(2_100_000_001) == []
  assert c.status["state"] == "blocked"


@pytest.mark.parametrize("source,result", [(0xC1, "rejected"), (0x81, "sent")])
def test_transmission_outcome(source, result):
  c = ready_controller()
  frame = c.take_can_sends(1_100_000_000)[0]
  c.observe_frame(1_200_000_000, 0x679, frame[1], source)
  if result == "sent":
    c.take_can_sends(4_100_000_000)
  assert c.status["state"] == result


def test_timeout_is_not_success():
  c = ready_controller()
  c.take_can_sends(1_100_000_000)
  c.take_can_sends(4_100_000_000)
  assert c.status["state"] == "no_echo"


def test_request_expiry_after_restart():
  c = ready_controller()
  c.pending = None
  class Params:
    def remove(self, key):
      pass

    def get(self, key):
      return json.dumps({"id": "old", "side": "right", "created_ns": 1}) if key == REQUEST_PARAM else None
  c.service_params(Params())
  assert c.take_can_sends(10_000_000_000) == []
  assert c.status["state"] == "blocked"
