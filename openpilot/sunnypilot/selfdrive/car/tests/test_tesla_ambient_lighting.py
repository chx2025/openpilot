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


@pytest.mark.parametrize("data,side", [(TEMPLATE, "both"), (b"", "left"), (b"\0" * 8, "left")])
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


@pytest.mark.parametrize("gear,speed", [(4, 0), (1, 1)])
def test_mid_test_motion_stops_transmission(gear, speed):
  c = ready_controller()
  c.take_can_sends(1_100_000_000)
  refresh(c, 1_200_000_000, gear, speed)
  assert c.take_can_sends(1_200_000_000) == []
  assert c.status["state"] == "blocked"


@pytest.mark.parametrize("gear,speed,now", [(4, 0, 1.1), (1, 1, 1.1), (1, 0, 2.1), (1, 0, 5)])
def test_blocks_motion_non_park_and_stale_data(gear, speed, now):
  c = ready_controller(gear, speed)
  assert c.take_can_sends(int(now * 1e9)) == []
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
