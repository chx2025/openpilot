"""Bounded three-second red lighting tests, using card's existing CAN publisher."""
import json
import threading

from opendbc.car.can_definitions import CanData

REQUEST_PARAM = "TeslaAmbientLightingRequest"
STATUS_PARAM = "TeslaAmbientLightingStatus"
ADDRESS = 0x679
BUS = 1
FRESH_NS = 1_000_000_000
DURATION_NS = 3_000_000_000
INTERVAL_NS = 100_000_000
MAX_FRAMES = 30
# Captured HW4 frame is seven bytes: FL/RL doors + left IP, or FR/RR doors + right IP.
TARGETS = {"left": (0xA8, 0), "right": (0x50, 1)}


def red_frame(template: bytes, side: str) -> bytes:
  if side not in TARGETS or len(template) != 7:
    raise ValueError("只支持左侧或右侧红色测试")
  data = bytearray(template)
  data[0] = (data[0] & 1) | 2  # Preserve power override, ON, instant transition.
  data[1:4] = bytes((255, 0, 0))
  data[4] = (data[4] & 0x80) | 100  # The live template reports 0; force a visible test level.
  data[5] = (data[5] & 0x06) | TARGETS[side][0]  # No audio visualizer; clear all other targets.
  data[6] = (data[6] & 0xFE) | TARGETS[side][1]
  return bytes(data)


class AmbientLightingController:
  def __init__(self):
    self.lock = threading.Lock()
    self.frames = {}
    self.pending = None
    self.active = None
    self.status = None
    self.last_id = None
    self.last_tx_ns = None

  def observe_frame(self, now_ns, address, data, source):
    with self.lock:
      self._observe_frame(now_ns, address, data, source)

  def _observe_frame(self, now_ns, address, data, source):
    expected_length = 7 if address == ADDRESS else 8
    if len(data) != expected_length:
      return
    if (address, source) == (ADDRESS, BUS):
      self.frames[address] = (bytes(data), now_ns)
    if self.active and address == ADDRESS and bytes(data) in self.active["payloads"] and source in (0x81, 0xC1):
      if source == 0xC1:
        self._finish("rejected", "Panda 拒绝发送，测试已停止；请确认固件与 Tesla safety 模式")
      else:
        self.active["echoes"] += 1

  def _finish(self, state, message):
    request = self.active or self.pending
    self.status = {"id": request["id"], "state": state, "message": message,
                   "submitted": request.get("count", 0), "echoes": request.get("echoes", 0)}
    self.pending = self.active = None

  def service_params(self, params):
    raw = params.get(REQUEST_PARAM)
    request = None
    if raw:
      params.remove(REQUEST_PARAM)
      try:
        value = json.loads(raw)
        if (isinstance(value, dict) and isinstance(value.get("id"), str) and isinstance(value.get("side"), str)
            and value["side"] in TARGETS and isinstance(value.get("created_ns"), int)):
          request = value
      except (ValueError, TypeError):
        pass
    with self.lock:
      if request is not None and request["id"] != self.last_id:
        self.last_id = request["id"]
        if self.pending is None and self.active is None:
          self.pending = request
      status, self.status = self.status, None
    if status is not None:
      params.put(STATUS_PARAM, json.dumps(status))

  def take_can_sends(self, now_ns):
    with self.lock:
      return self._take_can_sends(now_ns)

  def _take_can_sends(self, now_ns):
    if self.active:
      elapsed = now_ns - self.active["started_ns"]
      if elapsed >= DURATION_NS:
        count, echoes = self.active["count"], self.active["echoes"]
        self._finish("sent" if echoes else "no_echo",
                     f"3 秒测试结束：提交 {count} 帧 / 回显 {echoes} 帧；请观察左右灯带")
        return []
      if now_ns < self.active["next_ns"] or self.active["count"] >= MAX_FRAMES:
        return []
    elif self.pending is not None:
      if not 0 <= now_ns - self.pending["created_ns"] <= DURATION_NS:
        self._finish("blocked", "请求已过期，请重新点击")
        return []
      if self.last_tx_ns is not None and now_ns - self.last_tx_ns < FRESH_NS:
        self._finish("blocked", "请间隔至少一秒再测试")
        return []
    else:
      return []
    if ADDRESS not in self.frames or not 0 <= now_ns - self.frames[ADDRESS][1] <= FRESH_NS:
      self._finish("blocked", "缺少新鲜的氛围灯 CAN，测试已停止")
      return []
    request = self.active or self.pending
    try:
      data = red_frame(self.frames[ADDRESS][0], request["side"])
    except ValueError as error:
      self._finish("blocked", str(error))
      return []
    if self.active is None:
      self.active = {**self.pending, "started_ns": now_ns, "count": 0, "echoes": 0, "payloads": set()}
      self.pending = None
    self.active["payloads"].add(data)
    self.active["count"] += 1
    self.active["next_ns"] = now_ns + INTERVAL_NS  # Never catch up with a burst after a delayed loop.
    self.last_tx_ns = now_ns
    return [CanData(ADDRESS, data, BUS)]
