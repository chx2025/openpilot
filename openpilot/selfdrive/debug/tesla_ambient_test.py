"""Local console entry point for card-owned ambient lighting tests."""
import json
import threading
import time
import uuid

from opendbc.car.structs import CarParams

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.debug.driving_status import driving_status_snapshot
from openpilot.selfdrive.pandad import can_list_to_can_capnp
from openpilot.sunnypilot.selfdrive.car.tesla.ambient_lighting import (
  ADDRESS, AmbientLightingController, REQUEST_PARAM, STATUS_PARAM, TARGETS,
)

_LOCK = threading.Lock()
OFFROAD_ACTIVE_PARAM = "TeslaAmbientLightingOffroadActive"


def _wait_offroad_safety(active: bool, timeout_s: float = 2.0) -> bool:
  sm = messaging.SubMaster(["pandaStates"])
  deadline = time.monotonic() + timeout_s
  expected_param = 1 if active else 0
  while time.monotonic() < deadline:
    sm.update(100)
    if (sm.seen["pandaStates"] and len(sm["pandaStates"]) == 1 and
        sm["pandaStates"][0].safetyModel == CarParams.SafetyModel.noOutput and
        int(sm["pandaStates"][0].safetyParam) == expected_param):
      return True
  return False


def _run_offroad_test(side: str, params: Params) -> dict:
  can_sock = messaging.sub_sock("can", timeout=100)
  sendcan = messaging.pub_sock("sendcan")
  params.put_bool(OFFROAD_ACTIVE_PARAM, True, block=True)
  try:
    if not _wait_offroad_safety(True):
      raise RuntimeError("Panda 未进入受限氛围灯测试模式")

    controller = AmbientLightingController()
    capture_deadline = time.monotonic() + 2.0
    while time.monotonic() < capture_deadline and ADDRESS not in controller.frames:
      event = messaging.recv_one_or_none(can_sock)
      if event is not None:
        for frame in event.can:
          controller.observe_frame(event.logMonoTime, int(frame.address), bytes(frame.dat), int(frame.src))
    if ADDRESS not in controller.frames:
      raise RuntimeError("未收到原车 0x679 氛围灯模板")

    test_id = uuid.uuid4().hex
    controller.pending = {"id": test_id, "side": side, "created_ns": time.monotonic_ns()}
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
      event = messaging.recv_one_or_none(can_sock)
      if event is not None:
        for frame in event.can:
          controller.observe_frame(event.logMonoTime, int(frame.address), bytes(frame.dat), int(frame.src))
      for can_data in controller.take_can_sends(time.monotonic_ns()):
        sendcan.send(can_list_to_can_capnp([can_data], msgtype="sendcan"))
      if controller.status is not None:
        return controller.status
    return {"id": test_id, "state": "timeout", "message": "测试超时，不能确认发送"}
  finally:
    params.put_bool(OFFROAD_ACTIVE_PARAM, False, block=True)
    if not _wait_offroad_safety(False):
      raise RuntimeError("Panda 未恢复 noOutput；请重启设备")


def run_ambient_test(side: str) -> dict:
  if not isinstance(side, str) or side not in TARGETS:
    raise ValueError("只支持 left / right，颜色固定为红色")
  if not _LOCK.acquire(blocking=False):
    raise RuntimeError("已有氛围灯测试正在运行")
  try:
    snapshot = driving_status_snapshot()
    if not snapshot.get("ambient_test_available"):
      raise RuntimeError("请先唤醒车辆并开启原车氛围灯")
    params = Params()
    if not snapshot.get("onroad"):
      return _run_offroad_test(side, params)
    if not snapshot.get("ambient_test_ready"):
      raise RuntimeError("车辆控制进程尚未准备好")
    test_id = uuid.uuid4().hex
    params.put(REQUEST_PARAM, json.dumps({"id": test_id, "side": side, "created_ns": time.monotonic_ns()}))
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
      raw = params.get(STATUS_PARAM)
      if raw:
        try:
          result = json.loads(raw)
          if isinstance(result, dict) and result.get("id") == test_id:
            return result
        except (ValueError, TypeError):
          pass
      time.sleep(0.05)
    return {"id": test_id, "state": "timeout", "message": "测试超时，不能确认发送；请检查 card 和 Panda 状态"}
  finally:
    _LOCK.release()
