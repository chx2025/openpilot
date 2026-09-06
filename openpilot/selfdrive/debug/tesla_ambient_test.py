"""Local console entry point for card-owned ambient lighting tests."""
import json
import threading
import time
import uuid

from openpilot.common.params import Params
from openpilot.selfdrive.debug.driving_status import driving_status_snapshot
from openpilot.sunnypilot.selfdrive.car.tesla.ambient_lighting import REQUEST_PARAM, STATUS_PARAM, TARGETS

_LOCK = threading.Lock()


def run_ambient_test(side: str) -> dict:
  if not isinstance(side, str) or side not in TARGETS:
    raise ValueError("只支持 left / right，颜色固定为红色")
  if not _LOCK.acquire(blocking=False):
    raise RuntimeError("已有氛围灯测试正在运行")
  try:
    if not driving_status_snapshot().get("ambient_test_ready"):
      raise RuntimeError("请唤醒车辆并保持 P 挡静止，开启原车氛围灯；需要 card 运行及 Tesla safety 模式")
    params = Params()
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
