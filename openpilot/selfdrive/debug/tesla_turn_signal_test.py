#!/usr/bin/env python3
import argparse
import sys
import time
import uuid

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.validation_controller import (
  CANCEL_PARAM,
  RESULT_PARAM,
  REQUEST_PARAM,
  STATUS_PARAM,
  VALIDATION_LOG_PATH,
)


def start_validation_session(direction: str, params: Params | None = None) -> str:
  if direction not in ("left", "right"):
    raise RuntimeError(f"unsupported turn request: {direction}")
  params = Params() if params is None else params
  if not params.get_bool("TeslaTurnSignalValidation"):
    raise RuntimeError("TeslaTurnSignalValidation is disabled; enable it offroad and restart")

  test_id = uuid.uuid4().hex[:12]
  params.put(REQUEST_PARAM, {
    "test_id": test_id,
    "direction": direction,
    "wall_time_ns": time.time_ns(),
  })
  return test_id


def get_validation_status(test_id: str, params: Params | None = None) -> dict:
  params = Params() if params is None else params
  result = params.get(RESULT_PARAM)
  if result is not None and result.get("test_id") == test_id:
    return {"done": True, **result}
  status = params.get(STATUS_PARAM)
  if status is not None and status.get("test_id") == test_id:
    return {"done": False, **status}
  return {"done": False, "test_id": test_id, "phase": "queued"}


def cancel_validation_session(test_id: str, params: Params | None = None) -> None:
  params = Params() if params is None else params
  params.put(CANCEL_PARAM, {"test_id": test_id, "wall_time_ns": time.time_ns()})


def send_validation_pulse(direction: str, params: Params | None = None, timeout_s: float = 15.0,
                          poll_interval_s: float = 0.05) -> bool:
  params = Params() if params is None else params
  test_id = start_validation_session(direction, params)
  deadline = time.monotonic() + timeout_s
  while time.monotonic() < deadline:
    result = get_validation_status(test_id, params)
    if result.get("done"):
      status = str(result.get("result", "UNKNOWN"))
      if status == "PASS":
        print(f"PASS: vehicle reports {direction} blinker; log={VALIDATION_LOG_PATH}; test_id={test_id}")
        return True
      print(f"FAIL: {status}; log={VALIDATION_LOG_PATH}; test_id={test_id}")
      return False
    time.sleep(poll_interval_s)

  raise RuntimeError("card realtime turn-signal controller did not respond before timeout")


def main() -> int:
  parser = argparse.ArgumentParser(description="Tesla realtime DAS_bodyControls turn-signal validation")
  parser.add_argument("direction", choices=("left", "right"))
  args = parser.parse_args()
  try:
    return 0 if send_validation_pulse(args.direction) else 2
  except RuntimeError as error:
    print(f"BLOCKED: {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
