import os
import threading
from collections.abc import Callable, MutableMapping


class EgpuModelLoadError(RuntimeError):
  pass


def configure_default_device(comma_hardware: bool, environment: MutableMapping[str, str] = os.environ) -> None:
  """Prevent tinygrad's default-device scan from probing the USB AMD GPU."""
  if comma_hardware:
    environment.setdefault("DEV", "QCOM")


def load_with_timeout[T](load: Callable[[], T], timeout: float) -> T:
  result: list[T] = []
  error: list[Exception] = []
  done = threading.Event()

  def run() -> None:
    try:
      result.append(load())
    except Exception as e:
      error.append(e)
    finally:
      done.set()

  threading.Thread(target=run, name="egpu-model-loader", daemon=True).start()
  if not done.wait(timeout):
    raise TimeoutError(f"eGPU model load timed out after {timeout:g}s")
  if error:
    raise EgpuModelLoadError(f"eGPU model load failed: {error[0]}") from error[0]
  return result[0]
