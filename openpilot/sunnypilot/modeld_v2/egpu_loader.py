import os
import threading
from collections.abc import Callable, MutableMapping


# Measured on the C3XL/UT3G device on 2026-08-29:
# LM 71.50 s, BMV3 73.54 s, TT 74.66 s, BMV2 75.58 s.
# Keep a bounded 44.42 s margin for cold starts and USB scheduling variance.
C3XL_MODEL_LOAD_TIMEOUT = 120
C3XL_TINYGRAD_CACHE_HOME = "/data/cache"
C3XL_AM_POWER_LIMIT_W = 100
C3XL_AMD_USB_POLL_US = 100


class EgpuModelLoadError(RuntimeError):
  pass


def configure_default_device(comma_hardware: bool, environment: MutableMapping[str, str] = os.environ, *, c3xl: bool = False) -> None:
  """Prevent tinygrad's default-device scan from probing the USB AMD GPU."""
  if comma_hardware:
    environment.setdefault("DEV", "QCOM")
  if c3xl:
    # /home is an ephemeral overlay on C3XL. Keep AMD firmware and compiler
    # caches across reboots so model startup never depends on a live download.
    environment.setdefault("XDG_CACHE_HOME", C3XL_TINYGRAD_CACHE_HOME)
    # Limit the volatile SMU PPT before clocks are opened up. An explicit
    # environment override remains available for controlled testing.
    environment.setdefault("AM_POWER_LIMIT", str(C3XL_AM_POWER_LIMIT_W))
    # A3/BMV2 at 100 W: 100 us lowers completion latency without the CPU and
    # USB traffic cost of busy polling. Keep tinygrad's other devices unchanged.
    environment.setdefault("AMD_USB_POLL_US", str(C3XL_AMD_USB_POLL_US))


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
