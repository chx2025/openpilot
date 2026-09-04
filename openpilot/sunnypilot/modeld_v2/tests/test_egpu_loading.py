import threading
import unittest

from openpilot.sunnypilot.modeld_v2.egpu_loader import (
  C3XL_AM_POWER_LIMIT_W, C3XL_MODEL_LOAD_TIMEOUT, C3XL_TINYGRAD_CACHE_HOME, EgpuModelLoadError, configure_default_device, load_with_timeout,
)


class TestEgpuLoading(unittest.TestCase):
  def test_timeout_covers_measured_c3xl_model_loads(self):
    measured_max_seconds = 75.58
    self.assertEqual(C3XL_MODEL_LOAD_TIMEOUT, 120)
    self.assertGreaterEqual(C3XL_MODEL_LOAD_TIMEOUT, measured_max_seconds * 1.5)

  def test_configures_qcom_default_without_overriding_explicit_device(self):
    environment = {}
    configure_default_device(True, environment)
    self.assertEqual(environment["DEV"], "QCOM")

    environment = {"DEV": "CPU"}
    configure_default_device(True, environment)
    self.assertEqual(environment["DEV"], "CPU")

  def test_configures_persistent_c3xl_tinygrad_cache_without_overriding_explicit_path(self):
    environment = {}
    configure_default_device(True, environment, c3xl=True)
    self.assertEqual(environment["XDG_CACHE_HOME"], C3XL_TINYGRAD_CACHE_HOME)

    environment = {"XDG_CACHE_HOME": "/custom/cache"}
    configure_default_device(True, environment, c3xl=True)
    self.assertEqual(environment["XDG_CACHE_HOME"], "/custom/cache")

  def test_c3xl_defaults_amd_power_limit_to_100w_without_overriding_explicit_value(self):
    environment = {}
    configure_default_device(True, environment, c3xl=True)
    self.assertEqual(C3XL_AM_POWER_LIMIT_W, 100)
    self.assertEqual(environment["AM_POWER_LIMIT"], "100")

    environment = {"AM_POWER_LIMIT": "85"}
    configure_default_device(True, environment, c3xl=True)
    self.assertEqual(environment["AM_POWER_LIMIT"], "85")

  def test_standard_hardware_does_not_set_amd_power_limit(self):
    environment = {}
    configure_default_device(True, environment, c3xl=False)
    self.assertNotIn("AM_POWER_LIMIT", environment)

  def test_propagates_loader_exception(self):
    original = RuntimeError("USB AMD initialization failed")

    def load():
      raise original

    with self.assertRaisesRegex(EgpuModelLoadError, "USB AMD initialization failed") as ctx:
      load_with_timeout(load, 1.0)
    self.assertIs(ctx.exception.__cause__, original)

  def test_distinguishes_timeout_from_loader_exception(self):
    release = threading.Event()

    def load():
      release.wait()

    try:
      with self.assertRaisesRegex(TimeoutError, "0.01s"):
        load_with_timeout(load, 0.01)
    finally:
      release.set()

  def test_returns_loaded_model(self):
    model = object()
    self.assertIs(load_with_timeout(lambda: model, 1.0), model)
