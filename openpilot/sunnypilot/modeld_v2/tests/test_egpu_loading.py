import threading
import unittest

from openpilot.sunnypilot.modeld_v2.egpu_loader import (
  C3XL_MODEL_LOAD_TIMEOUT, EgpuModelLoadError, configure_default_device, load_with_timeout,
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
