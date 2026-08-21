import threading
import unittest

from openpilot.sunnypilot.modeld_v2.egpu_loader import EgpuModelLoadError, configure_default_device, load_with_timeout, wait_for_link


class TestEgpuLoading(unittest.TestCase):
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

  def test_waits_for_pcie_link(self):
    checks = iter((False, False, True))
    delays = []
    self.assertTrue(wait_for_link(lambda: next(checks), attempts=3, delay_fn=delays.append))
    self.assertEqual(delays, [1.0, 1.0])

  def test_reports_pcie_link_failure(self):
    delays = []
    self.assertFalse(wait_for_link(lambda: False, attempts=3, delay_fn=delays.append))
    self.assertEqual(delays, [1.0, 1.0])
