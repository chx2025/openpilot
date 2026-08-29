import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "benchmark_chestnut_model_load.py"
SPEC = importlib.util.spec_from_file_location("benchmark_chestnut_model_load", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_model_load_limit_is_inclusive_at_45_seconds():
  assert MODULE.within_limit(44.999999999, 45.0)
  assert MODULE.within_limit(45.0, 45.0)
  assert not MODULE.within_limit(45.000000001, 45.0)


def test_benchmark_uses_current_chestnut_api_and_functional_output_gate():
  source = SCRIPT.read_text()
  assert "chestnut=True" in source
  assert "usbgpu=True" not in source
  assert 'report["finite"]' in source
  assert 'report["plan_present"]' in source
  assert 'trace["bulk_fail"] == 0' in source
