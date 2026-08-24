import numpy as np

from openpilot.selfdrive.modeld.helpers import dump_oob, load_oob


def test_load_oob_reports_monotonic_byte_progress(tmp_path):
  path = tmp_path / "model.pkl"
  expected = {"weights": np.arange(256 * 1024, dtype=np.float32), "metadata": {"name": "test"}}
  with path.open("wb") as f:
    dump_oob(expected, f)

  progress = []
  with path.open("rb") as f:
    actual = load_oob(f, total_size=path.stat().st_size, progress_callback=progress.append)

  assert np.array_equal(actual["weights"], expected["weights"])
  assert actual["metadata"] == expected["metadata"]
  assert progress
  assert progress == sorted(progress)
  assert progress[-1] == 1.0
