import pytest

from openpilot.sunnypilot.system.update_hooks import hydrate_lfs_checkout


class FakeRunner:
  def __init__(self, listing):
    self.listing = listing
    self.calls = []

  def __call__(self, command, cwd):
    self.calls.append((command, cwd))
    return self.listing if command[-1] == "ls-files" else ""


def test_hydrate_lfs_checkout_accepts_materialized_files():
  run = FakeRunner("abc123 * model.onnx\ndef456 * font.ttf")
  hydrate_lfs_checkout("/checkout", run)
  assert run.calls == [
    (["git", "lfs", "checkout"], "/checkout"),
    (["git", "lfs", "ls-files"], "/checkout"),
  ]


def test_hydrate_lfs_checkout_rejects_pointer_files():
  run = FakeRunner("abc123 - model.onnx\ndef456 * font.ttf")
  with pytest.raises(RuntimeError, match="model.onnx"):
    hydrate_lfs_checkout("/checkout", run)
