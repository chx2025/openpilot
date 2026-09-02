"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import hashlib
import os
from pathlib import Path
import subprocess

from openpilot.sunnypilot import get_file_hash
from openpilot.sunnypilot.models import default_model
from openpilot.sunnypilot.models.default_model import BIG_MODEL_HASH_PATH, BIG_SUPERCOMBO_ONNX_PATH, MODEL_HASH_PATH, SUPERCOMBO_ONNX_PATH
from openpilot.common.test import OpenpilotTestCase


class TestDefaultModel(OpenpilotTestCase):
  def test_compare_onnx_hashes(self):
    supercombo_hash = get_file_hash(SUPERCOMBO_ONNX_PATH)

    combined_hash = hashlib.sha256(supercombo_hash.encode()).hexdigest()

    with open(MODEL_HASH_PATH) as f:
      current_hash = f.read().strip()

    assert combined_hash == current_hash, "Run openpilot/sunnypilot/models/default_model.py to update the default model name and hash"

  def test_compare_big_onnx_hashes(self):
    if not os.path.exists(BIG_SUPERCOMBO_ONNX_PATH):
      self.skipTest("big_driving_supercombo.onnx not present")

    relative_path = os.path.relpath(BIG_SUPERCOMBO_ONNX_PATH, os.getcwd())
    pointer = subprocess.check_output(["git", "show", f"HEAD:{relative_path}"], text=True)
    oid = next(line.split(":", 1)[1] for line in pointer.splitlines() if line.startswith("oid sha256:"))
    combined_hash = hashlib.sha256(oid.encode()).hexdigest()

    with open(BIG_MODEL_HASH_PATH) as f:
      current_hash = f.read().strip()

    assert combined_hash == current_hash, "Run openpilot/sunnypilot/models/default_model.py to update the default big model hash"


def test_update_model_hash_tracks_big_model_lfs_content(tmp_path, monkeypatch):
  small_model = tmp_path / "driving_supercombo.onnx"
  big_model = tmp_path / "big_driving_supercombo.onnx"
  small_hash = tmp_path / "model_hash"
  big_hash = tmp_path / "big_model_hash"
  lfs_oid = "1" * 64

  small_model.write_bytes(b"small model")
  big_model.write_text("".join((
    "version https://git-lfs.github.com/spec/v1\n",
    f"oid sha256:{lfs_oid}\n",
    "size 123\n",
  )))
  subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
  subprocess.run(["git", "add", big_model.name], cwd=tmp_path, check=True)
  subprocess.run(["git", "-c", "user.name=test", "-c", "user.email=test@example.com", "commit", "-qm", "model"],
                 cwd=tmp_path, check=True)

  monkeypatch.chdir(tmp_path)
  monkeypatch.setattr(default_model, "SUPERCOMBO_ONNX_PATH", str(small_model))
  monkeypatch.setattr(default_model, "BIG_SUPERCOMBO_ONNX_PATH", str(big_model), raising=False)
  monkeypatch.setattr(default_model, "MODEL_HASH_PATH", str(small_hash))
  monkeypatch.setattr(default_model, "BIG_MODEL_HASH_PATH", str(big_hash), raising=False)

  default_model.update_model_hash()

  expected_small = hashlib.sha256(get_file_hash(str(small_model)).encode()).hexdigest()
  assert small_hash.read_text() == expected_small
  assert big_hash.read_text() == hashlib.sha256(lfs_oid.encode()).hexdigest()
  assert Path(default_model.BIG_MODEL_HASH_PATH) == big_hash
