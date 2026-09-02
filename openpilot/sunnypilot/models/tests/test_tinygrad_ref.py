import requests
import subprocess

from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.models.tinygrad_ref import get_tinygrad_ref
from openpilot.sunnypilot.models.fetcher import ModelFetcher
from openpilot.common.test import OpenpilotTestCase

def fetch_tinygrad_ref():
  response = requests.get(ModelFetcher.MODEL_URL, timeout=10)
  response.raise_for_status()
  json_data = response.json()
  return json_data.get("tinygrad_ref")


class TestTinygradRef(OpenpilotTestCase):
  def test_tinygrad_ref(self):
    current_ref = get_tinygrad_ref()
    remote_ref = fetch_tinygrad_ref()
    repo = f"{BASEDIR}/tinygrad_repo"
    ancestry = subprocess.run(["git", "merge-base", "--is-ancestor", remote_ref, current_ref], cwd=repo, check=False)
    assert ancestry.returncode == 0, (
      f"""tinygrad_repo does not contain the ref used to compile the current driving models.
    Current: {current_ref}
    Remote: {remote_ref}
    Please run build-all workflow to update models."""
    )
    approved_runtime_patch = {
      "test/unit/test_buffer_initial_value.py",
      "tinygrad/device.py",
      "tinygrad/runtime/ops_amd.py",
      "tinygrad/runtime/support/am/amdev.py",
      "tinygrad/runtime/support/memory.py",
    }
    changed = set(subprocess.check_output(
      ["git", "diff", "--name-only", f"{remote_ref}..{current_ref}"], cwd=repo, text=True,
    ).splitlines())
    assert changed <= approved_runtime_patch, f"unreviewed tinygrad runtime changes: {sorted(changed - approved_runtime_patch)}"
