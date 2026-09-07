import pytest

from openpilot.sunnypilot.system.update_hooks import (
  LOCAL_UPDATE_BRANCHES, LOCAL_UPDATE_URL, available_update_branches,
  hydrate_lfs_checkout, prepare_update_remote,
)


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


@pytest.mark.parametrize("current", LOCAL_UPDATE_BRANCHES)
@pytest.mark.parametrize("cached", ("", "dev-sp-egpu", "master,dev-sp-egpu,master"))
def test_local_branch_choices_survive_empty_or_stale_cache(current, cached):
  choices = available_update_branches(current, cached)
  assert choices[0] == current
  assert set(LOCAL_UPDATE_BRANCHES) <= set(choices)
  assert len(choices) == len(set(choices))


def test_other_projects_keep_their_branch_list():
  assert available_update_branches("master", "master,dev") == ["master", "dev"]
  calls = []
  prepare_update_remote("/staging", "master", lambda *args: calls.append(args))
  assert calls == []


@pytest.mark.parametrize("remotes,action", (("", "add"), ("origin\nother\n", "set-url")))
def test_local_update_uses_public_origin_even_for_flattened_deployment(remotes, action):
  calls = []
  def run(cmd, cwd):
    calls.append(cmd)
    return remotes if cmd == ["git", "remote"] else ""
  prepare_update_remote("/staging", "dev-sp-egpu-prebuild", run)
  assert calls[-1] == ["git", "remote", action, "origin", LOCAL_UPDATE_URL]


def test_git_network_timeout_becomes_reportable_update_failure(monkeypatch):
  import subprocess
  from openpilot.system.updated import updated
  def stalled(cmd, **kwargs):
    assert kwargs["timeout"] == 30
    assert kwargs["env"]["GIT_TERMINAL_PROMPT"] == "0"
    raise subprocess.TimeoutExpired(cmd, 30, output=b"connecting")
  monkeypatch.setattr(subprocess, "check_output", stalled)
  with pytest.raises(subprocess.CalledProcessError) as exc:
    updated.run(["git", "ls-remote", "--heads", "origin"], "/staging")
  assert exc.value.returncode == 124
  assert "DNS" in exc.value.output


def test_update_check_uses_explicit_origin_and_publishes_all_local_heads(monkeypatch):
  from types import SimpleNamespace
  from openpilot.system.updated import updated
  updater = updated.Updater()
  updater.params = SimpleNamespace(get=lambda key: "navassist-track-p0")
  calls = []
  def run(cmd, cwd):
    calls.append(cmd)
    if cmd[:2] == ["git", "ls-remote"]:
      assert cmd == ["git", "ls-remote", "--heads", "origin"]
      return "\n".join(f"{'1' * 40}\trefs/heads/{b}" for b in LOCAL_UPDATE_BRANCHES)
    if cmd == ["git", "remote"]:
      return "origin"
    if cmd == ["git", "rev-parse", "--abbrev-ref", "HEAD"]:
      return "dev-sp-egpu"
    return "1" * 40
  monkeypatch.setattr(updated, "run", run)
  updater.check_for_update()
  assert set(updater.branches) == set(LOCAL_UPDATE_BRANCHES)
  assert updater.has_internet


def test_custom_submodule_urls_match_the_repositories_containing_local_commits():
  import subprocess
  from openpilot.common.basedir import BASEDIR
  from pathlib import Path
  config = Path(BASEDIR) / ".gitmodules"
  if not config.exists():
    pytest.skip("flattened prebuild has no submodules")
  for name, repo in (("panda", "panda"), ("opendbc", "opendbc")):
    url = subprocess.check_output(["git", "config", "-f", str(config), f"submodule.{name}.url"], text=True).strip()
    assert url == f"https://github.com/onemiless/{repo}.git"


@pytest.mark.parametrize("start,target", [(a, b) for a in LOCAL_UPDATE_BRANCHES for b in LOCAL_UPDATE_BRANCHES if a != b])
def test_fetch_switches_between_flattened_prebuild_and_source_submodules(tmp_path, monkeypatch, start, target):
  import subprocess
  from types import SimpleNamespace
  from openpilot.system.updated import updated
  from openpilot.sunnypilot.system import update_hooks
  monkeypatch.setenv("GIT_ALLOW_PROTOCOL", "file")
  def git(path, *args):
    return subprocess.check_output(["git", "-c", "user.name=Test", "-c", "user.email=test@example.com", *args],
                                   cwd=path, stderr=subprocess.STDOUT, text=True)
  sub = tmp_path / "sub"
  sub.mkdir()
  git(sub, "init")
  (sub / "code.py").write_text("value = 1\n")
  git(sub, "add", ".")
  git(sub, "commit", "-m", "submodule")
  origin = tmp_path / "origin"
  origin.mkdir()
  git(origin, "init", "-b", "dev-sp-egpu-prebuild")
  (origin / "prebuilt").touch()
  (origin / "module").mkdir()
  (origin / "module/code.py").write_text("value = 1\n")
  git(origin, "add", ".")
  git(origin, "commit", "-m", "flattened")
  git(origin, "checkout", "-b", "dev-sp-egpu")
  git(origin, "rm", "-r", "prebuilt", "module")
  git(origin, "submodule", "add", str(sub), "module")
  git(origin, "commit", "-am", "source")
  git(origin, "branch", "navassist-track-p0")
  staging = tmp_path / "staging"
  git(tmp_path, "clone", "--recurse-submodules", "-b", start, str(origin), str(staging))
  monkeypatch.setattr(update_hooks, "LOCAL_UPDATE_URL", str(origin))
  monkeypatch.setattr(updated, "OVERLAY_MERGED", str(staging))
  monkeypatch.setattr(updated, "AGNOS", False)
  monkeypatch.setattr(updated, "set_consistent_flag", lambda value: None)
  monkeypatch.setattr(updated, "finalize_update", lambda: None)
  updater = updated.Updater()
  updater.params = SimpleNamespace(get=lambda key: target, put=lambda *a, **kw: None, put_bool=lambda *a, **kw: None)
  updater.fetch_update()
  assert git(staging, "branch", "--show-current").strip() == target
  assert (staging / "module/code.py").read_text() == "value = 1\n"
  assert (staging / "prebuilt").exists() == (target == "dev-sp-egpu-prebuild")
  assert (staging / ".gitmodules").exists() == (target != "dev-sp-egpu-prebuild")
