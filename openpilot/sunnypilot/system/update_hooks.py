import re
from collections.abc import Callable
from pathlib import Path


CommandRunner = Callable[[list[str], str | Path], str]

LOCAL_UPDATE_BRANCHES = ("dev-sp-egpu", "dev-sp-egpu-prebuild", "navassist-track-p0")
LOCAL_UPDATE_URL = "https://github.com/onemiless/openpilot.git"


def available_update_branches(current: str, cached: str) -> list[str]:
  branches = list(dict.fromkeys(branch for branch in cached.split(",") if branch))
  if current in LOCAL_UPDATE_BRANCHES:
    return list(dict.fromkeys((current, *LOCAL_UPDATE_BRANCHES, *branches)))
  return branches


def prepare_update_remote(cwd: str | Path, current: str, run: CommandRunner) -> None:
  """Configure the staging checkout, including flattened device deployments."""
  if current not in LOCAL_UPDATE_BRANCHES:
    return
  remotes = run(["git", "remote"], cwd).splitlines()
  if "origin" in remotes:
    run(["git", "remote", "set-url", "origin", LOCAL_UPDATE_URL], cwd)
  else:
    run(["git", "remote", "add", "origin", LOCAL_UPDATE_URL], cwd)


def update_command_timeout(cmd: list[str]) -> int | None:
  if cmd[:2] == ["git", "ls-remote"]:
    return 30
  if cmd[:2] == ["git", "fetch"] or cmd[:3] == ["git", "submodule", "update"] or cmd[:2] == ["git", "lfs"]:
    return 600
  return None


def hydrate_lfs_checkout(cwd: str | Path, run: CommandRunner) -> None:
  """Fail finalization while any tracked LFS file is still a pointer."""
  run(["git", "lfs", "checkout"], cwd)
  lfs_files = run(["git", "lfs", "ls-files"], cwd)
  missing = [line.split(" - ", 1)[1] for line in lfs_files.splitlines() if re.fullmatch(r"[0-9a-f]+ - .+", line)]
  if missing:
    preview = ", ".join(missing[:5])
    suffix = "" if len(missing) <= 5 else f" (+{len(missing) - 5} more)"
    raise RuntimeError(f"LFS objects unavailable for finalized update: {preview}{suffix}")
