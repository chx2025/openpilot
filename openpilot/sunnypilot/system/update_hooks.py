import re
from collections.abc import Callable
from pathlib import Path


CommandRunner = Callable[[list[str], str | Path], str]


def hydrate_lfs_checkout(cwd: str | Path, run: CommandRunner) -> None:
  """Fail finalization while any tracked LFS file is still a pointer."""
  run(["git", "lfs", "checkout"], cwd)
  lfs_files = run(["git", "lfs", "ls-files"], cwd)
  missing = [line.split(" - ", 1)[1] for line in lfs_files.splitlines() if re.fullmatch(r"[0-9a-f]+ - .+", line)]
  if missing:
    preview = ", ".join(missing[:5])
    suffix = "" if len(missing) <= 5 else f" (+{len(missing) - 5} more)"
    raise RuntimeError(f"LFS objects unavailable for finalized update: {preview}{suffix}")
