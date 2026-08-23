#!/usr/bin/env python3
"""Conservative daily sync of low-risk sunnypilot upstream commits.

The tool deliberately cherry-picks onto the maintained branch. It never rebases,
force-pushes, deploys a device, or replaces an opendbc submodule pointer. Runtime
and vehicle-facing changes are emitted for human review instead of being applied.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


AUTO_PREFIXES = (
  "docs/",
  ".github/ISSUE_TEMPLATE/",
)
AUTO_EXACT = {
  "README.md",
  "LICENSE",
  "LICENSE.md",
  ".github/PULL_REQUEST_TEMPLATE.md",
}

REVIEW_PREFIXES = (
  ".github/workflows/",
  "opendbc_repo",
  "panda",
  "tinygrad_repo",
  "msgq_repo",
  "rednose_repo",
  "openpilot/cereal/",
  "openpilot/common/",
  "openpilot/selfdrive/",
  "openpilot/sunnypilot/",
  "openpilot/system/",
  "release/",
)
REVIEW_EXACT = {
  ".gitattributes",
  ".gitignore",
  ".gitmodules",
  "SConstruct",
  "hardware_profile",
  "pyproject.toml",
  "uv.lock",
}


class SyncError(RuntimeError):
  pass


@dataclass
class CommitAudit:
  sha: str
  subject: str
  paths: list[str]
  statuses: list[str]
  equivalent: bool = False
  classification: str = "review"
  reasons: list[str] = field(default_factory=list)


@dataclass
class FileAudit:
  path: str
  upstream_status: str = ""
  local_status: str = ""
  comparison_status: str = ""
  additions: int | None = None
  deletions: int | None = None
  binary: bool = False
  classification: str = "review_upstream"
  conclusion: str = ""
  reasons: list[str] = field(default_factory=list)


@dataclass
class SyncReport:
  base_sha: str
  upstream_sha: str
  upstream_ref: str
  target_branch: str
  commits: list[CommitAudit]
  generated_at: str = field(default_factory=lambda: datetime.now().astimezone().isoformat(timespec="seconds"))
  merge_base_sha: str | None = None
  comparison_sha: str | None = None
  files: list[FileAudit] = field(default_factory=list)
  auto_applied: list[str] = field(default_factory=list)
  result: str = "audit_only"
  candidate_sha: str | None = None
  error: str | None = None


def _run(args: list[str], cwd: Path, *, check: bool = True) -> subprocess.CompletedProcess[str]:
  result = subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=False)
  if check and result.returncode != 0:
    command = " ".join(args)
    raise SyncError(f"{command} failed ({result.returncode}): {result.stderr.strip() or result.stdout.strip()}")
  return result


def _git(repo: Path, *args: str, check: bool = True) -> str:
  return _run(["git", *args], repo, check=check).stdout.strip()


def _is_auto_path(path: str) -> bool:
  return path in AUTO_EXACT or path.startswith(AUTO_PREFIXES)


def _is_review_path(path: str) -> bool:
  return path in REVIEW_EXACT or path.startswith(REVIEW_PREFIXES)


def classify_commit(commit: CommitAudit, *, has_binary: bool = False, is_merge: bool = False) -> CommitAudit:
  if commit.equivalent:
    commit.classification = "equivalent"
    return commit

  reasons: list[str] = []
  if is_merge:
    reasons.append("merge commit")
  if has_binary:
    reasons.append("binary change")
  if any(status.startswith(("D", "R", "C")) for status in commit.statuses):
    reasons.append("delete/rename/copy requires review")

  review_paths = [path for path in commit.paths if _is_review_path(path)]
  if review_paths:
    reasons.append("sensitive path: " + ", ".join(review_paths[:5]))

  non_auto_paths = [path for path in commit.paths if not _is_auto_path(path)]
  if non_auto_paths:
    reasons.append("outside automatic allowlist: " + ", ".join(non_auto_paths[:5]))

  if not commit.paths:
    reasons.append("empty or merge-only patch")

  commit.reasons = reasons
  commit.classification = "auto" if not reasons else "review"
  return commit


def enforce_contiguous_prefix(commits: Iterable[CommitAudit]) -> None:
  blocked = False
  for commit in commits:
    if commit.equivalent or commit.classification == "accepted":
      continue
    if blocked and commit.classification == "auto":
      commit.classification = "blocked"
      commit.reasons.append("earlier upstream commit requires human review")
    elif commit.classification == "review":
      blocked = True


def _changed_files(repo: Path, sha: str) -> tuple[list[str], list[str], bool]:
  raw = _git(repo, "diff-tree", "--root", "--no-commit-id", "--name-status", "-r", "-M", sha)
  paths: list[str] = []
  statuses: list[str] = []
  for line in raw.splitlines():
    fields = line.split("\t")
    if len(fields) < 2:
      continue
    statuses.append(fields[0])
    paths.extend(fields[1:])

  numstat = _git(repo, "diff-tree", "--root", "--no-commit-id", "--numstat", "-r", sha)
  has_binary = any(line.split("\t", 2)[:2] == ["-", "-"] for line in numstat.splitlines())
  return paths, statuses, has_binary


def _accepted_decision(repo: Path, sha: str, ledger: dict[str, dict[str, object]]) -> str | None:
  decision = ledger.get(sha)
  if not decision:
    return None
  for local_sha in decision.get("local_commits", []):
    result = _run(["git", "merge-base", "--is-ancestor", str(local_sha), "HEAD"], repo, check=False)
    if result.returncode != 0:
      return None
  return str(decision.get("reason", "previously reviewed"))


def _diff_statuses(repo: Path, old: str, new: str) -> dict[str, str]:
  raw = _git(repo, "diff", "--name-status", "-M", old, new)
  statuses: dict[str, str] = {}
  for line in raw.splitlines():
    fields = line.split("\t")
    if len(fields) < 2:
      continue
    status = fields[0]
    path = fields[-1]
    statuses[path] = status
  return statuses


def _diff_numstat(repo: Path, old: str, new: str) -> dict[str, tuple[int | None, int | None, bool]]:
  raw = _git(repo, "diff", "--numstat", old, new)
  stats: dict[str, tuple[int | None, int | None, bool]] = {}
  for line in raw.splitlines():
    fields = line.split("\t")
    if len(fields) < 3:
      continue
    added, deleted, path = fields[0], fields[1], fields[-1]
    binary = added == "-" or deleted == "-"
    stats[path] = (None if binary else int(added), None if binary else int(deleted), binary)
  return stats


def _blob_oid(repo: Path, revision: str, path: str) -> str | None:
  result = _run(["git", "rev-parse", f"{revision}:{path}"], repo, check=False)
  return result.stdout.strip() if result.returncode == 0 else None


def classify_file(file: FileAudit, *, content_equal: bool) -> FileAudit:
  reasons: list[str] = []
  if file.binary:
    reasons.append("binary or submodule-sized change")
  if _is_review_path(file.path):
    reasons.append("sensitive runtime/dependency path")

  if content_equal:
    file.classification = "equivalent"
    file.conclusion = "当前分支与官方最终内容相同，无需同步"
  elif file.upstream_status and file.local_status:
    file.classification = "manual_merge"
    file.conclusion = "官方与本地都修改，必须逐文件人工合并并保留本地功能"
  elif file.upstream_status:
    safe_status = file.upstream_status.startswith(("A", "M"))
    if _is_auto_path(file.path) and safe_status and not file.binary:
      file.classification = "safe_candidate"
      file.conclusion = "仅官方修改，属于低风险白名单，可作为安全同步候选"
    else:
      file.classification = "review_upstream"
      file.conclusion = "仅官方修改，但涉及代码/删除/依赖，需人工审核后同步"
  elif file.local_status:
    file.classification = "keep_local"
    file.conclusion = "仅本地修改，属于当前新增功能或本地取舍，应保留"
  else:
    file.classification = "equivalent"
    file.conclusion = "当前分支与官方无有效内容差异"

  file.reasons = reasons
  return file


def audit_files(repo: Path, upstream_ref: str) -> tuple[str, str, list[FileAudit]]:
  comparison_sha = _git(repo, "rev-parse", "HEAD")
  merge_base = _git(repo, "merge-base", comparison_sha, upstream_ref)
  upstream_statuses = _diff_statuses(repo, merge_base, upstream_ref)
  local_statuses = _diff_statuses(repo, merge_base, comparison_sha)
  comparison_statuses = _diff_statuses(repo, comparison_sha, upstream_ref)
  comparison_stats = _diff_numstat(repo, comparison_sha, upstream_ref)
  paths = sorted(set(upstream_statuses) | set(local_statuses) | set(comparison_statuses))

  files: list[FileAudit] = []
  for path in paths:
    additions, deletions, binary = comparison_stats.get(path, (0, 0, False))
    file = FileAudit(
      path=path,
      upstream_status=upstream_statuses.get(path, ""),
      local_status=local_statuses.get(path, ""),
      comparison_status=comparison_statuses.get(path, ""),
      additions=additions,
      deletions=deletions,
      binary=binary,
    )
    head_oid = _blob_oid(repo, comparison_sha, path)
    upstream_oid = _blob_oid(repo, upstream_ref, path)
    classify_file(file, content_equal=head_oid == upstream_oid)
    files.append(file)
  return merge_base, comparison_sha, files


def fetch_branch(repo: Path, remote: str, branch: str) -> None:
  refspec = f"+refs/heads/{branch}:refs/remotes/{remote}/{branch}"
  _git(repo, "fetch", "--no-tags", "--no-recurse-submodules", remote, refspec)


def audit(repo: Path, upstream_ref: str, target_branch: str,
          ledger: dict[str, dict[str, object]] | None = None) -> SyncReport:
  base_sha = _git(repo, "rev-parse", "HEAD")
  upstream_sha = _git(repo, "rev-parse", upstream_ref)
  cherry_lines = _git(repo, "cherry", "HEAD", upstream_ref).splitlines()
  commits: list[CommitAudit] = []

  for line in cherry_lines:
    if not line:
      continue
    marker, sha = line.split(maxsplit=1)
    subject = _git(repo, "show", "-s", "--format=%s", sha)
    paths, statuses, has_binary = _changed_files(repo, sha)
    parents = _git(repo, "show", "-s", "--format=%P", sha).split()
    commit = CommitAudit(sha=sha, subject=subject, paths=paths, statuses=statuses, equivalent=marker == "-")
    classify_commit(commit, has_binary=has_binary, is_merge=len(parents) > 1)
    if marker == "+" and (reason := _accepted_decision(repo, sha, ledger or {})) is not None:
      commit.classification = "accepted"
      commit.reasons = [reason]
    commits.append(commit)

  enforce_contiguous_prefix(commits)
  merge_base, comparison_sha, files = audit_files(repo, upstream_ref)
  return SyncReport(
    base_sha=base_sha,
    upstream_sha=upstream_sha,
    upstream_ref=upstream_ref,
    target_branch=target_branch,
    commits=commits,
    merge_base_sha=merge_base,
    comparison_sha=comparison_sha,
    files=files,
  )


def _validate_candidate(worktree: Path, base_sha: str) -> None:
  _git(worktree, "diff", "--check", f"{base_sha}..HEAD")


def apply_safe(repo: Path, report: SyncReport, target_remote: str) -> None:
  safe = [commit for commit in report.commits if commit.classification == "auto"]
  review = [commit for commit in report.commits if commit.classification in ("review", "blocked")]
  if not safe:
    report.result = "review_required" if review else "up_to_date"
    return

  branch = _git(repo, "branch", "--show-current")
  if branch != report.target_branch:
    raise SyncError(f"expected branch {report.target_branch}, found {branch or 'detached HEAD'}")
  if _git(repo, "status", "--porcelain", "--untracked-files=no"):
    raise SyncError("tracked worktree changes present")

  remote_ref = f"{target_remote}/{report.target_branch}"
  remote_sha = _git(repo, "rev-parse", remote_ref)
  if remote_sha != report.base_sha:
    raise SyncError(f"local HEAD {report.base_sha} differs from {remote_ref} {remote_sha}")

  temp_root = Path(tempfile.mkdtemp(prefix="sp-upstream-sync-"))
  candidate_sha: str | None = None
  try:
    _git(repo, "worktree", "add", "--detach", str(temp_root), report.base_sha)
    try:
      for commit in safe:
        _git(temp_root, "cherry-pick", "-x", commit.sha)
      _validate_candidate(temp_root, report.base_sha)
      candidate_sha = _git(temp_root, "rev-parse", "HEAD")
    except Exception:
      _git(temp_root, "cherry-pick", "--abort", check=False)
      raise
    finally:
      _git(repo, "worktree", "remove", str(temp_root), check=False)
  finally:
    if temp_root.exists():
      shutil.rmtree(temp_root)

  if candidate_sha is None:
    raise SyncError("candidate was not created")
  if _git(repo, "rev-parse", "HEAD") != report.base_sha:
    raise SyncError("target branch moved during validation")
  if _git(repo, "status", "--porcelain", "--untracked-files=no"):
    raise SyncError("tracked worktree changed during validation")

  _git(repo, "merge", "--ff-only", candidate_sha)
  _git(repo, "push", target_remote, f"HEAD:refs/heads/{report.target_branch}")
  report.auto_applied = [commit.sha for commit in safe]
  report.candidate_sha = candidate_sha
  report.result = "auto_synced_with_review_remaining" if review else "auto_synced"


def render_markdown(report: SyncReport) -> str:
  file_counts: dict[str, int] = {}
  for file in report.files:
    file_counts[file.classification] = file_counts.get(file.classification, 0) + 1
  lines = [
    "# sunnypilot upstream sync report",
    "",
    f"- Generated: `{report.generated_at}`",
    f"- Base: `{report.base_sha}`",
    f"- File comparison HEAD: `{report.comparison_sha or report.base_sha}`",
    f"- Merge base: `{report.merge_base_sha or 'unknown'}`",
    f"- Upstream: `{report.upstream_ref}` → `{report.upstream_sha}`",
    f"- Result: `{report.result}`",
    "",
    "## File-level conclusion summary",
    "",
    "| Classification | Files | Conclusion |",
    "|---|---:|---|",
    f"| `safe_candidate` | {file_counts.get('safe_candidate', 0)} | 仅官方修改，低风险同步候选 |",
    f"| `review_upstream` | {file_counts.get('review_upstream', 0)} | 仅官方修改，但需人工审核 |",
    f"| `manual_merge` | {file_counts.get('manual_merge', 0)} | 双方均修改，必须人工合并 |",
    f"| `keep_local` | {file_counts.get('keep_local', 0)} | 仅本地修改，应保留 |",
    f"| `equivalent` | {file_counts.get('equivalent', 0)} | 最终内容相同，无需同步 |",
    "",
  ]
  if report.error:
    lines.extend([f"- Error: `{report.error}`", ""])
  for title, classes in (
    ("Automatically applied", {"auto"}),
    ("Human review required", {"review", "blocked"}),
    ("Previously reviewed", {"accepted"}),
    ("Already equivalent", {"equivalent"}),
  ):
    selected = [commit for commit in report.commits if commit.classification in classes]
    lines.extend([f"## {title}", ""])
    if not selected:
      lines.append("- None")
    for commit in selected:
      reason = f" — {'; '.join(commit.reasons)}" if commit.reasons else ""
      lines.append(f"- `{commit.sha[:12]}` {commit.subject}{reason}")
      if commit.paths:
        lines.append("  - " + ", ".join(commit.paths[:12]))
    lines.append("")

  lines.extend(["## Per-file conclusions", ""])
  if not report.files:
    lines.extend(["- No file differences.", ""])
  else:
    lines.extend([
      "| File | Upstream | Local | HEAD→Upstream | Diff | Classification | Conclusion |",
      "|---|---|---|---|---:|---|---|",
    ])
    order = {"manual_merge": 0, "review_upstream": 1, "safe_candidate": 2, "keep_local": 3, "equivalent": 4}
    for file in sorted(report.files, key=lambda item: (order.get(item.classification, 9), item.path)):
      path = file.path.replace("|", "\\|")
      conclusion = file.conclusion.replace("|", "\\|")
      if file.reasons:
        conclusion += "；" + "；".join(file.reasons)
      diff = "binary" if file.binary else f"+{file.additions or 0}/-{file.deletions or 0}"
      lines.append("".join((
        f"| `{path}` | `{file.upstream_status or '-'} ` | `{file.local_status or '-'} ` | ",
        f"`{file.comparison_status or '-'} ` | {diff} | `{file.classification}` | {conclusion} |",
      )))
    lines.append("")
  return "\n".join(lines)


def _write_report(report: SyncReport, json_path: Path, markdown_path: Path) -> None:
  json_path.parent.mkdir(parents=True, exist_ok=True)
  markdown_path.parent.mkdir(parents=True, exist_ok=True)
  json_path.write_text(json.dumps(asdict(report), indent=2) + "\n")
  markdown_path.write_text(render_markdown(report) + "\n")


def _write_history(report: SyncReport, history_dir: Path) -> None:
  history_dir.mkdir(parents=True, exist_ok=True)
  stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
  _write_report(report, history_dir / f"{stamp}.json", history_dir / f"{stamp}.md")
  _write_report(report, history_dir / "latest.json", history_dir / "latest.md")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--repo", type=Path, default=Path.cwd())
  parser.add_argument("--upstream-remote", default="sunnypilot")
  parser.add_argument("--upstream-branch", default="master")
  parser.add_argument("--target-remote", default="onemiless-openpilot")
  parser.add_argument("--target-branch", default="dev-sp-egpu")
  parser.add_argument("--apply-safe", action="store_true")
  parser.add_argument("--skip-fetch", action="store_true")
  parser.add_argument("--accepted-ledger", type=Path, default=Path("tools/sunnypilot_upstream_sync_accepted.json"))
  parser.add_argument("--json-output", type=Path, default=Path(".git/upstream-sync/latest.json"))
  parser.add_argument("--markdown-output", type=Path, default=Path(".git/upstream-sync/latest.md"))
  parser.add_argument("--history-dir", type=Path, default=Path("artifacts/upstream-audit"))
  args = parser.parse_args()

  repo = args.repo.resolve()
  upstream_ref = f"{args.upstream_remote}/{args.upstream_branch}"
  report: SyncReport | None = None
  try:
    if not args.skip_fetch:
      fetch_branch(repo, args.upstream_remote, args.upstream_branch)
      fetch_branch(repo, args.target_remote, args.target_branch)
    ledger_path = args.accepted_ledger if args.accepted_ledger.is_absolute() else repo / args.accepted_ledger
    ledger = json.loads(ledger_path.read_text()) if ledger_path.is_file() else {}
    report = audit(repo, upstream_ref, args.target_branch, ledger)
    if args.apply_safe:
      apply_safe(repo, report, args.target_remote)
      if report.auto_applied:
        report.merge_base_sha, report.comparison_sha, report.files = audit_files(repo, upstream_ref)
    elif any(commit.classification in ("review", "blocked") for commit in report.commits):
      report.result = "review_required"
    elif any(commit.classification == "auto" for commit in report.commits):
      report.result = "safe_updates_available"
    else:
      report.result = "up_to_date"
  except Exception as exc:
    if report is None:
      head = _git(repo, "rev-parse", "HEAD", check=False) or "unknown"
      report = SyncReport(head, "unknown", upstream_ref, args.target_branch, [])
    report.result = "error"
    report.error = str(exc)

  _write_report(report, args.json_output, args.markdown_output)
  history_dir = args.history_dir if args.history_dir.is_absolute() else repo / args.history_dir
  _write_history(report, history_dir)
  print(render_markdown(report))
  return 1 if report.result == "error" else 0


if __name__ == "__main__":
  raise SystemExit(main())
