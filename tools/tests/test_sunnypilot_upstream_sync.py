from pathlib import Path
import subprocess

from tools.sunnypilot_upstream_sync import FileAudit, audit_files, classify_file


def git(repo: Path, *args: str) -> str:
  result = subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True, text=True)
  return result.stdout.strip()


def write(repo: Path, path: str, content: str) -> None:
  target = repo / path
  target.parent.mkdir(parents=True, exist_ok=True)
  target.write_text(content)


def commit_all(repo: Path, message: str) -> None:
  git(repo, "add", ".")
  git(repo, "commit", "-m", message)


def test_file_audit_distinguishes_upstream_local_conflict_and_equivalence(tmp_path):
  repo = tmp_path / "repo"
  repo.mkdir()
  git(repo, "init", "-b", "target")
  git(repo, "config", "user.name", "Test")
  git(repo, "config", "user.email", "test@example.com")
  write(repo, "README.md", "base\n")
  write(repo, "local.txt", "base\n")
  write(repo, "shared.py", "base\n")
  write(repo, "same.txt", "base\n")
  commit_all(repo, "base")

  git(repo, "switch", "-c", "upstream")
  write(repo, "README.md", "official\n")
  write(repo, "shared.py", "official\n")
  write(repo, "same.txt", "same final\n")
  commit_all(repo, "official")

  git(repo, "switch", "target")
  write(repo, "local.txt", "local\n")
  write(repo, "shared.py", "local\n")
  write(repo, "same.txt", "same final\n")
  commit_all(repo, "local")

  merge_base, comparison_sha, files = audit_files(repo, "upstream")
  by_path = {file.path: file for file in files}

  assert merge_base == git(repo, "rev-parse", "target~1")
  assert comparison_sha == git(repo, "rev-parse", "target")
  assert by_path["README.md"].classification == "safe_candidate"
  assert by_path["local.txt"].classification == "keep_local"
  assert by_path["shared.py"].classification == "manual_merge"
  assert by_path["same.txt"].classification == "equivalent"


def test_sensitive_upstream_file_requires_review():
  file = FileAudit(path="openpilot/selfdrive/controls/controlsd.py", upstream_status="M")

  classify_file(file, content_equal=False)

  assert file.classification == "review_upstream"
  assert "人工审核" in file.conclusion

