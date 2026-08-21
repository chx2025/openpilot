from tools.sunnypilot_upstream_sync import CommitAudit, classify_commit, enforce_contiguous_prefix


def commit(path: str, status: str = "M") -> CommitAudit:
  return CommitAudit(sha="a" * 40, subject="test", paths=[path], statuses=[status])


def test_docs_only_change_is_eligible_for_automatic_sync():
  audited = classify_commit(commit("docs/guide.md"))
  assert audited.classification == "auto"
  assert audited.reasons == []


def test_runtime_and_submodule_changes_require_human_review():
  runtime = classify_commit(commit("openpilot/selfdrive/controls/plannerd.py"))
  submodule = classify_commit(commit("opendbc_repo"))
  assert runtime.classification == "review"
  assert submodule.classification == "review"


def test_delete_is_never_automatic_even_in_docs():
  audited = classify_commit(commit("docs/obsolete.md", "D"))
  assert audited.classification == "review"
  assert "delete/rename/copy requires review" in audited.reasons


def test_safe_commit_after_review_commit_is_blocked_to_preserve_order():
  first = classify_commit(commit("openpilot/common/params_keys.h"))
  second = classify_commit(commit("docs/guide.md"))
  enforce_contiguous_prefix([first, second])
  assert first.classification == "review"
  assert second.classification == "blocked"


def test_patch_equivalent_commit_does_not_block_later_safe_commit():
  equivalent = commit("openpilot/common/version.py")
  equivalent.equivalent = True
  classify_commit(equivalent)
  safe = classify_commit(commit("README.md"))
  enforce_contiguous_prefix([equivalent, safe])
  assert equivalent.classification == "equivalent"
  assert safe.classification == "auto"


def test_previously_reviewed_commit_does_not_block_later_safe_commit():
  accepted = classify_commit(commit("openpilot/common/version.py"))
  accepted.classification = "accepted"
  accepted.reasons = ["ported locally"]
  safe = classify_commit(commit("README.md"))
  enforce_contiguous_prefix([accepted, safe])
  assert accepted.classification == "accepted"
  assert safe.classification == "auto"
