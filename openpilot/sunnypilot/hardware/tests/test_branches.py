from openpilot.sunnypilot.hardware.branches import C3XL_COMPATIBLE_BRANCHES, is_prebuild_branch, selectable_tici_branches
from openpilot.sunnypilot.hardware.profile import HardwareProfile
from openpilot.common.version import TICI_COMPATIBLE_BRANCHES


REMOTE_BRANCHES = [
  "dev-sp-egpu",
  "dev-sp-egpu-nva",
  "dev-sp-egpu-prebuild",
  "dev",
  "master-new",
  "release-tici",
  "staging-tici",
]


def test_c3xl_exposes_only_the_maintained_branches() -> None:
  assert C3XL_COMPATIBLE_BRANCHES == ("dev-sp-egpu", "dev-sp-egpu-nva", "dev-sp-egpu-prebuild")
  assert selectable_tici_branches(REMOTE_BRANCHES, HardwareProfile.C3XL) == [
    "dev-sp-egpu",
    "dev-sp-egpu-nva",
    "dev-sp-egpu-prebuild",
  ]


def test_standard_tici_keeps_upstream_tici_suffix_filter() -> None:
  assert selectable_tici_branches(REMOTE_BRANCHES, HardwareProfile.STANDARD) == [
    "release-tici",
    "staging-tici",
  ]


def test_build_metadata_uses_the_same_c3xl_allowlist() -> None:
  assert TICI_COMPATIBLE_BRANCHES == frozenset(C3XL_COMPATIBLE_BRANCHES)


def test_prebuild_spelling_is_grouped_with_upstream_prebuilt_branches() -> None:
  assert is_prebuild_branch("dev-sp-egpu-prebuild")
  assert is_prebuild_branch("feature-prebuilt")
  assert not is_prebuild_branch("dev-sp-egpu")
