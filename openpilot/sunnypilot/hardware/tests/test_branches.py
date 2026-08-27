from openpilot.sunnypilot.hardware.branches import C3XL_COMPATIBLE_BRANCHES, selectable_tici_branches
from openpilot.sunnypilot.hardware.profile import HardwareProfile
from openpilot.common.version import TICI_COMPATIBLE_BRANCHES


REMOTE_BRANCHES = [
  "dev-sp-egpu",
  "dev-sp-egpu-nva",
  "dev",
  "master-new",
  "release-tici",
  "staging-tici",
]


def test_c3xl_exposes_only_the_two_maintained_branches() -> None:
  assert C3XL_COMPATIBLE_BRANCHES == ("dev-sp-egpu", "dev-sp-egpu-nva")
  assert selectable_tici_branches(REMOTE_BRANCHES, HardwareProfile.C3XL) == [
    "dev-sp-egpu",
    "dev-sp-egpu-nva",
  ]


def test_standard_tici_keeps_upstream_tici_suffix_filter() -> None:
  assert selectable_tici_branches(REMOTE_BRANCHES, HardwareProfile.STANDARD) == [
    "release-tici",
    "staging-tici",
  ]


def test_build_metadata_uses_the_same_c3xl_allowlist() -> None:
  assert TICI_COMPATIBLE_BRANCHES == frozenset(C3XL_COMPATIBLE_BRANCHES)
