from collections.abc import Iterable

from openpilot.sunnypilot.hardware.profile import HardwareProfile


# Single source of truth for branches that include the isolated C3XL hardware,
# Panda, Tesla, eGPU, and boot-chain compatibility seams.
C3XL_COMPATIBLE_BRANCHES = ("dev-sp-egpu", "dev-sp-egpu-nva", "dev-sp-egpu-prebuild")


def is_prebuild_branch(branch: str) -> bool:
  return branch.endswith(("-prebuild", "-prebuilt"))


def selectable_tici_branches(branches: Iterable[str], profile: HardwareProfile) -> list[str]:
  """Return update targets that are safe for the effective TICI hardware profile."""
  if profile == HardwareProfile.C3XL:
    available = set(branches)
    return [branch for branch in C3XL_COMPATIBLE_BRANCHES if branch in available]
  return [branch for branch in branches if branch.endswith("-tici")]
