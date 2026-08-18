import json
from pathlib import Path

import pytest

from openpilot.sunnypilot.hardware.agnos import UnsafeBootChainManifest, validate_agnos_manifest
from openpilot.sunnypilot.hardware.profile import HardwareProfile


MANIFEST = Path(__file__).parents[3] / "common/hardware/comma/agnos.json"


def read_manifest() -> list[dict]:
  with MANIFEST.open() as manifest_file:
    return json.load(manifest_file)


def test_c3xl_manifest_matches_allowlist() -> None:
  validate_agnos_manifest(read_manifest(), HardwareProfile.C3XL)


@pytest.mark.parametrize("name", ["xbl", "xbl_config", "abl", "aop", "devcfg", "boot"])
def test_c3xl_rejects_changed_boot_chain_image(name: str) -> None:
  manifest = read_manifest()
  partition = next(partition for partition in manifest if partition["name"] == name)
  partition["hash"] = "0" * 64

  with pytest.raises(UnsafeBootChainManifest, match=name):
    validate_agnos_manifest(manifest, HardwareProfile.C3XL)


def test_standard_profile_does_not_apply_c3xl_allowlist() -> None:
  manifest = read_manifest()
  next(partition for partition in manifest if partition["name"] == "boot")["hash"] = "0" * 64
  validate_agnos_manifest(manifest, HardwareProfile.STANDARD)
