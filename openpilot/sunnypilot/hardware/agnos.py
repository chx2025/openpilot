from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile


@dataclass(frozen=True)
class BootChainImage:
  hash: str
  hash_raw: str
  size: int


class UnsafeBootChainManifest(RuntimeError):
  pass


# Known-good images from mr-one/openpilot:c3xl-dev@16c99e6b0b.
# Matching entries are safe to flash automatically; changed entries must first
# be validated on C3XL hardware and deliberately updated here.
C3XL_BOOT_CHAIN_ALLOWLIST: Mapping[str, BootChainImage] = {
  "xbl": BootChainImage(
    hash="e8acf2a9cc7f0ce84cb803bfea9477f765c0d7b4daf26048e59651b9e6a7bfbb",
    hash_raw="e8acf2a9cc7f0ce84cb803bfea9477f765c0d7b4daf26048e59651b9e6a7bfbb",
    size=3282256,
  ),
  "xbl_config": BootChainImage(
    hash="758552ecf92b5569677197783bf0ccb73d7f961685308e45d3276ac9dd974f85",
    hash_raw="758552ecf92b5569677197783bf0ccb73d7f961685308e45d3276ac9dd974f85",
    size=98124,
  ),
  "abl": BootChainImage(
    hash="32a2174b5f764e95dfc54cf358ba01752943b1b3b90e626149c3da7d5f1830b6",
    hash_raw="32a2174b5f764e95dfc54cf358ba01752943b1b3b90e626149c3da7d5f1830b6",
    size=274432,
  ),
  "aop": BootChainImage(
    hash="78b2287ca219a0811b3004c523fa0f4749e4d1fd92be3aba61699305b7943ad1",
    hash_raw="78b2287ca219a0811b3004c523fa0f4749e4d1fd92be3aba61699305b7943ad1",
    size=184364,
  ),
  "devcfg": BootChainImage(
    hash="f71df3a86958c093ba3969254c4db025187eef9385427f1ade946742939b43cc",
    hash_raw="f71df3a86958c093ba3969254c4db025187eef9385427f1ade946742939b43cc",
    size=40336,
  ),
  "boot": BootChainImage(
    hash="0191529aa97d90d1fa04b472d80230b777606459e1e1e9e2323c9519839827b4",
    hash_raw="0191529aa97d90d1fa04b472d80230b777606459e1e1e9e2323c9519839827b4",
    size=18515968,
  ),
}


def validate_agnos_manifest(partitions: Sequence[dict], profile: HardwareProfile | None = None) -> None:
  if (profile or get_hardware_profile()) != HardwareProfile.C3XL:
    return

  by_name = {partition.get("name"): partition for partition in partitions}
  if len(by_name) != len(partitions):
    raise UnsafeBootChainManifest("AGNOS manifest contains duplicate or unnamed partitions")

  for name, allowed in C3XL_BOOT_CHAIN_ALLOWLIST.items():
    partition = by_name.get(name)
    if partition is None:
      raise UnsafeBootChainManifest(f"C3XL AGNOS manifest is missing required boot-chain partition {name!r}")

    actual = BootChainImage(
      hash=str(partition.get("hash", "")).lower(),
      hash_raw=str(partition.get("hash_raw", "")).lower(),
      size=partition.get("size", -1),
    )
    if actual != allowed:
      raise UnsafeBootChainManifest(
        f"C3XL refuses unvalidated {name!r} image: expected hash={allowed.hash} size={allowed.size}, got hash={actual.hash} size={actual.size}"
      )

    if partition.get("has_ab", True) is not True or partition.get("full_check", False) is not True:
      raise UnsafeBootChainManifest(f"C3XL requires full A/B verification for boot-chain partition {name!r}")
