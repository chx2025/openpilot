# sunnypilot update integration — 2026-09-05

Official source reviewed: `047ae41c0d9fe3ae5656d1541a376d6fbf17c9a3`.

## Integrated

- Port modeld_v2 same-device warp compilation and `warp_dev` metadata, v24
  Chestnut catalog, raw supercombo finite-output validation and post-enqueue
  telemetry callback. Preserve legacy downloaded bundles, loading progress,
  timeout, persistent caches, 100 W default and small-model fallback.
- Exclude `warp_dev` from split/multi-policy metadata enumeration. Otherwise
  the new scalar metadata is mistaken for a policy's input/output definitions.
- Preserve final plan finite checks for all big-model architectures.
- Adapt DM artifact completeness to our existing native SCons release flow:
  both camera-resolution DM warps are already compiled by SConscript; require
  both nonempty files before accepting a prebuilt. Do not import upstream HF
  publisher workflows or implicitly publish to sunnypilot's model repositories.
- opendbc: only port the upstream cppcheck suppression from `f95f996f`.
  Preserve the Nav branch's `81a3419d` navigation signal changes separately.

## Not completed / not authorized by a successful validation

- AGNOS 19.7 is NOT enabled or flashed. Device 192.168.10.179 reports 19.6.
  The official boot image is `6ecf6f987cd11968104abcccabbe268485d329cdb73012dfd3c381a6b8deb27d`
  (46,897,152 bytes), whereas C3XL's validated boot image is
  `0191529aa97d90d1fa04b472d80230b777606459e1e1e9e2323c9519839827b4`
  (18,515,968 bytes). Do not relax the allowlist, replace the kernel blindly,
  or assemble an unvalidated 19.7 system/old-kernel hybrid.
- Panda health-packet format migration must be a paired firmware/pandad native
  build and hardware validation. No Panda pointer or bootstub changed here.
- The Prius steering-rack dashcam restriction revert is not applied.
- Official CI polling / HF publishing changes are not imported wholesale:
  local release workflows do not have the same default-model download jobs.
- The device USB queue/copy-in changes were subsequently captured in tinygrad
  `5b3d60d0d`, with 11 protocol/queue tests. The 100-us polling configuration
  and tinygrad pointer are now committed in both maintained source branches.

## Validation scope

Local modeld_v2 suite, AGNOS allowlist suite and new release-artifact tests:
131 tests passed after incorporating the polling tests. An additional 148
Navassist/Tesla/boundary tests passed. Ruff and diff-check passed.

Device 192.168.10.179, AGNOS 19.6, offroad, comma service stopped:

| Candidate | Load | Synthetic inference mean / p95 | Result |
| --- | --- | --- | --- |
| Nav BMV6, 30 s | 23.45 s | 47.13 / 49.20 ms | finite outputs, 586 frames |
| Base modeld_v2 BMV6, 30 s | 21.47 s | 46.70 / 48.45 ms | finite outputs, 587 frames |
| Nav selected QCOM model, 20 s | 4.51 s | 48.78 / 51.88 ms | finite outputs, 323 frames |
| Nav BMV4 from v24 catalog, 30 s | 24.62 s | 46.98 / 48.90 ms | finite outputs, 585 frames |

All tested big artifacts report QCOM warp metadata; this does not constitute
hardware validation of a newly compiled AMD-warp artifact. GPU telemetry in all
big tests returned 100 W PPT, PCIe L0 (0x78), metricsValid=true, supplyValid=false.
Some frames exceeded 50 ms; synthetic tests are not a real-road latency guarantee.
The harness publishes no vehicle/cereal messages and preserves model selection.
916 main-repository Python/cereal file hashes matched the Nav source checkout
after updating eight stale test files. The device remains a flattened prebuilt
tree: source provenance is separate from its deployment Git commit.

With user approval, removed unused `/data/mrsp`; also removed rebuildable SCons
cache and historical logs/realdata. Kept installed models, UT3G recovery material,
parameters and `/data/openpilot`. Historical logs were archived to the Mac before
deletion. Free device storage increased from about 13 to 18 GiB.
