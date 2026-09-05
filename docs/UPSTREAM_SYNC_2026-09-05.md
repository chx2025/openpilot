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
- Device and Nav worktree have independent uncommitted USB queue/copy-in and
  100-us polling changes. They are preserved and not included in this upstream
  integration. Do not overwrite them during deployment or claim a clean,
  reproducible device until their owner finishes and commits them.

## Validation scope

Local modeld_v2 suite, AGNOS allowlist suite and new release-artifact tests:
129 tests passed. Ruff on changed Python implementation/tests and diff-check
passed. This is not native GPU performance, onroad, or system-upgrade validation.
