# sp-dev-egpu to sunnypilot dev migration plan

## Verified baselines

| Role | Commit | Use |
| --- | --- | --- |
| sunnypilot source | `f403b3a5e334cd7cf49dedd876a9d6ed8419bd37` | Source Baseline named by the current `dev` release |
| sunnypilot release | `6384047adee5d82dc8b87db9c33134146fd4260e` | Prebuilt Snapshot; not a merge/rebase base |
| local final tree | `6108f38` | Evidence of features that still exist |
| local history bridge | `629eda5` | Intent and commit provenance only |
| mr-one C3XL reference | `16c99e6b0bdc37f31b1c9c4c1f5ffad52545a452` | Behavioral reference for C3XL/Panda/AGNOS |

The fast startup premise needed one correction: sunnypilot `dev` is fast
because it contains the `prebuilt` marker and compiled artifacts. It is not a
non-precompiled development branch. Development therefore starts from
`master-dev`; a separate branch such as `dev-c3xl-prebuilt-tici` publishes the
prebuilt tree. The final deployed branch must end in `-tici` so upstream's
existing channel check recognizes the target without a global bypass.

## Boot and Panda policy

The C3XL and current upstream manifests agree on `xbl`, `xbl_config`, `aop`,
`devcfg`, and `system`. They differ on `abl` and `boot`. The C3XL profile uses
the mr-one ABL and boot images and validates exact hash, raw hash, size, full
check, and A/B attributes before flashing.

This is an allowlist, not a blanket flash ban: every matching image, including
matching boot-chain images, may update automatically. An unknown changed
boot-chain image fails before any partition write. Panda reset/recover and raw
type interpretation are profile-scoped; raw and effective types remain
observable. The old unconditional `Panda.get_type() == 9` override is excluded.

## Feature inventory and migration slices

| Priority | Module | Content to port | Upstream intrusion after refactor | Status |
| --- | --- | --- | --- | --- |
| P0 | C3XL Profile | AGNOS allowlist/manifest, hardware identity Adapter, Panda Startup, read-only probe | build profile define; AGNOS validator call; Panda startup call | Host-tested |
| P0 | Tesla Control | DBC/HW4 decoding, MADS/coop steering, manual longitudinal selection, Dynamic Auto Stock, AP Hybrid, auto speed, safety validation | Tesla Control Profile at car init; Tesla Control Runtime at selfdrived | Core/opendbc ported; device test pending |
| P0 | Radar Backend | OEM/ARS408/Off selector, ARS RX parser, tracker, diagnostics, bounded motion TX, Panda safety | one backend selector in opendbc; one enum Param/UI control | Host-tested; device test pending |
| P1 | Planner Backend | Official and TN-NoDEC; session latch; TN acceleration personality; stopping policy | one planner factory; three default-no-op MPC hooks; two planner-helper hooks | Host-tested; device test pending |
| P1 | Traffic-control Plan Constraint | Off/Observe/Shadow/StopOnly/StopGo, freshness, confirmation, radar/lead gate, pedal bypass, HUD diagnostics | one planner decorator; card observation publisher; UI consumes Decision | Host-tested; device test pending |
| P2 | Device Query/Command | read-only status, Tesla diagnostics, settings allowlist, hotspot | one managed service; query/command registry | Pending security review |
| P2 | Update reliability | proxy Adapter, current-tree LFS hydrate, last-known-good clock | narrow updater/time hooks | LFS and clock host-tested; proxy pending |
| P3 | Local Defaults/UX | offroad brightness entry, one-minute shutdown choice, optional buzzer/sounds, speed offset cap | separate defaults policy and isolated UI rows | Brightness, shutdown, speed cap, and SCC-V ported; buzzer/sounds pending hardware review |

### Tesla safety unit

The following move together and are not independently versioned:

1. Tesla DBCs and HW4 shadow decoding.
2. `CarState` ownership state machine and `CarController` output.
3. Panda Tesla safety flags, TX allowlists, and validation tests.
4. Tesla Control Profile parameters and the opendbc submodule commit.
5. Tesla Control Runtime event policy and same-cycle `carStateSP` publication.

Turning all local Tesla switches off must result in upstream behavior and no
new CAN transmission. ARS408 additionally requires its safety flag before any
motion frame is accepted.

### Planner migration rule

Do not keep a forked `longitudinal_planner_official.py`. The Official Adapter
constructs the Source Baseline planner directly. TN owns only NoDEC,
acceleration-personality, and stopping-policy deltas and reuses the same
upstream six-parameter MPC and generated solver. The old separate eight-
parameter TN solver is excluded: its status-4 failure is reproducible in the
final old tree, and keeping it would add a second generated-code maintenance
surface. The retired Experimental selection fails closed to Official.

Full live MPC tuning profiles are not migrated. Comfort-brake and stop-distance
values are compiled into the current upstream six-parameter model, so a faithful
runtime implementation would require another solver or upstream model change.
TN acceleration personality remains supported because it crosses the existing
bounded acceleration-policy seam. Traffic control is a Plan Constraint with
`observe`, `decide`, and `constrain`; it cannot directly own the planner or
duplicate decisions in the UI.

## Explicitly excluded

- eGPU/UT3G model routing, GPU HUD, model assets, and Chestnut/libusb changes.
- The old offline-Panda-wake series: its intermediate commits are not present
  as a functional net change in final `6108f38`.
- Unconditional Panda type spoofing.
- Arbitrary browser shell, default password `123456`, and `shell=True` command
  execution. A future web console must use a command registry and on-road write
  gates.
- Disabled audio-feedback code, disabled loggerd/DM/micd changes, hard-coded
  IMEI/private hosts, and Params with no consumer.
- Duplicate GPS time-sync process; only last-known-good clock persistence may
  be added to upstream `timed`.
- The old separate Experimental planner snapshot and live MPC tuning profiles;
  both require long-lived copies or a second generated solver. Reconsider only
  if upstream adds a stable runtime tuning Interface.
- Whole-file copies of old `updated.py`, `selfdrived.py`, UI settings pages, or
  planner files.

## Acceptance gates

1. Host lint, schema generation, unit tests, and native C++ profile tests.
2. Tesla/opendbc Python and Panda safety suites with each capability disabled
   and enabled.
3. C3XL read-only probe records raw Panda type, effective type, boot slot,
   AGNOS version, and bootstub/application state before any flash.
4. Offroad dry-run validates all AGNOS allowlist entries without writes.
5. Bench CAN replay verifies no extra TX when a Module is disabled, followed by
   bounded ARS408 and Tesla handoff tests.
6. Only after the above, build and publish `dev-c3xl-prebuilt-tici`; retain the
   previous known-good Prebuilt Snapshot for rollback.
