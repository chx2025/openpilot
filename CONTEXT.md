# sunnypilot C3XL Maintenance

This context defines how the private C3XL build follows sunnypilot while preserving compatibility with non-official comma hardware.

## Language

**Source Baseline**:
The `master-dev` source commit named by a sunnypilot `dev` release commit. All maintained changes are developed and tested from this commit.
_Avoid_: dev source, release source

**Prebuilt Snapshot**:
The orphan `dev` release tree containing compiled artifacts and the `prebuilt` marker. It is a deployment artifact, not a merge or rebase base.
_Avoid_: dev branch, source branch

**Hardware Profile**:
An explicit description of hardware capabilities and compatibility overrides consumed through a small seam. A profile does not replace the device's reported identity.
_Avoid_: hardware hack, device spoof

**C3XL Profile**:
The Hardware Profile for the non-official C3X-compatible target, behaviorally referenced against `mr-one/openpilot:c3xl-dev`.
_Avoid_: TICI mode, fake TIZI

**Panda Startup**:
The ordered reset, application-wait, recovery, firmware-check, and connection sequence executed before `pandad` starts.
_Avoid_: Panda retry loop, Panda workaround

**Boot-chain Allowlist**:
The exact C3XL-validated hashes and sizes for boot-critical AGNOS partitions. Matching images may be flashed automatically; any changed image requires hardware validation before the allowlist is updated.
_Avoid_: frozen AGNOS, disable AGNOS updates

**Tesla Control Profile**:
The startup snapshot that converts user Params into Tesla capabilities and Panda safety flags. Generic car initialization crosses this Seam once and does not enumerate Tesla sub-features.
_Avoid_: Tesla params list, Tesla feature flags

**Tesla Control Runtime**:
The fail-closed runtime view of longitudinal owner, lateral owner, and handoff phase derived from fresh `carStateSP` flags. Generic `selfdrived` consumes policy from this Module rather than interpreting bit masks.
_Avoid_: split-control flags, AP hybrid booleans

**Radar Backend**:
The interchangeable provider of standard `RadarData`. OEM Tesla radar, isolated ARS408, and Off are implementations selected during CarParams initialization.
_Avoid_: ARS mode, radar toggle

**Planner Backend**:
An Adapter implementing the longitudinal planner Interface. Official upstream and local TN implementations must publish the same diagnostics and remain selectable only at session start.
_Avoid_: planner mode code path, copied official planner

**Plan Constraint**:
A decorator that can observe context and return a bounded change to a base longitudinal plan without becoming a Planner Backend. Traffic-control stopping is a Plan Constraint.
_Avoid_: traffic planner, red-light MPC
