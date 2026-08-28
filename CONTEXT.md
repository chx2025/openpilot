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
An Adapter implementing the longitudinal planner Interface. Official upstream,
legacy Experimental, and legacy TN-NoDEC implementations publish the same
diagnostics and remain selectable only at session start.
_Avoid_: planner mode code path, copied official planner

**Legacy Cruise MPC**:
The single eight-parameter cruise-obstacle equation source shared by
Experimental and TN-NoDEC. It generates the legacy primary solver and a
numerically robust recovery variant, restoring the confirmed old candidate
structure without copying either old platform-specific generated tree.
_Avoid_: old Official solver, duplicated TN solver

**Traffic Radar**:
A typed, planner-only Traffic target produced by `trafficcontrold`. It may be an
independent obstacle candidate but is never a physical radar lead, model input,
FCW target, vehicle state, or CAN signal.
_Avoid_: fake leadTwo, virtual vehicle, traffic radarState

**Plan Constraint**:
A decorator that can observe context and return a bounded change to a base
longitudinal plan without becoming a Planner Backend. The direct Stop Profile
is a Plan Constraint; the Traffic Radar strategy uses the same producer through
the planner's optional target seam.
_Avoid_: traffic planner, duplicated traffic controller

**Model Platform**:
The official hardware-driven QCOM/Chestnut selection. Each platform keeps an
independent selected bundle; a healthy connected Chestnut activates Big Model,
otherwise QCOM activates Small Model.
_Avoid_: explicit model source, USBGPU model selector

**C3XL Model Adapter**:
The narrow model-runtime Adapter that preserves C3XL hardware capability:
downloaded Chestnut bundle readiness, 75-second loading, loading progress,
available compile CPU, UT3G identity, telemetry, and safe eject. It never
chooses between QCOM and Chestnut.
_Avoid_: C3XL model manager, alternate model selector
