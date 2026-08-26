# Restore legacy custom planners and use an independent Traffic Radar

## Decision

Official continues to instantiate the upstream planner provider. Its shared MPC
retains the existing tuning adapter; with Traffic Off and the Default profile,
explicit fast paths preserve upstream inputs and outputs without a
floating-point round trip. Experimental and
TN-NoDEC restore the final `sp-dev-rs408`/`sp-dev-egpu` planner decision flow:
the legacy cruise obstacle is solved inside MPC, final arbitration is MPC plus
the legacy optional E2E candidate, state recursion and acceleration clipping
follow the old implementation, and TN retains its old acceleration controller
without DEC.

The two custom backends share one reproducible eight-parameter cruise-obstacle
equation source. It generates the old numerical configuration as the primary
solver plus a less aggressively condensed recovery solver. Recovery runs only
after a primary failure while longitudinal control is active; the primary
success and inactive paths therefore retain the old route output. Neither old
platform-specific generated tree is copied. A recovered primary failure is
rate-limited but always logged; this numerical fail-safe is the sole non-Traffic
behavioral exception, because reproducing the old zero-trajectory failure while
engaged would violate the safety-first deployment gate.

Traffic control is produced once by `trafficcontrold` as a typed
`trafficRadarState`. The message is not `radarState`, is not fed to modeld, and
does not create or overwrite a physical `leadOne` or `leadTwo`. The selected
Official, Experimental, or TN-NoDEC backend first produces its normal base
plan. `FinalPlanArbitrator` then consumes `trafficRadarState` at the common
post-planner publish boundary. No planner or MPC contains a Traffic target,
adapter, or `lead2` injection path.

Tesla bus-2 `0x25D` is the authoritative color and distance selected by the
vehicle for its current lane. Turn indicators, model turn intent, navigation,
Tesla continuation state, and physical/vision leads do not veto or create a
Traffic STOP. Frames beyond 200 metres remain diagnostic-only. A STOP is merged
as a more conservative post-plan constraint.

The traffic-light state machine remains independent of `radarState`, but the
post-plan bounded START uses a separate fail-closed lead gate. Any current lead
within eight metres blocks START immediately. A selected lead within eight
metres must persist for 0.5 seconds before the whole stop session is delegated
to the base lead planner; transient unselected targets clear after 0.4 seconds
of healthy no-lead observations. Leads beyond eight metres do not veto the
bounded low-speed START, and unhealthy lead sensing leaves the base plan
unchanged. A moving same-session GREEN removes the Traffic STOP immediately and
returns the complete plan to the selected base planner.

The GO request is bounded and deduplicated per stop session and never modifies
Tesla vehicle state, CAN, or other vehicle signals. Traffic Off and Observe are
output-transparent even if a prior STOP/HOLD/START was latched. `applied` means
that Traffic changed at least one final longitudinal output field; an eligible
constraint already dominated by the base planner is not attributed to Traffic
in the UI.

## Consequences

- The old “all planners reuse the upstream solver” decision is superseded for
  Experimental and TN-NoDEC only.
- One shared equation/build module avoids the duplicate Experimental and TN
  solver sources in the old repository; its primary and recovery artifacts are
  generated for the target platform and never committed.
- Old route output remains the behavioral oracle through target-specific
  Darwin/arm64 and Linux/aarch64 baselines; synthetic convergence and timing
  tests are additional deployment gates.
- The old planner adapter, MPC Traffic-target setter, and fake-`leadTwo` path
  are removed; the post-planner arbitrator is the only longitudinal control
  seam.
- Yellow PASS, driver gas override, and the configured maximum-speed bypass are
  event-scoped: the same intersection cannot reacquire STOP ownership late.
- Flashing green requires three in-range, motion-consistent GREEN/OFF pulses;
  stable same-track GREEN for the maximum flash interval releases the stop.
- The three backend profiles remain independently adjustable and require an
  explicit, lossless configuration migration when their schema changes.
