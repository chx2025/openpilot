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

Traffic control does not subscribe to or gate its state on `radarState`.
Physical and vision leads remain inputs of the selected base planner, not inputs
of the traffic-light state machine. A STOP is merged as a more conservative
post-plan constraint. A confirmed same-session GREEN may apply the bounded,
time-limited START or rolling-release profile; it does not represent obstacle
clearance and does not use lead presence as a veto. This is an explicit product
choice, not an inference that the path is clear.

The GO request is bounded and deduplicated per stop session and never modifies
Tesla vehicle state, CAN, or other vehicle signals. Traffic Off and Observe are
output-transparent.

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
- The three backend profiles remain independently adjustable and require an
  explicit, lossless configuration migration when their schema changes.
