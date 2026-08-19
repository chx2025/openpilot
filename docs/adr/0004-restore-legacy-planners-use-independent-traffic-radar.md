# Restore legacy custom planners and use an independent Traffic Radar

## Decision

Official continues to instantiate the upstream planner provider. Its shared MPC
has only the optional Traffic-target setter and the existing tuning adapter;
with Traffic Off and the Default profile, explicit fast paths preserve upstream
inputs and outputs without a floating-point round trip. Experimental and
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
does not create or overwrite a physical `leadOne` or `leadTwo`. Each planner may
include it as an independent obstacle candidate; `lead2` identifies that source
without making `hasLead` or FCW report a physical vehicle. A current physical
lead suppresses the Traffic target at both producer and planner boundaries.

The direct Stop Profile and Traffic Radar strategies consume the same producer
event and state machine. A GO request is read only by the longitudinal planner,
is bounded and deduplicated per event, and never modifies Tesla vehicle state,
CAN, or other vehicle signals. Traffic Off, Observe, and Shadow are output-
transparent.

## Consequences

- The old “all planners reuse the upstream solver” decision is superseded for
  Experimental and TN-NoDEC only.
- One shared equation/build module avoids the duplicate Experimental and TN
  solver sources in the old repository; its primary and recovery artifacts are
  generated for the target platform and never committed.
- Old route output remains the behavioral oracle; synthetic convergence and
  timing tests are additional deployment gates.
- The old fake-`leadTwo` Traffic adapter is forbidden.
- The three backend profiles remain independently adjustable and require an
  explicit, lossless configuration migration when their schema changes.
