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

Each new `stopSessionId` owns fresh geometry derived from the confirming
bus-2 CAN distance; it never inherits a stop station tracked while GREEN or
owned by an earlier session. During an ordinary approach/braking/yellow STOP,
one discontinuous RED/YELLOW tuple is evidence only. A nearer recalculation
keeps the existing fast confirmation path so Traffic does not delay a more
conservative stop point. A farther recalculation, which would relax braking,
must instead remain motion-consistent for at least three distinct frames and
one second before it creates a new stop session. Every confirmed replacement
lets the final arbitrator repeat its one-time feasibility decision. A
stationary HOLD and a confirmed flashing STOP do not move to a discontinuous
RED/YELLOW distance.

Only a fresh, in-range RED or YELLOW frame supports an ordinary moving STOP.
OFF, unsupported colors, and non-wrap out-of-range observations freeze the
last confirmed stop station rather than changing its geometry. An ordinary
approach/braking/yellow STOP retains that frozen station for a two-second
evidence-loss grace and then enters the existing jerk-limited RELEASE. A
stationary HOLD and a confirmed flashing-green STOP remain latched; a genuine
near-to-254 distance wrap still passes the event immediately. RED recovered
after an evidence-loss RELEASE creates a fresh stop session, so motion during
the release cannot reuse the earlier feasibility decision. A multi-second CAN
transport gap clears all uncommitted color, flash, candidate, and replacement
evidence; an ordinary moving STOP recovered as RED/YELLOW also receives a new
session after two fresh frames. A HOLD that reaches its frozen absolute stop
station remains latched even though the last raw CAN distance is intentionally
not updated during the evidence gap.

Ordinary GREEN release is authoritative independently of stop-station geometry:
a moving vehicle releases on the first fresh GREEN frame, while standstill
still requires two distinct fresh GREEN frames. This color decision is made
before a geometry-conflict early return, so a stale stop station cannot
permanently retain STOP after Tesla reports GREEN. A confirmed flashing-green
STOP keeps its separate stable-GREEN exit rule.

The traffic-light state machine remains independent of `radarState`, but the
post-plan bounded START uses a separate fail-closed lead gate. Any current lead
within eight metres blocks START immediately. A selected lead within eight
metres must persist for 0.5 seconds before the whole stop session is delegated
to the base lead planner; transient unselected targets clear after 0.4 seconds
of healthy no-lead observations. Leads beyond eight metres do not veto the
bounded low-speed START, and unhealthy lead sensing leaves the base plan
unchanged. A moving same-session GREEN removes the Traffic STOP immediately and
returns the complete plan to the selected base planner.

The moving-GREEN threshold distinguishes a vehicle that was already rolling
when GREEN arrived from a START that Traffic initiated from standstill. The
former remains output-transparent after STOP removal. The latter continues
past the rolling threshold within the existing 2.5 m/s and three-second bounds,
and rechecks the lead and driver gates every cycle.

For a confirmed selected queue lead, the base lead planner also owns queue
motion while the Traffic stop point remains outside a dynamic guard: the larger
of five metres and the personality-aware comfortable stopping envelope. The
near lead must remain valid, selected, healthy, and continuously confirmed;
radar-health recovery requires a fresh 0.5-second confirmation. The Traffic
stop session stays armed rather than being discarded and resumes its ordinary
STOP before the guard is consumed. This prevents a zero-speed Traffic profile
from pinning a vehicle behind a departing queue, avoids a hard ownership switch
at 0.3 m/s, and does not give a stale or far lead slot authority over the final
stop-line guard.

The GO request is bounded and deduplicated per stop session and never modifies
Tesla vehicle state, CAN, or other vehicle signals. Traffic Off and Observe are
output-transparent even if a prior STOP/HOLD/START was latched. `active` means
that Traffic changed or maintains part of the complete published plan,
including a future-only trajectory constraint; a Traffic candidate completely
dominated by an unchanged base plan is not active. `applied` is narrower:
Traffic changed the current actuator contract consumed by controls (`aTarget`
by more than the `1e-3 m/s²` diagnostic noise tolerance, or any `shouldStop`
change). The UI attributes current control only from `applied`; a two-second,
same-session UI-only notice may report a recent applied action in past tense,
but never turns an `active`-only future constraint into current Traffic
control. RELEASE wording also follows the current color and phase rather than
assuming every trajectory handoff was caused by GREEN.

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
  one or two pulses remain internal evidence and never become a control phase;
  stable same-track GREEN for the maximum flash interval releases a confirmed
  flashing stop.
- A new stop session always rebases to its confirming CAN distance. A sustained
  recalculated RED/YELLOW track creates a new session instead of being fused
  into or permanently rejected by the previous session.
- A nearer replacement retains fast confirmation. A farther replacement needs
  three real frames spanning one second, so a short jump cannot relax an
  already confirmed STOP before returning to the original track.
- Ordinary moving STOP geometry freezes when its supporting signal evidence is
  lost and releases smoothly after two seconds. HOLD, confirmed flashing STOP,
  and genuine passed-distance wraps retain their stricter dedicated behavior.
- Traffic-initiated GO continues across the 0.3 m/s rolling threshold; a vehicle
  that was already moving when GREEN arrived remains transparent to Traffic.
  The existing speed/time hard bounds still return the untouched base plan and
  never extend Traffic control.
- Yellow receives STOP ownership only when the personality-aware comfortable,
  jerk-limited stopping envelope plus a bounded uncertainty margin fits inside
  the remaining distance. A rejected yellow session cannot reacquire ownership
  after changing to red.
- The delayed jerk-limited stopping envelope uses an O(1) closed-form solution.
  Its STOP-ownership decision is cached per stop session; armed and rejected
  sessions do not repeat that decision at planner frequency while the
  publisher continues to identify the same session. Loss of the Traffic
  service clears armed ownership because a restarted producer may reuse a
  numeric session ID. A currently delegated queue lead recomputes only the
  lightweight dynamic line guard as speed changes.
- The three backend profiles remain independently adjustable and require an
  explicit, lossless configuration migration when their schema changes.
