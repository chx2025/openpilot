# Port local features through six maintenance seams

The old `sp-dev-egpu` tree mixed Tesla ownership logic, longitudinal planning,
web services, UI preferences, C3XL compatibility, and eGPU experiments into
upstream files.  Future source work follows the `master-dev` commit named by a
`dev` Prebuilt Snapshot.  Local behavior crosses upstream through six small
Interfaces: Tesla Control Profile, Tesla Control Runtime, Radar Backend,
Planner Backend, Plan Constraint, and Device Query/Device Command.

## Consequences

- Features are migrated from the final `6108f38` tree, not inferred from old
  migration notes or intermediate commits.
- Every feature must be inert when disabled and must have a test at its safety
  or process boundary before UI is added.
- A copy of an upstream planner, `selfdrived`, `card`, updater, or settings page
  is not an acceptable long-term Adapter; only a thin wrapper or hook is.
- Official instantiates the current upstream planner and preserves exact
  Traffic-Off/Default behavior through no-op MPC seams. Experimental and
  TN-NoDEC share one local legacy cruise-obstacle equation source because
  their confirmed old behavior cannot be expressed by the upstream lead-only
  solver. It generates a legacy primary plus a numerical recovery variant; the
  MPC family and live tuning profiles require route differential, convergence-
  grid, and timing regression tests.
- opendbc and the main repository are versioned as one Tesla safety unit.
- UI defaults are a separate Local Defaults policy. Any changed Param default is
  explicit, tested, and limited to a confirmed product decision (for example,
  Simplified Chinese on this local build).
- eGPU/model routing was deferred until the source-mode Modules passed device
  validation. Its current official Model Platform and C3XL Model Adapter are
  governed by ADR-0006.
