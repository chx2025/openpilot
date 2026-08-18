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
- Planner Backends share the current upstream MPC and generated solver. A local
  solver or live tuning profile is accepted only after a numerical regression
  suite passes and upstream cannot supply the required Interface.
- opendbc and the main repository are versioned as one Tesla safety unit.
- UI defaults are a separate Local Defaults policy and never justify changing
  upstream Param defaults.
- eGPU/model routing remains out of scope until these source-mode Modules pass
  device validation.
