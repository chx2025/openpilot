# Tesla legacy planner replay fixture

`tesla_legacy_planner_warm.rlog.zst` is a minimized planner-only slice from a
user-provided route. It contains no CAN, GPS, camera, audio, route identifier,
or device-identifying services. The retained services are only the inputs needed
to deterministically replay `plannerd` through its public process-replay seam.

The two original `tesla_legacy_{experimental,tn}.json` files are the Darwin/
arm64 numerical baselines produced from the final Experimental and TN-NoDEC
implementations in `sp-dev-rs408@64d9c54e2c` and `sp-dev-egpu@6108f38d09`.
Those planner/MPC source blobs are byte-identical between the references.

`tesla_legacy_linux_aarch64.json` is the C3XL target baseline. The old final
tree's ignored `long_official` generated C was rebuilt in a device `/tmp`
directory, loaded as the primary solver, and replayed through the same current
planner harness. On the same Linux/aarch64 device, old and migrated primary
solvers matched all 236 cycles exactly: every numeric maximum error and every
discrete-field difference was zero. The baseline SHA-256 is
`30e5da2925023786fe21199c74bac81caf924791b81f93b5a2bf3fccd6c71c50`.

acados trajectories are not assumed to be cross-platform identical. The test
selects a recorded old-tree numerical baseline for the executing target rather
than weakening tolerances or tuning the target solver to imitate another CPU.

The route is intentionally tracked despite the repository-wide `*.zst` ignore
rule. A root `.gitignore` exception keeps it in clean checkouts, and the replay
test verifies SHA-256
`36e4a6e774d33839b2b9f78d9f90d14b132020fb1cb88511d1096c737b0747f4`
before using it.
