# Tesla legacy planner replay fixture

`tesla_legacy_planner_warm.rlog.zst` is a minimized planner-only slice from a
user-provided route. It contains no CAN, GPS, camera, audio, route identifier,
or device-identifying services. The retained services are only the inputs needed
to deterministically replay `plannerd` through its public process-replay seam.

The expected JSON files were produced from the identical final Experimental and
TN-NoDEC implementations in `sp-dev-rs408@64d9c54e2c` and
`sp-dev-egpu@6108f38d09`. Those planner/MPC source blobs are byte-identical
between the two references.

The route is intentionally tracked despite the repository-wide `*.zst` ignore
rule. A root `.gitignore` exception keeps it in clean checkouts, and the replay
test verifies SHA-256
`36e4a6e774d33839b2b9f78d9f90d14b132020fb1cb88511d1096c737b0747f4`
before using it.
