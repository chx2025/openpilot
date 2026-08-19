# Tesla legacy planner replay fixture

`tesla_legacy_planner_warm.rlog.zst` is a minimized planner-only slice from a
user-provided route. It contains no CAN, GPS, camera, audio, route identifier,
or device-identifying services. The retained services are only the inputs needed
to deterministically replay `plannerd` through its public process-replay seam.

The expected JSON files were produced from the identical final Experimental and
TN-NoDEC implementations in `sp-dev-rs408@64d9c54e2c` and
`sp-dev-egpu@6108f38d09`. Those planner/MPC source blobs are byte-identical
between the two references.
