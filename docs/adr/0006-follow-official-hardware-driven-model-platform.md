# Follow the official hardware-driven model platform

The Model Platform follows sunnypilot's official QCOM/Chestnut policy. QCOM
and Chestnut retain independent selected bundles, while physical Chestnut
availability and runtime state determine which slot is active. A healthy,
connected Chestnut uses the Big Model slot; without Chestnut the QCOM Small
Model slot is active. `ModelManager_ActiveSource` is migration data only and
is not authoritative.

## Consequences

- The official Model Selector, ref downloads, verification, queueing, cache
  cleanup, status text, and failure semantics remain the source baseline.
- Selecting a Small Model while Chestnut is healthy prepares the QCOM slot but
  does not make it the active driving model. Safe-eject or disconnect Chestnut
  to use the QCOM slot.
- A C3XL Model Adapter may report hardware capability and preserve the 75
  second load allowance, loading progress, downloaded-bundle readiness,
  compile-CPU selection, UT3G identity, telemetry, and safe eject. It must not
  implement a second product-selection algorithm.
- Legacy USBGPU bundle and catalog Params migrate once into the official
  Chestnut slots. Selector-v17 and off-catalog bundles are not retained as
  active choices.
- The model files remain in place; a matching selector-v18 ref and artifact
  hash can be verified and reused without downloading again.
- Default Big requires its official compiled artifact. Downloaded LM/TT/IDM
  bundles remain independent of the optional big ONNX source.
