# Select the model platform explicitly

The model manager keeps independent QCOM and USBGPU selections and addresses
downloads by the catalog bundle `ref`. `ModelManager_ActiveSource` records the
user's explicit platform choice. Physical eGPU presence is a runtime capability
check only: it may prevent a USBGPU model from starting while the adapter is
absent, but it must not silently replace the user's QCOM/USBGPU selection.

## Consequences

- Both catalogs remain visible and cached independently, whether or not an eGPU
  is currently connected.
- Selecting a downloaded bundle activates its owning slot without downloading
  it again; selecting another bundle writes a stable ref request.
- The pre-split `ModelManager_ActiveBundle` is migrated according to its stored
  platform metadata instead of being copied into both slots.
- Cache cleanup preserves the selected artifacts in both slots.
- The deprecated index request and hardware-requirement flag remain for one
  migration release, but new UI and runtime decisions do not depend on them.
- WebRTC cloud logging is unrelated to model selection and is not enabled by
  this migration.
