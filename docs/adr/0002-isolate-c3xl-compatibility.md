# Isolate C3XL compatibility behind a hardware profile

C3XL behavior is implemented by a C3XL Profile and a Panda Startup adapter rather than global device-type or Panda-type hardcoding. The JihuLab `c3xl-dev` release is a behavioral reference, while upstream sunnypilot and Panda remain the source baseline; this keeps compatibility local and makes upstream updates auditable.

## Consequences

The physical device identity and the effective C++ logging/Panda capabilities may differ, so raw values must remain observable before profile overrides are applied. AGNOS boot-chain images are allowed to update automatically only when they match the C3XL Boot-chain Allowlist; Panda bootstub updates are not inherited automatically from upstream or the reference release.
