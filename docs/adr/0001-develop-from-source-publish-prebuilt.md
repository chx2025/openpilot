# Develop from the source baseline and publish a prebuilt snapshot

Changes are maintained on the Source Baseline identified by each sunnypilot `dev` release message, then built into a separate Prebuilt Snapshot for deployment. The orphan, force-published `dev` history is intentionally not used as a merge or rebase base: it optimizes startup and distribution, while `master-dev` preserves reviewable source history.

The deployed Prebuilt Snapshot branch must end in `-tici`, for example `dev-c3xl-prebuilt-tici`, so the existing upstream hardware/channel check remains enabled without a C3XL-specific bypass. The stock workflow's automatic `-prebuilt` suffix is not used unchanged because that final name would no longer end in `-tici`.
