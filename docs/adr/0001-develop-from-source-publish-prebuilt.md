# Develop from the source baseline and publish a prebuilt snapshot

Changes are maintained on the Source Baseline identified by each sunnypilot `dev` release message, then built into a separate Prebuilt Snapshot for deployment. The orphan, force-published `dev` history is intentionally not used as a merge or rebase base: it optimizes startup and distribution, while `master-dev` preserves reviewable source history.

The maintained source and device branch is `dev-sp-egpu`.  That exact published
channel is allowlisted as TICI-compatible in the build-metadata seam; arbitrary
development branches remain rejected by the upstream hardware/channel check.
A future prebuilt artifact must preserve the same source commit and channel
identity instead of renaming the maintained branch or globally bypassing the
check.
