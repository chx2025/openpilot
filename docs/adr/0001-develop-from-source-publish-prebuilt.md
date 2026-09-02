# Develop from the source baseline and publish a prebuilt snapshot

Changes are maintained on the Source Baseline identified by each sunnypilot `dev` release message, then built into a separate Prebuilt Snapshot for deployment. The orphan, force-published `dev` history is intentionally not used as a merge or rebase base: it optimizes startup and distribution, while `master-dev` preserves reviewable source history.

The maintained source and device branch is `dev-sp-egpu`. Branch/channel
classification remains metadata for update and release behavior; it must not be
an onroad gate. Ignition and the upstream safety, thermal, storage, setup, update,
and explicit-offroad conditions remain authoritative regardless of branch name.

Only release tooling may create the `prebuilt` marker. Runtime settings such as
Quick Boot must not fabricate it in a source checkout, because doing so silently
skips required native builds. A future prebuilt artifact must preserve the source
commit and channel identity recorded by its release builder.
