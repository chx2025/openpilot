# UT3G V1.6 dual firmware host adapter

## Runtime identity

```text
VID:PID       ADD1:0002
manufacturer  tiny
product       custom d1377a01-UT3G-DUAL
serial        AAAABBBC1377
```

The identity is intentionally outside comma's official Chestnut updater domain
(`ADD1/3801:0001`). An unmodified official openpilot therefore ignores the dual
device instead of replacing it with `ed4e39b7`.

## Ownership split

- Runtime discovery, modeld, tinygrad, passive status, and safe eject accept
  exact official current firmware plus exact UT3G dual firmware.
- The bundled official flasher continues to discover only official `0001` and
  ROM identities. It cannot discover or persistently write `ADD1:0002`.
- Factory `2065:2463` remains outside runtime and official flash ownership in
  this SP adapter.

The dual product parser is strict: lowercase eight-hex build ID followed by
`-UT3G-DUAL`, manufacturer `tiny`, and PID `0002`. Dirty or malformed builds
are neither run nor handed to the official updater.

## ROM boundary

If a dual device loses its active identity and falls into `174c:2463/2464`
ROM, USB identity alone no longer proves whether it was an official Chestnut or
an enrolled UT3G dual device. This adapter currently preserves comma's official
ROM recovery behavior. Before hardware installation, choose and test a
persistent host enrollment gate if ROM must also be protected from ed4 restore.

## No installation authorization

These changes only provide host support for an offline candidate. They do not
authorize firmware installation and do not include a dual firmware flasher.
