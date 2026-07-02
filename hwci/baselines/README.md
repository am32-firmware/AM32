# Baselines

Committed performance baselines, one JSON per target, e.g.
`ARK_4IN1_F051.json`. Each is the metrics snapshot of a known-good run that the
hardware-CI gate compares new runs against (see `hwci baseline-save` and
`hwci ci --baseline ...`).

These are **rig- and device-specific** (motor, prop, battery, ambient). Capture
on your own bench and commit:

```
hwci ci --profile efficiency_sweep --config rig.yaml --out runs/baseline
hwci baseline-save runs/baseline --out baselines/ARK_4IN1_F051.json
```

While no baseline file exists, `hwci ci --baseline ...` warns and skips the
gate instead of failing, so the first run (or a `save_baseline` workflow
dispatch) can bootstrap it.

Baselines are stamped with `format_version`, the run meta (target, profile),
and per-channel sample coverage; the comparison fails on identity mismatch, on
any missing/NaN gated metric, and on collapsed channel coverage — a baseline
captured with a dead SWD or telemetry channel would otherwise gate nothing.

Re-baseline deliberately (and note why in the commit) when the motor/prop/setup
changes or after an intentional, validated performance change.
