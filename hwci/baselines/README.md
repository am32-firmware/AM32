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

Re-baseline deliberately (and note why in the commit) when the motor/prop/setup
changes or after an intentional, validated performance change.
