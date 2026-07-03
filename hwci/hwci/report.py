"""Render run metrics + regression comparison to Markdown (and optional plots)."""
from __future__ import annotations

from pathlib import Path


def _fmt(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.3f}" if abs(v) < 1000 else f"{v:.0f}"
    return str(v)


def render_markdown(metrics: dict, comparison: dict | None = None,
                    meta: dict | None = None) -> str:
    out: list[str] = []
    out.append("# AM32 Hardware-CI Report\n")
    if meta:
        out.append("## Run\n")
        for k in ("target", "profile", "mode", "git_sha", "firmware_version",
                  "motor", "prop", "pole_pairs", "timestamp", "aborted",
                  "perf_read_errors"):
            if k in meta and meta[k] is not None:
                out.append(f"- **{k}**: {meta[k]}")
        out.append("")

    s = metrics["summary"]
    if comparison is not None:
        verdict = "✅ PASS" if comparison["passed"] else "❌ FAIL"
        out.append(f"## Verdict: {verdict}\n")

    out.append("## Summary\n")
    out.append("| metric | value |")
    out.append("|---|---|")
    for k, v in s.items():
        out.append(f"| {k} | {_fmt(v)} |")
    out.append("")

    pts = metrics.get("steady_points", [])
    if pts:
        out.append("## Steady-state operating points\n")
        cols = ["segment", "throttle", "rpm", "thrust_gf", "current_a",
                "voltage_v", "elec_power_w", "eff_gf_per_w",
                "ctrl_exec_us_max", "cpu_load_pct",
                "zc_jitter_pct", "zc_jitter_max_pct"]
        out.append("| " + " | ".join(cols) + " |")
        out.append("|" + "---|" * len(cols))
        for p in pts:
            out.append("| " + " | ".join(_fmt(p.get(c)) for c in cols) + " |")
        out.append("")

    d = metrics.get("demag", {})
    out.append("## Demag / desync\n")
    out.append(f"- events: **{d.get('event_count', 0)}**")
    out.append(f"- bemf-timeout samples: {d.get('bemf_timeout_samples', 0)}")
    out.append(f"- commutation-spike samples: {d.get('comm_spike_samples', 0)}")
    out.append(f"- ESC-eRPM vs stand-RPM mismatch samples: "
               f"{d.get('esc_rpm_mismatch_samples', 0)}")
    out.append("")

    if comparison is not None:
        out.append("## Regression checks (vs baseline)\n")
        out.append("| check | baseline | current | pass | rule |")
        out.append("|---|---|---|---|---|")
        for c in comparison["checks"]:
            mark = "✅" if c["pass"] else "❌"
            out.append(f"| {c['name']} | {_fmt(c['baseline'])} | "
                       f"{_fmt(c['current'])} | {mark} | {c['note']} |")
        out.append("")

    return "\n".join(out)


def write_report(run_dir: str | Path, metrics: dict,
                 comparison: dict | None = None, meta: dict | None = None,
                 plots: bool = True) -> Path:
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    md = render_markdown(metrics, comparison, meta)
    report_path = run_dir / "report.md"
    report_path.write_text(md)
    if plots:
        try:
            _write_plots(run_dir, metrics)
        except Exception:
            pass  # plotting is best-effort / optional
    return report_path


def _write_plots(run_dir: Path, metrics: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pts = metrics.get("steady_points", [])
    if not pts:
        return
    thr = [p["throttle"] * 100 for p in pts]

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    axes[0, 0].plot(thr, [p["thrust_gf"] for p in pts], "o-")
    axes[0, 0].set(title="Thrust", xlabel="throttle %", ylabel="gf")
    axes[0, 1].plot(thr, [p["eff_gf_per_w"] for p in pts], "o-", color="green")
    axes[0, 1].set(title="Efficiency", xlabel="throttle %", ylabel="g/W")
    axes[1, 0].plot(thr, [p["cpu_load_pct"] for p in pts], "o-", color="red")
    axes[1, 0].set(title="CPU load", xlabel="throttle %", ylabel="%")
    axes[1, 1].plot(thr, [p["ctrl_exec_us_max"] for p in pts], "o-", color="purple")
    axes[1, 1].axhline(50, ls="--", color="gray", label="20kHz budget")
    axes[1, 1].set(title="Control-loop exec", xlabel="throttle %", ylabel="us")
    axes[1, 1].legend()
    fig.tight_layout()
    fig.savefig(run_dir / "summary.png", dpi=110)
    plt.close(fig)
