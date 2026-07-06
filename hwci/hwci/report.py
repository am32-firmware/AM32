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

    st = metrics.get("startup", {})
    if st.get("attempts"):
        out.append("## Start attempts\n")
        out.append(f"- attempts: {st['attempts']}, **failures: {st['failures']}**")
        out.append(f"- time to running: mean {_fmt(st.get('time_to_run_ms_mean'))} ms, "
                   f"max {_fmt(st.get('time_to_run_ms_max'))} ms")
        fails = [a["segment"] for a in st.get("per_attempt", []) if not a["success"]]
        if fails:
            out.append(f"- failed segments: {', '.join(fails)}")
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


# --------------------------------------------------------------------------
# Auto-tune PDF report
# --------------------------------------------------------------------------
def write_tune_pdf(out_dir: str | Path, manifest: dict, result: dict,
                   diff_rows: list | None = None, *,
                   log=None) -> Path | None:
    """Render one auto-tune session to a multi-page PDF (``tune_report.pdf``).

    Best-effort: uses matplotlib's ``PdfPages`` so it needs nothing beyond the
    existing optional ``plot`` extra. Returns the PDF path on success, or
    ``None`` if matplotlib is unavailable / rendering fails (the Markdown
    report is written by the caller regardless).

    ``diff_rows`` is the ``Settings.diff`` output ``[(name, default, best)]``.
    """
    def _log(msg: str) -> None:
        if log is not None:
            log(msg)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages
    except Exception as e:  # matplotlib not installed -> skip, don't fail tune
        _log(f"PDF report skipped (matplotlib unavailable: {e}); "
             "install the 'plot' extra to enable it")
        return None

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / "tune_report.pdf"
    try:
        with PdfPages(pdf_path) as pdf:
            _pdf_cover_page(plt, pdf, manifest, result, diff_rows or [])
            _pdf_progress_page(plt, pdf, manifest, result)
            _pdf_tables_pages(plt, pdf, manifest)
    except Exception as e:
        _log(f"PDF report failed to render: {e}")
        try:
            pdf_path.unlink()
        except OSError:
            pass
        return None
    return pdf_path


_PAGE = (8.27, 11.69)   # A4 portrait, inches
_CONFIRMED_GREEN = "#1a7f37"
_DEFAULT_GRAY = "#57606a"


def _pdf_cover_page(plt, pdf, manifest: dict, result: dict, diff_rows: list):
    fig = plt.figure(figsize=_PAGE)
    fig.text(0.06, 0.955, "AM32 Auto-Tune Report", fontsize=22,
             fontweight="bold", va="top")
    fig.text(0.06, 0.925, str(manifest.get("spec_name", "")), fontsize=13,
             color=_DEFAULT_GRAY, va="top")

    # -- metadata block --
    addr = manifest.get("eeprom_address")
    meta_lines = [
        ("mode", manifest.get("mode")),
        ("git_sha", manifest.get("git_sha")),
        ("eeprom_address",
         f"0x{addr:08x}" if isinstance(addr, int) else addr),
        ("battery_cells", manifest.get("battery_cells")),
        ("jitter reference (default)",
         f"{manifest.get('jitter_reference')} %"),
        ("pack swaps", len(manifest.get("pack_events", []))),
        ("trials run", len(manifest.get("trials", []))),
    ]
    y = 0.885
    for k, v in meta_lines:
        fig.text(0.06, y, f"{k}", fontsize=10, color=_DEFAULT_GRAY, va="top")
        fig.text(0.42, y, _fmt(v), fontsize=10, va="top")
        y -= 0.024

    # -- verdict banner --
    confirmed = bool(result.get("confirmed"))
    banner_color = _CONFIRMED_GREEN if confirmed else _DEFAULT_GRAY
    verdict = ("WINNER CONFIRMED" if confirmed
               else "DEFAULT SETTINGS KEPT")
    ax = fig.add_axes([0.06, 0.60, 0.88, 0.09])
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                               facecolor=banner_color, alpha=0.14,
                               edgecolor=banner_color, linewidth=1.5))
    ax.text(0.5, 0.5, verdict, transform=ax.transAxes, ha="center",
            va="center", fontsize=20, fontweight="bold", color=banner_color)

    ov = result.get("winner_overrides") or {}
    detail = [
        f"winner overrides: {ov or '{} (default)'}",
        (f"median paired delta: {_fmt(result.get('median_paired_delta'))} g/W "
         f"over {len(result.get('paired_deltas') or [])} ABBA pairs"),
        f"winner constraint failures: {result.get('winner_constraint_failures', 0)}",
    ]
    st = result.get("startup")
    if st is not None:
        detail.append(f"startup: {st.get('failed')}/{st.get('cycles')} failed")
    y = 0.565
    for line in detail:
        fig.text(0.06, y, line, fontsize=11, va="top")
        y -= 0.026

    # -- settings diff table --
    fig.text(0.06, 0.46, "Settings diff (default → best)", fontsize=13,
             fontweight="bold", va="top")
    if diff_rows:
        rows = [[name, _fmt(a), _fmt(b)] for name, a, b in diff_rows]
    else:
        rows = [["(no change — default settings won)", "", ""]]
    ax_t = fig.add_axes([0.06, 0.08, 0.88, 0.35])
    ax_t.axis("off")
    tbl = ax_t.table(cellText=rows,
                     colLabels=["setting", "default", "best"],
                     colLoc="left", cellLoc="left", loc="upper left")
    _style_table(tbl, header_cols=3)
    pdf.savefig(fig)
    plt.close(fig)


def _pdf_progress_page(plt, pdf, manifest: dict, result: dict):
    trials = [t for t in manifest.get("trials", []) if not t.get("discarded")]
    fig, (ax_prog, ax_abba) = plt.subplots(2, 1, figsize=_PAGE)
    fig.suptitle("Tuning progress", fontsize=18, fontweight="bold", x=0.06,
                 ha="left", y=0.97)

    # scored trials: raw + normalized g/W vs trial index
    idx_raw = [(t["index"], t["score_raw"]) for t in trials
               if t.get("score_raw") is not None]
    idx_norm = [(t["index"], t["score_norm"]) for t in trials
                if t.get("score_norm") is not None]
    dq = [t["index"] for t in trials if t.get("disqualified")]

    if idx_raw:
        xs, ys = zip(*idx_raw)
        ax_prog.plot(xs, ys, "o-", color="#0969da", label="raw g/W", ms=4)
    if idx_norm:
        xs, ys = zip(*idx_norm)
        ax_prog.plot(xs, ys, "s--", color="#bf8700",
                     label="drift-normalized g/W", ms=4)
    if dq:
        ymin = min([y for _, y in idx_raw], default=0)
        ax_prog.plot(dq, [ymin] * len(dq), "x", color="#cf222e", ms=8,
                     label="disqualified")
    ax_prog.set(xlabel="trial index", ylabel="objective (g/W)",
                title="Objective per trial")
    if idx_raw or idx_norm or dq:
        ax_prog.legend(loc="best", fontsize=8)
    ax_prog.grid(True, alpha=0.3)

    # ABBA paired deltas (winner - default), median line
    deltas = result.get("paired_deltas") or []
    if deltas:
        colors = ["#1a7f37" if d > 0 else "#cf222e" for d in deltas]
        ax_abba.bar(range(1, len(deltas) + 1), deltas, color=colors,
                    alpha=0.8)
        med = result.get("median_paired_delta")
        if med is not None:
            ax_abba.axhline(med, ls="--", color="#0969da",
                            label=f"median {med:g} g/W")
        ax_abba.axhline(0, color="#57606a", lw=0.8)
        ax_abba.legend(loc="best", fontsize=8)
    else:
        ax_abba.text(0.5, 0.5, "no ABBA paired deltas", ha="center",
                     va="center", transform=ax_abba.transAxes,
                     color=_DEFAULT_GRAY)
    ax_abba.set(xlabel="ABBA pair", ylabel="winner − default (g/W)",
                title="Finals: interleaved paired deltas")
    ax_abba.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    pdf.savefig(fig)
    plt.close(fig)


def _fmt_ov(ov) -> str:
    """Compact one-line overrides: {'a': 1, 'b': 2} -> 'a=1, b=2'."""
    if not ov:
        return "default"
    if isinstance(ov, dict):
        return ", ".join(f"{k}={v}" for k, v in ov.items())
    return str(ov)


def _pdf_tables_pages(plt, pdf, manifest: dict):
    # stages table
    stages = manifest.get("stages", {})
    stage_rows = [[name, _fmt_ov(s.get("winner")), _fmt(s.get("winner_score"))]
                  for name, s in stages.items()]
    _table_page(plt, pdf, "Stage winners", stage_rows,
                ["stage", "winner", "score (g/W, norm)"])

    # trials table (paginated)
    trial_rows = []
    for e in manifest.get("trials", []):
        dq = "; ".join(e["disqualified"]) if e.get("disqualified") else ""
        if e.get("discarded"):
            dq = (dq + " " if dq else "") + "(discarded: pack swap)"
        trial_rows.append([
            _fmt(e.get("index")), _fmt(e.get("stage")), _fmt(e.get("kind")),
            _fmt_ov(e.get("overrides")), _fmt(e.get("score_raw")),
            _fmt(e.get("score_norm")), dq])
    cols = ["#", "stage", "kind", "overrides", "raw g/W", "norm g/W",
            "disqualified"]
    per_page = 26
    if not trial_rows:
        _table_page(plt, pdf, "Trials", [], cols)
        return
    for i in range(0, len(trial_rows), per_page):
        chunk = trial_rows[i:i + per_page]
        title = "Trials" if i == 0 else f"Trials (cont. {i // per_page + 1})"
        _table_page(plt, pdf, title, chunk, cols)


def _table_page(plt, pdf, title: str, rows: list, cols: list):
    fig = plt.figure(figsize=_PAGE)
    fig.text(0.06, 0.955, title, fontsize=18, fontweight="bold", va="top")
    ax = fig.add_axes([0.04, 0.04, 0.92, 0.86])
    ax.axis("off")
    if not rows:
        ax.text(0.5, 0.5, "(none)", ha="center", va="center",
                transform=ax.transAxes, color=_DEFAULT_GRAY)
    else:
        tbl = ax.table(cellText=rows, colLabels=cols, cellLoc="left",
                       colLoc="left", loc="upper center")
        _style_table(tbl, header_cols=len(cols))
    pdf.savefig(fig)
    plt.close(fig)


def _style_table(tbl, *, header_cols: int) -> None:
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    tbl.auto_set_column_width(list(range(header_cols)))
    tbl.scale(1, 1.4)
    for (r, _c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#d0d7de")
        if r == 0:
            cell.set_facecolor("#f6f8fa")
            cell.set_text_props(fontweight="bold")


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
