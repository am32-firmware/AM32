"""Render run metrics + regression comparison to Markdown (and optional plots)."""
from __future__ import annotations

import json
import textwrap
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
                   diff_rows: list | None = None,
                   settings_rows: list | None = None, *,
                   log=None) -> Path | None:
    """Render one auto-tune session to a multi-page PDF (``tune_report.pdf``).

    Best-effort: uses matplotlib's ``PdfPages`` so it needs nothing beyond the
    existing optional ``plot`` extra. Returns the PDF path on success, or
    ``None`` if matplotlib is unavailable / rendering fails (the Markdown
    report is written by the caller regardless).

    ``diff_rows`` is the ``Settings.diff`` output ``[(name, default, best)]``.
    ``settings_rows`` is the full default/best tunable setting table.
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
            _pdf_settings_pages(plt, pdf, settings_rows or [])
            _pdf_progress_page(plt, pdf, manifest, result)
            _pdf_tables_pages(plt, pdf, manifest)
            _pdf_raw_data_pages(plt, pdf, out_dir, manifest)
    except Exception as e:
        _log(f"PDF report failed to render: {e}")
        try:
            pdf_path.unlink()
        except OSError:
            pass
        return None
    return pdf_path


_PAGE = (8.27, 11.69)        # A4 portrait, inches
_PAGE_LS = (11.69, 8.27)     # A4 landscape (wide tables)
_CONFIRMED_GREEN = "#1a7f37"
_DEFAULT_GRAY = "#57606a"
_BORDER_GRAY = "#d0d7de"
_HEADER_BG = "#f6f8fa"


def _wrap(v, width: int) -> str:
    s = _fmt(v) if not isinstance(v, str) else v
    return "\n".join(textwrap.wrap(s, width)) if len(s) > width else s


def _place_table(ax, rows: list, cols: list, widths: list,
                 fontsize: float = 8.0, line_h: float = 0.032) -> None:
    """Left-aligned table pinned to the top of ``ax`` with EXPLICIT column
    width fractions (auto_set_column_width lets wide tables overflow the
    page and clip - seen on real trial tables). Cell text may contain
    newlines (pre-wrapped); row heights follow the tallest cell."""
    ax.axis("off")
    if not rows:
        ax.text(0.5, 0.5, "(none)", ha="center", va="center",
                transform=ax.transAxes, color=_DEFAULT_GRAY)
        return
    tbl = ax.table(cellText=rows, colLabels=cols, cellLoc="left",
                   colLoc="left", loc="upper left", colWidths=widths)
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(fontsize)
    lines_in_row: dict[int, int] = {}
    for (r, _c), cell in tbl.get_celld().items():
        n = cell.get_text().get_text().count("\n") + 1
        lines_in_row[r] = max(lines_in_row.get(r, 1), n)
    for (r, _c), cell in tbl.get_celld().items():
        cell.set_height(line_h * (lines_in_row[r] + 0.6))
        cell.set_edgecolor(_BORDER_GRAY)
        cell.PAD = 0.02
        if r == 0:
            cell.set_facecolor(_HEADER_BG)
            cell.set_text_props(fontweight="bold")


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
        f"winner overrides: {_fmt_ov(ov)}",
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
    _place_table(fig.add_axes([0.06, 0.29, 0.60, 0.15]), rows,
                 ["setting", "default", "best"], [0.50, 0.25, 0.25],
                 line_h=0.10)

    # -- stage winners (was its own near-empty page) --
    fig.text(0.06, 0.25, "Stage winners", fontsize=13,
             fontweight="bold", va="top")
    stage_rows = [[name, _wrap(_fmt_ov(s.get("winner")), 58),
                   _fmt(s.get("winner_score"))]
                  for name, s in manifest.get("stages", {}).items()]
    _place_table(fig.add_axes([0.06, 0.05, 0.88, 0.18]), stage_rows,
                 ["stage", "winner", "score (g/W, norm)"],
                 [0.15, 0.62, 0.23], line_h=0.085)
    pdf.savefig(fig)
    plt.close(fig)


def _pdf_progress_page(plt, pdf, manifest: dict, result: dict):
    from matplotlib.ticker import MaxNLocator
    from matplotlib.transforms import blended_transform_factory

    trials = [t for t in manifest.get("trials", []) if not t.get("discarded")]
    fig, (ax_prog, ax_abba) = plt.subplots(2, 1, figsize=_PAGE)
    fig.suptitle("Tuning progress", fontsize=18, fontweight="bold", x=0.06,
                 ha="left", y=0.97)

    # scored trials: raw + normalized g/W vs trial index; anchors (incumbent
    # re-runs) marked distinctly so drift is readable at a glance
    idx_raw = [(t["index"], t["score_raw"]) for t in trials
               if t.get("score_raw") is not None]
    idx_norm = [(t["index"], t["score_norm"]) for t in trials
                if t.get("score_norm") is not None]
    anchors = [(t["index"], t["score_raw"]) for t in trials
               if t.get("kind") == "anchor"
               and t.get("score_raw") is not None]
    dq = [t["index"] for t in trials if t.get("disqualified")]

    if idx_raw:
        xs, ys = zip(*idx_raw)
        ax_prog.plot(xs, ys, "o-", color="#0969da", label="raw g/W", ms=4)
    if idx_norm:
        xs, ys = zip(*idx_norm)
        ax_prog.plot(xs, ys, "s--", color="#bf8700",
                     label="drift-normalized g/W", ms=4)
    if anchors:
        xs, ys = zip(*anchors)
        ax_prog.plot(xs, ys, "o", mfc="white", mec="#0969da", ms=8,
                     ls="none", label="anchor (incumbent re-run)")
    if dq:
        # park DQ markers in their own band under the data instead of on
        # top of the minimum score (they have no score of their own)
        ys = [y for _, y in idx_raw + idx_norm]
        lo, hi = (min(ys), max(ys)) if ys else (0.0, 1.0)
        band = lo - 0.08 * (hi - lo or 1.0)
        ax_prog.plot(dq, [band] * len(dq), "x", color="#cf222e", ms=8,
                     mew=2, ls="none", label="disqualified (no score)")

    # stage bands: alternating background + label per contiguous stage
    trans = blended_transform_factory(ax_prog.transData, ax_prog.transAxes)
    spans: list[tuple[str, int, int]] = []
    for t in sorted(trials, key=lambda t: t["index"]):
        if spans and spans[-1][0] == t["stage"]:
            spans[-1] = (t["stage"], spans[-1][1], t["index"])
        else:
            spans.append((t["stage"], t["index"], t["index"]))
    for i, (name, a, b) in enumerate(spans):
        if i % 2:
            ax_prog.axvspan(a - 0.5, b + 0.5, color="#afb8c1", alpha=0.15,
                            lw=0)
        ax_prog.text((a + b) / 2.0, 1.015, name, transform=trans,
                     ha="center", va="bottom", fontsize=8,
                     color=_DEFAULT_GRAY)

    ax_prog.set(xlabel="trial index", ylabel="objective (g/W)")
    ax_prog.set_title("Objective per trial", pad=22)
    ax_prog.xaxis.set_major_locator(MaxNLocator(integer=True))
    if idx_raw or idx_norm or dq:
        ax_prog.legend(loc="best", fontsize=8)
    ax_prog.grid(True, alpha=0.3)

    # ABBA paired deltas (winner - default), median line
    deltas = result.get("paired_deltas") or []
    if deltas:
        colors = ["#1a7f37" if d > 0 else "#cf222e" for d in deltas]
        ax_abba.bar(range(1, len(deltas) + 1), deltas, color=colors,
                    alpha=0.8, width=0.5)
        med = result.get("median_paired_delta")
        if med is not None:
            ax_abba.axhline(med, ls="--", color="#0969da",
                            label=f"median {med:g} g/W")
        ax_abba.axhline(0, color="#57606a", lw=0.8)
        ax_abba.set_xticks(list(range(1, len(deltas) + 1)))
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


def _pdf_settings_pages(plt, pdf, settings_rows: list) -> None:
    if not settings_rows:
        return
    rows = [[_fmt(r.get("setting")), _fmt(r.get("offset")),
             _fmt(r.get("default")), _fmt(r.get("best")),
             "yes" if r.get("changed") else ""]
            for r in settings_rows]
    for i, chunk in enumerate(_paginate_rows(rows, 34)):
        fig = plt.figure(figsize=_PAGE_LS)
        title = "Full settings" if i == 0 else f"Full settings (cont. {i + 1})"
        fig.text(0.04, 0.94, title, fontsize=18, fontweight="bold", va="top")
        _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                     ["setting", "offset", "default", "best", "changed"],
                     [0.42, 0.10, 0.16, 0.16, 0.12], line_h=0.026)
        pdf.savefig(fig)
        plt.close(fig)


def _fmt_ov(ov) -> str:
    """Compact one-line overrides: {'a': 1, 'b': 2} -> 'a=1, b=2'."""
    if not ov:
        return "default"
    if isinstance(ov, dict):
        return ", ".join(f"{k}={v}" for k, v in ov.items())
    return str(ov)


_TRIAL_COLS = ["#", "stage", "kind", "overrides", "raw g/W", "norm g/W",
               "disqualified"]
_TRIAL_WIDTHS = [0.04, 0.09, 0.11, 0.40, 0.075, 0.075, 0.21]


def _fmt_raw(v) -> str:
    if isinstance(v, (dict, list)):
        return json.dumps(v, sort_keys=True)
    return _fmt(v)


def _md_cell(v) -> str:
    return _fmt_raw(v).replace("|", "\\|").replace("\n", "<br>")


def _load_trial_metrics(out_dir: str | Path,
                        manifest: dict) -> list[tuple[dict, dict]]:
    out = []
    root = Path(out_dir)
    for e in manifest.get("trials", []):
        path = root / e.get("dir", "") / "metrics.json"
        try:
            metrics = json.loads(path.read_text())
        except Exception as exc:
            metrics = {"_load_error": f"{path}: {exc}"}
        out.append((e, metrics))
    return out


def _paginate_rows(rows: list[list[str]],
                   line_budget: int) -> list[list[list[str]]]:
    pages: list[list[list[str]]] = [[]]
    lines = 0
    for row in rows:
        n = max(cell.count("\n") + 1 for cell in row)
        if pages[-1] and lines + n > line_budget:
            pages.append([])
            lines = 0
        pages[-1].append(row)
        lines += n
    return pages


def _iter_run_metrics(metrics: dict):
    for name, value in metrics.get("summary", {}).items():
        yield name, value
    for section in ("demag", "startup"):
        block = metrics.get(section)
        if not isinstance(block, dict):
            continue
        for name, value in block.items():
            yield f"{section}.{name}", value


def render_tune_raw_markdown(out_dir: str | Path, manifest: dict) -> str:
    """Long-form high-level raw metrics from every tune trial."""
    data = _load_trial_metrics(out_dir, manifest)
    out: list[str] = []
    out.append("## Run summary raw data\n")
    out.append("| # | stage | kind | profile | metric | value |")
    out.append("|---|---|---|---|---|---|")
    for e, metrics in data:
        if metrics.get("_load_error"):
            out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                       f"{_md_cell(e.get('kind'))} | {_md_cell(e.get('profile'))} | "
                       f"metrics.json | {_md_cell(metrics['_load_error'])} |")
            continue
        for name, value in _iter_run_metrics(metrics):
            out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                       f"{_md_cell(e.get('kind'))} | {_md_cell(e.get('profile'))} | "
                       f"{_md_cell(name)} | {_md_cell(value)} |")
    out.append("")

    out.append("## Steady-point raw data\n")
    out.append("| # | stage | kind | profile | segment | metric | value |")
    out.append("|---|---|---|---|---|---|---|")
    for e, metrics in data:
        if metrics.get("_load_error"):
            continue
        for point in metrics.get("steady_points", []):
            segment = point.get("segment")
            for name, value in point.items():
                if name == "segment":
                    continue
                out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                           f"{_md_cell(e.get('kind'))} | "
                           f"{_md_cell(e.get('profile'))} | "
                           f"{_md_cell(segment)} | {_md_cell(name)} | "
                           f"{_md_cell(value)} |")
    out.append("")
    return "\n".join(out)


def _pdf_metric_rows(out_dir: str | Path,
                     manifest: dict) -> tuple[list[list[str]], list[list[str]]]:
    summary_rows = []
    steady_rows = []
    for e, metrics in _load_trial_metrics(out_dir, manifest):
        stage_kind = f"{e.get('stage')}/{e.get('kind')}"
        if metrics.get("_load_error"):
            summary_rows.append([_fmt(e.get("index")), _wrap(stage_kind, 22),
                                 _fmt(e.get("profile")), "metrics.json",
                                 _wrap(metrics["_load_error"], 70)])
            continue
        for name, value in _iter_run_metrics(metrics):
            summary_rows.append([_fmt(e.get("index")), _wrap(stage_kind, 22),
                                 _fmt(e.get("profile")), _wrap(name, 28),
                                 _wrap(_fmt_raw(value), 60)])
        for point in metrics.get("steady_points", []):
            segment = point.get("segment")
            for name, value in point.items():
                if name == "segment":
                    continue
                steady_rows.append([_fmt(e.get("index")), _wrap(stage_kind, 20),
                                    _wrap(_fmt(segment), 18), _wrap(name, 28),
                                    _wrap(_fmt_raw(value), 56)])
    return summary_rows, steady_rows


def _pdf_raw_data_pages(plt, pdf, out_dir: str | Path, manifest: dict) -> None:
    summary_rows, steady_rows = _pdf_metric_rows(out_dir, manifest)
    for title, rows, cols, widths in (
            ("Run summary raw data", summary_rows,
             ["#", "stage/kind", "profile", "metric", "value"],
             [0.05, 0.18, 0.12, 0.28, 0.37]),
            ("Steady-point raw data", steady_rows,
             ["#", "stage/kind", "segment", "metric", "value"],
             [0.05, 0.18, 0.16, 0.28, 0.33])):
        if not rows:
            continue
        for i, chunk in enumerate(_paginate_rows(rows, 30)):
            fig = plt.figure(figsize=_PAGE_LS)
            page_title = title if i == 0 else f"{title} (cont. {i + 1})"
            fig.text(0.04, 0.94, page_title, fontsize=18,
                     fontweight="bold", va="top")
            _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                         cols, widths, fontsize=7.2, line_h=0.026)
            pdf.savefig(fig)
            plt.close(fig)


def _pdf_tables_pages(plt, pdf, manifest: dict):
    # trials table: landscape + explicit widths + wrapped text (a portrait
    # auto-width table clipped both edges on real sessions), paginated on a
    # LINE budget so wrapped disqualification reasons don't overflow a page
    trial_rows = []
    for e in manifest.get("trials", []):
        dq = "; ".join(e["disqualified"]) if e.get("disqualified") else ""
        if e.get("discarded"):
            dq = (dq + " " if dq else "") + "(discarded: pack swap)"
        trial_rows.append([
            _fmt(e.get("index")), _fmt(e.get("stage")), _fmt(e.get("kind")),
            _wrap(_fmt_ov(e.get("overrides")), 62), _fmt(e.get("score_raw")),
            _fmt(e.get("score_norm")), _wrap(dq, 38)])

    for i, chunk in enumerate(_paginate_rows(trial_rows, 30)):
        fig = plt.figure(figsize=_PAGE_LS)
        title = "Trials" if i == 0 else f"Trials (cont. {i + 1})"
        fig.text(0.04, 0.94, title, fontsize=18, fontweight="bold", va="top")
        _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                     _TRIAL_COLS, _TRIAL_WIDTHS, line_h=0.026)
        pdf.savefig(fig)
        plt.close(fig)


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
