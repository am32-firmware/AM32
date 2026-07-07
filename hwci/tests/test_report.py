"""Auto-tune PDF report rendering (report.write_tune_pdf)."""
import json
import sys

import pytest

from hwci import report

# The whole PDF path is a best-effort matplotlib feature; skip when it's not
# installed (the tune itself never depends on it).
mpl = pytest.importorskip("matplotlib")


def _manifest(**over):
    m = {
        "spec_name": "unit", "mode": "sim", "git_sha": "deadbee",
        "eeprom_address": 0x08007C00, "battery_cells": 6,
        "jitter_reference": 0.8, "pack_events": [],
        "stages": {"advance": {"winner": {"advance_level": 30},
                               "winner_score": 6.30}},
        "trials": [
            {"index": 0, "stage": "baseline", "kind": "baseline",
             "overrides": {}, "score_raw": 5.9, "score_norm": 5.9,
             "disqualified": None, "discarded": False},
            {"index": 1, "stage": "advance", "kind": "trial",
             "overrides": {"advance_level": 30}, "score_raw": 6.3,
             "score_norm": 6.3, "disqualified": None, "discarded": False},
            {"index": 2, "stage": "advance", "kind": "trial",
             "overrides": {"advance_level": 10}, "score_raw": None,
             "score_norm": None, "disqualified": ["demag"],
             "discarded": False},
        ],
    }
    m.update(over)
    return m


def _result(**over):
    r = {"confirmed": True, "winner_overrides": {"advance_level": 30},
         "median_paired_delta": 0.2, "paired_deltas": [0.1, 0.3, -0.05],
         "winner_constraint_failures": 0, "startup": {"failed": 0, "cycles": 5}}
    r.update(over)
    return r


def _is_pdf(path) -> bool:
    return path is not None and path.read_bytes()[:5] == b"%PDF-"


def test_write_tune_pdf_produces_valid_pdf(tmp_path):
    p = report.write_tune_pdf(tmp_path, _manifest(), _result(),
                              [("advance_level", 26, 30)])
    assert p == tmp_path / "tune_report.pdf"
    assert _is_pdf(p)


def test_write_tune_pdf_default_kept_no_deltas(tmp_path):
    # not confirmed, empty diff, no ABBA deltas, no stages/trials
    m = _manifest(stages={}, trials=[])
    r = _result(confirmed=False, winner_overrides={},
                median_paired_delta=None, paired_deltas=[], startup=None)
    p = report.write_tune_pdf(tmp_path, m, r, [])
    assert _is_pdf(p)


def test_write_tune_pdf_paginates_many_trials(tmp_path):
    trials = [{"index": i, "stage": "advance", "kind": "trial",
               "overrides": {"advance_level": i}, "score_raw": 6.0 + i * 0.01,
               "score_norm": 6.0 + i * 0.01, "disqualified": None,
               "discarded": False} for i in range(60)]
    p = report.write_tune_pdf(tmp_path, _manifest(trials=trials), _result(), [])
    assert _is_pdf(p)


def test_render_tune_raw_markdown_includes_trial_metrics(tmp_path):
    trial_dir = tmp_path / "trials" / "T000-baseline"
    trial_dir.mkdir(parents=True)
    (trial_dir / "metrics.json").write_text(json.dumps({
        "summary": {"max_current_a": 12.3, "n_samples": 42},
        "demag": {"event_count": 1},
        "startup": {"attempts": 2, "failures": 0},
        "steady_points": [{"segment": "s50", "rpm": 1000.0,
                           "eff_gf_per_w": 4.2}],
    }))
    manifest = _manifest(trials=[{
        "index": 0, "stage": "baseline", "kind": "baseline",
        "profile": "probe", "overrides": {}, "score_raw": 4.2,
        "score_norm": 4.2, "disqualified": None, "discarded": False,
        "dir": "trials/T000-baseline",
    }])

    md = report.render_tune_raw_markdown(tmp_path, manifest)

    assert "## Run summary raw data" in md
    assert "max_current_a" in md
    assert "demag.event_count" in md
    assert "startup.attempts" in md
    assert "## Steady-point raw data" in md
    assert "eff_gf_per_w" in md


def test_write_tune_pdf_skips_gracefully_without_matplotlib(tmp_path, monkeypatch):
    # Force `import matplotlib` to fail; the tune must not blow up.
    monkeypatch.setitem(sys.modules, "matplotlib", None)
    logs = []
    p = report.write_tune_pdf(tmp_path, _manifest(), _result(), [],
                              log=logs.append)
    assert p is None
    assert not (tmp_path / "tune_report.pdf").exists()
    assert any("matplotlib" in m for m in logs)


def test_fmt_ov_compact():
    assert report._fmt_ov({}) == "default"
    assert report._fmt_ov({"advance_level": 30}) == "advance_level=30"
    assert report._fmt_ov({"a": 1, "b": 2}) == "a=1, b=2"


def test_tune_run_writes_pdf_report(tmp_path):
    # end-to-end: a real sim tune emits tune_report.pdf alongside report.md
    from test_tuner import make_backend, run_tune, small_spec

    _, result = run_tune(tmp_path, small_spec(), make_backend(advance_optimum=33.0))
    pdf = tmp_path / "tune" / "tune_report.pdf"
    assert (tmp_path / "tune" / "report.md").exists()
    assert _is_pdf(pdf)
