"""Tune session checkpointing: crash -> resume reuses completed trials and
reaches the identical winner; pack-swap events pause/prompt correctly."""
import json

import pytest
import yaml

from hwci.sim import MotorParams
from hwci.tuner import SimTuneBackend, TunePaused, Tuner, tune_spec_from_dict


def spec_dict() -> dict:
    return {
        "name": "resume-test",
        "probe": {"dwell_s": 1.0},
        # tight noise floor: with noise=0 the winner is decided purely by
        # deterministic scores, so crash/resume equality is exact
        "objective": {"min_power_w": 5.0, "noise_floor_pct": 0.1},
        "anchors_every": 4,
        "parameters": {
            "advance_level": {"values": [18, 22, 26, 30, 34],
                              "refine_step": 2},
            "pwm_frequency": {"values": [8, 24, 48]},
        },
        "stages": [
            {"name": "advance", "sweep": "advance_level"},
            {"name": "pwm", "sweep": "pwm_frequency"},
        ],
        "finals": {"profile": "tune_probe", "repeats": 1,
                   "startup_check": False},
    }


def make_backend() -> SimTuneBackend:
    # noise=0 so a resumed session is bit-deterministic: reused trials carry
    # their recorded scores and re-executed ones recompute the same values.
    # The optimum sits OFF the grid midpoints (30.5, not 31) so no two grid
    # values score exactly equal and the winner never falls to a jitter
    # tie-break (whose RNG phase differs after a resume).
    return SimTuneBackend(motor_params=MotorParams(
        pole_pairs=7, demag_prone=True, advance_optimum=30.5,
        startup_fail_ref=100.0), noise=0.0)


def make_tuner(out, backend, *, resume=False, **kw) -> Tuner:
    d = spec_dict()
    return Tuner(tune_spec_from_dict(d), backend, out,
                 spec_text=yaml.safe_dump(d), no_prompt=True, resume=resume,
                 log=lambda s: None, **kw)


def test_crash_then_resume_reaches_identical_winner(tmp_path):
    # Reference: uninterrupted run.
    ref = make_tuner(tmp_path / "ref", make_backend())
    ref_result = ref.run()
    total = len(ref.ledger)

    # Crash after K completed trials.
    K = 7

    def crash(index, plan):
        if index >= K:
            raise RuntimeError("injected crash")

    crashing = make_tuner(tmp_path / "crash", make_backend(),
                          before_trial=crash)
    with pytest.raises(RuntimeError, match="injected crash"):
        crashing.run()
    m = json.loads((tmp_path / "crash" / "manifest.json").read_text())
    assert len(m["trials"]) == K          # checkpointed up to the crash
    assert m["result"] is None

    # Simulate a crash that left a partial trial dir behind (no trial.json).
    stray = tmp_path / "crash" / "trials" / f"T{K:03d}-halfway"
    stray.mkdir(parents=True)
    (stray / "samples.csv").write_text("t\n")

    # Resume with a FRESH backend (new process, device state unknown).
    resumed = make_tuner(tmp_path / "crash", make_backend(), resume=True)
    result = resumed.run()

    assert result["winner_overrides"] == ref_result["winner_overrides"]
    assert result["confirmed"] == ref_result["confirmed"]
    # Completed trials were reused, only the remainder was executed.
    assert resumed.executed == total - K
    assert len(resumed.ledger) == total
    # The stray partial dir was quarantined, not silently reused.
    incomplete = list((tmp_path / "crash" / "trials").glob("*.incomplete*"))
    assert incomplete
    # Resume re-programmed the incumbent blob before continuing (a crash may
    # have left arbitrary trial settings flashed).
    assert (tmp_path / "crash" / "resume_settings.bin").exists()


def test_resume_of_finished_session_reuses_everything(tmp_path):
    t1 = make_tuner(tmp_path / "s", make_backend())
    r1 = t1.run()
    t2 = make_tuner(tmp_path / "s", make_backend(), resume=True)
    r2 = t2.run()
    assert t2.executed == 0
    assert r2["winner_overrides"] == r1["winner_overrides"]


# --------------------------------------------------------------------------
# pack swap
# --------------------------------------------------------------------------
def test_low_pack_prompts_for_swap_and_records_event(tmp_path):
    backend = make_backend()
    backend.voltage_fn = lambda: 12.0     # 3.0 V/cell on a 4S: too low
    prompts = []

    def prompt(msg):
        prompts.append(msg)
        backend.voltage_fn = None         # "pack swapped" - healthy again
        return ""

    d = spec_dict()
    d["battery_cells"] = 4
    tuner = Tuner(tune_spec_from_dict(d), backend, tmp_path / "t",
                  spec_text=yaml.safe_dump(d), prompt_fn=prompt,
                  log=lambda s: None)
    result = tuner.run()
    assert len(prompts) == 1
    assert "too low" in prompts[0]
    m = json.loads((tmp_path / "t" / "manifest.json").read_text())
    assert len(m["pack_events"]) == 1
    assert m["pack_events"][0]["event"] == "pack_swap"
    assert result["winner_overrides"]      # the tune still completed


def test_low_pack_with_no_prompt_pauses_cleanly_then_resumes(tmp_path):
    backend = make_backend()
    backend.voltage_fn = lambda: 12.0
    d = spec_dict()
    d["battery_cells"] = 4
    tuner = Tuner(tune_spec_from_dict(d), backend, tmp_path / "t",
                  spec_text=yaml.safe_dump(d), no_prompt=True,
                  log=lambda s: None)
    with pytest.raises(TunePaused, match="resume"):
        tuner.run()
    # manifest was checkpointed before exiting
    assert (tmp_path / "t" / "manifest.json").exists()

    # after the swap (fresh healthy backend), --resume finishes the session
    resumed = make_tuner(tmp_path / "t", make_backend(), resume=True)
    result = resumed.run()
    assert result["winner_overrides"]
