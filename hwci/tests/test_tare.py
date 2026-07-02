"""Pre-run tare: zero the load cells only AFTER the ESC signal is up at zero
throttle. AM32 beeps the motor whenever it has NO input signal (and again as
it arms), and each beep is a torque pulse through the mount - a tare taken on
a beeping ESC bakes that twitching into the load-cell zero (seen on the bench
as a wandering thrust offset between runs)."""
from types import SimpleNamespace

from hwci.runner import ARM_TUNE_SETTLE_S, TARE_SETTLE_S, tare_for_run


class _Rig:
    """Fake stand + throttle sharing ONE event log, so tests can assert
    ordering ACROSS the two objects - the whole point of the choreography."""

    def __init__(self, thrust_n=0.0, read_raises=False):
        self.events: list = []
        self._thrust_n = thrust_n
        self._read_raises = read_raises
        self.throttle = SimpleNamespace(arm=lambda: self.events.append("arm"))
        self.stand = SimpleNamespace(tare=lambda: self.events.append("tare"),
                                     read_sample=self._read_sample)

    def _read_sample(self):
        self.events.append("read")
        if self._read_raises:
            raise RuntimeError("stand went away")
        return SimpleNamespace(thrust_n=self._thrust_n)

    def settle(self, seconds: float) -> None:
        self.events.append(("settle", seconds))


def test_signal_is_up_and_arm_tune_finished_before_tare():
    rig = _Rig()
    tare_for_run(rig.stand, rig.throttle, settle=rig.settle)
    assert rig.events == [
        "arm",                          # signal at zero: beacon stops
        ("settle", ARM_TUNE_SETTLE_S),  # arm tune stops shaking the motor
        "tare",
        ("settle", TARE_SETTLE_S),      # post-tare readings settle
        "read",
    ]


def test_residual_is_reported_in_gf():
    rig = _Rig(thrust_n=0.0980665)  # exactly 10 gf
    residual = tare_for_run(rig.stand, rig.throttle, settle=rig.settle)
    assert abs(residual - 10.0) < 1e-9


def test_residual_read_is_best_effort_but_tare_is_not():
    rig = _Rig(read_raises=True)
    residual = tare_for_run(rig.stand, rig.throttle, settle=rig.settle)
    assert residual is None          # unreadable residual doesn't kill the run
    assert "tare" in rig.events      # ... but the tare itself already happened
