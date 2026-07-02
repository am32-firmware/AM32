"""_tail_reset_tick: the mid-segment perf-stats reset must land strictly
before the metrics tail window, not at its boundary.

reset_stats() clears the firmware register over an SWD round trip, and in
realtime mode perf_get() reads an independent background poller's CACHED
value (~2ms interval) - so the tick that ISSUES the reset can still observe
the pre-reset cached sample. Observed on the bench: an 877us arming-tune
transient latched at a segment's first tick was still visible in the exact
sample the reset was issued on, making the "steady" max equal the raw run
max. The fix needs a margin, not an exact boundary match - and that margin
must be measured against the SAME tail_start_index() metrics.py uses: an
earlier version of this fix recomputed the tail boundary with its own
independently-rounded formula (round() vs metrics.py's int() truncation),
which silently disagreed by a tick for some segment lengths (e.g. n=300,
fraction=0.8) - exactly the kind of gap this race hides in.
"""
from hwci.metrics import tail_start_index
from hwci.runner import RESET_MARGIN_TICKS, _tail_reset_tick


def test_reset_lands_before_metrics_tail_with_margin():
    for n, fraction in [(1000, 0.5), (600, 0.5), (100, 0.3), (300, 0.8)]:
        reset_tick = _tail_reset_tick(n, fraction)
        tail_start = tail_start_index(n, fraction)
        assert reset_tick <= tail_start - RESET_MARGIN_TICKS, (
            f"n={n} fraction={fraction}: reset_tick={reset_tick} leaves "
            f"less than {RESET_MARGIN_TICKS}-tick margin before tail_start={tail_start}")


def test_reset_tick_never_negative_on_short_segments():
    # A segment shorter than the margin must clamp to 0, not go negative.
    assert _tail_reset_tick(5, 0.5) == 0
    assert _tail_reset_tick(1, 0.9) == 0
