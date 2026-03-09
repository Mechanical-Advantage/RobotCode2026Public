"""Additional BallCounter tests for coverage of increment_paused and edge cases."""

import pytest

from hub_counter.gpio.ball_counter import BallCounter


def test_increment_paused_increments_paused_total():
    counter = BallCounter()
    counts = counter.increment_paused()
    assert counts.paused_total == 1
    assert counts.total == 0  # active channels unchanged


def test_increment_paused_multiple_times():
    counter = BallCounter()
    counter.increment_paused()
    counts = counter.increment_paused()
    assert counts.paused_total == 2


def test_increment_paused_triggers_callback():
    counter = BallCounter()
    received = []
    counter.on_count_change(lambda c: received.append(c.paused_total))
    counter.increment_paused()
    assert received == [1]


def test_set_counts_wrong_length_raises():
    counter = BallCounter()
    with pytest.raises(ValueError, match="Expected 4 channels"):
        counter.set_counts([1, 2, 3])


def test_callback_exception_does_not_stop_other_callbacks():
    counter = BallCounter()
    second_called = []

    def bad_cb(counts):
        raise RuntimeError("oops")

    def good_cb(counts):
        second_called.append(counts.total)

    counter.on_count_change(bad_cb)
    counter.on_count_change(good_cb)
    counter.increment(0)

    assert second_called == [1]


def test_reset_clears_paused_total():
    counter = BallCounter()
    counter.increment_paused()
    counter.increment_paused()
    counts = counter.reset()
    assert counts.paused_total == 0
