"""Tests for hub_counter.core.models."""

import pytest

from hub_counter.core.models import BallCounts, MatchInfo, SystemStatus


class TestBallCounts:
    def test_default_channels_are_zero(self):
        bc = BallCounts()
        assert bc.channels == [0, 0, 0, 0]
        assert bc.total == 0
        assert bc.paused_total == 0

    def test_total_computed_on_init(self):
        bc = BallCounts(channels=[1, 2, 3, 4])
        assert bc.total == 10

    def test_wrong_length_channels_replaced_with_zeros(self):
        bc = BallCounts(channels=[1, 2])
        assert bc.channels == [0, 0, 0, 0]
        assert bc.total == 0

    def test_to_dict_includes_all_fields(self):
        bc = BallCounts(channels=[1, 2, 3, 4], paused_total=5)
        d = bc.to_dict()
        assert d["channels"] == [1, 2, 3, 4]
        assert d["total"] == 10
        assert d["paused_total"] == 5

    def test_to_dict_channels_is_copy(self):
        bc = BallCounts(channels=[1, 2, 3, 4])
        d = bc.to_dict()
        d["channels"][0] = 99
        assert bc.channels[0] == 1  # original unaffected

    def test_from_dict_roundtrip(self):
        bc = BallCounts(channels=[5, 10, 15, 20], paused_total=3)
        bc2 = BallCounts.from_dict(bc.to_dict())
        assert bc2.channels == [5, 10, 15, 20]
        assert bc2.paused_total == 3
        assert bc2.total == 50

    def test_from_dict_missing_keys_uses_defaults(self):
        bc = BallCounts.from_dict({})
        assert bc.channels == [0, 0, 0, 0]
        assert bc.paused_total == 0


class TestMatchInfo:
    def test_defaults(self):
        m = MatchInfo()
        assert m.match_time == 0.0
        assert m.is_disabled is True
        assert m.is_autonomous is False
        assert m.is_teleop is False

    def test_to_dict(self):
        m = MatchInfo(match_time=30.5, is_teleop=True, is_disabled=False, match_number=7, event_name="TEST")
        d = m.to_dict()
        assert d["match_time"] == 30.5
        assert d["is_teleop"] is True
        assert d["is_disabled"] is False
        assert d["match_number"] == 7
        assert d["event_name"] == "TEST"


class TestSystemStatus:
    def test_defaults(self):
        s = SystemStatus()
        assert s.nt_connected is False
        assert s.match_info is None
        assert s.led_active is False

    def test_to_dict_no_match_info(self):
        s = SystemStatus()
        d = s.to_dict()
        assert d["nt_connected"] is False
        assert d["match_info"] is None
        assert "counts" in d

    def test_to_dict_with_match_info(self):
        s = SystemStatus(match_info=MatchInfo(match_number=3))
        d = s.to_dict()
        assert d["match_info"]["match_number"] == 3

    def test_counts_embedded_in_dict(self):
        bc = BallCounts(channels=[1, 2, 3, 4])
        s = SystemStatus(counts=bc, nt_connected=True)
        d = s.to_dict()
        assert d["counts"]["total"] == 10
        assert d["nt_connected"] is True
