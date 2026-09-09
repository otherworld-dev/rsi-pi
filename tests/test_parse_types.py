"""Inbound values keep the type the config declared - by VALUE, not by text.

A controller applies its Precision setting to everything it sends, so a Bool
can arrive as "0.00" or "1.000000". Judging a Bool by string comparison would
read "0.00" as True, and until 2026-09-09 grouped values (dotted tags, sent
as attributes) were simply float()-ed regardless of their declared type.
"""
import pytest

from RSIPI.network_handler import NetworkProcess, _coerce_like


class TestCoerceLike:
    @pytest.mark.parametrize("text,expected", [
        ("0", False), ("0.00", False), ("0.000000", False), ("", False),
        ("1", True), ("1.00", True), ("1.000000", True), ("2", True),
        ("false", False), ("FALSE", False), ("true", True), ("True", True),
    ])
    def test_bool_is_judged_by_value(self, text, expected):
        assert _coerce_like(False, text) is expected

    def test_long_from_decimal_text(self):
        assert _coerce_like(0, "100.00") == 100
        assert isinstance(_coerce_like(0, "100.00"), int)

    def test_double(self):
        assert _coerce_like(0.0, "3.25") == 3.25

    def test_garbage_keeps_the_previous_value(self):
        assert _coerce_like(7, "n/a") == 7
        assert _coerce_like(1.5, "n/a") == 1.5

    def test_string_passes_through(self):
        assert _coerce_like("", " hello ") == "hello"


class TestGroupedValuesKeepDeclaredType:
    def test_grouped_bools_arrive_as_bools(self):
        target = {"Digout": {"o1": False, "o2": False, "o3": False}}
        NetworkProcess._parse_received_data(
            '<Rob Type="KUKA"><Digout o1="1.00" o2="0.00" o3="0.000000" /></Rob>',
            target)
        assert target["Digout"] == {"o1": True, "o2": False, "o3": False}
        assert all(isinstance(v, bool) for v in target["Digout"].values())

    def test_grouped_doubles_unchanged(self):
        """The correction/position path must be exactly as before."""
        target = {"RIst": {"X": 0.0, "Y": 0.0}}
        NetworkProcess._parse_received_data(
            '<Rob Type="KUKA"><RIst X="1234.56" Y="-78.90" /></Rob>', target)
        assert target["RIst"] == {"X": 1234.56, "Y": -78.9}

    def test_internal_groups_are_not_truncated_to_their_placeholder(self):
        """ConfigParser seeds RIst/AIPos/MACur... from internal_structure,
        whose values are int 0 placeholders rather than declared types. The
        first cut of this fix coerced to them and turned 12.5 A into 12 - the
        loopback suite caught it. Positions would have gone the same way."""
        target = {"MACur": {"A1": 0, "A2": 0}, "RIst": {"X": 0, "Y": 0}}
        NetworkProcess._parse_received_data(
            '<Rob Type="KUKA"><MACur A1="12.5" A2="0.25" />'
            '<RIst X="1234.56" Y="-0.5" /></Rob>', target)
        assert target["MACur"] == {"A1": 12.5, "A2": 0.25}
        assert target["RIst"] == {"X": 1234.56, "Y": -0.5}
        assert all(isinstance(v, float) for v in target["RIst"].values())

    def test_scalar_long_and_double(self):
        target = {"OvPro": 0, "AnIn1": 0.0}
        NetworkProcess._parse_received_data(
            '<Rob Type="KUKA"><OvPro>100</OvPro><AnIn1>3.25</AnIn1></Rob>', target)
        assert target["OvPro"] == 100 and isinstance(target["OvPro"], int)
        assert target["AnIn1"] == 3.25
