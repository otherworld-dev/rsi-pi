"""Tests for the API added alongside the RSIPI_Max context.

Covers applied-correction monitors (POSCORRMON/AXISCORRMON), analogue I/O
(ANIN/MAP2ANOUT), $SEN_PINT (SEN_PINT/MAP2SEN_PINT) and program override
(OV_PRO/MAP2OV_PRO).

Every one of these needs an object that only the Max context wires, so each
method must fail with a clear, actionable error on a context that lacks it
rather than silently returning a default - that is the failure mode this
project keeps getting bitten by.
"""
import pytest
from unittest.mock import MagicMock

from RSIPI import context
from RSIPI.config_parser import ConfigParser
from RSIPI.exceptions import RSIVariableError
from RSIPI.io_api import IOAPI
from RSIPI.krl_api import KRLAPI
from RSIPI.monitoring_api import MonitoringAPI


def _client(send=None, receive=None):
    client = MagicMock()
    client.send_variables = send if send is not None else {}
    client.receive_variables = receive if receive is not None else {}
    client.safety_manager.validate.side_effect = lambda path, value: value
    return client


# --------------------------------------------------------------- monitors

class TestAppliedCorrectionMonitors:
    def test_reports_applied_cartesian_correction(self):
        api = MonitoringAPI(_client(send={"PosCorrMon": {"X": 4.9, "Y": 0.0}}))
        assert api.get_applied_correction()["X"] == 4.9

    def test_reports_applied_joint_correction(self):
        api = MonitoringAPI(_client(send={"AxisCorrMon": {"A6": 0.02}}))
        assert api.get_applied_joint_correction()["A6"] == 0.02

    def test_empty_when_context_has_no_monitor(self):
        api = MonitoringAPI(_client(send={}))
        assert api.get_applied_correction() == {}
        assert api.get_applied_joint_correction() == {}

    def test_returns_a_copy_not_the_live_dict(self):
        send = {"PosCorrMon": {"X": 1.0}}
        api = MonitoringAPI(_client(send=send))
        api.get_applied_correction()["X"] = 99.0
        assert send["PosCorrMon"]["X"] == 1.0


# ------------------------------------------------------------------ override

class TestOverride:
    def test_reads_override_as_int(self):
        api = MonitoringAPI(_client(send={"OvPro": 100.0}))
        assert api.get_override() == 100

    def test_none_when_not_wired(self):
        assert MonitoringAPI(_client(send={})).get_override() is None

    def test_sets_override(self):
        client = _client(receive={"OvProW": 0})
        assert "30" in MonitoringAPI(client).set_override(30)
        assert client.receive_variables["OvProW"] == 30

    def test_rejects_out_of_range_rather_than_clamping(self):
        client = _client(receive={"OvProW": 0})
        api = MonitoringAPI(client)
        for bad in (0, 101, -5):
            with pytest.raises(ValueError):
                api.set_override(bad)
        assert client.receive_variables["OvProW"] == 0

    def test_raises_when_context_cannot_write_override(self):
        with pytest.raises(RSIVariableError, match="OvProW"):
            MonitoringAPI(_client(receive={})).set_override(50)


# ---------------------------------------------------------------- analogue

class TestAnalogueIO:
    def test_reads_analogue_input(self):
        assert IOAPI(_client(send={"AnIn1": 0.42})).read_analog() == 0.42

    def test_raises_when_no_anin_object(self):
        with pytest.raises(RSIVariableError, match="AnIn1"):
            IOAPI(_client(send={})).read_analog()

    def test_writes_analogue_output(self):
        client = _client(receive={"AnOut1": 0.0})
        IOAPI(client).set_analog(0.75)
        assert client.receive_variables["AnOut1"] == 0.75

    def test_raises_when_no_map2anout_object(self):
        with pytest.raises(RSIVariableError, match="AnOut1"):
            IOAPI(_client(receive={})).set_analog(0.5)


# ---------------------------------------------------------------- SEN_PINT

class TestSenPint:
    def test_reads_integer(self):
        assert KRLAPI(_client(send={"SenPInt1": 7.0})).read_sen_pint() == 7

    def test_raises_when_not_declared(self):
        with pytest.raises(RSIVariableError, match="SenPInt1"):
            KRLAPI(_client(send={})).read_sen_pint()

    def test_writes_integer(self):
        client = _client(receive={"SenPIntW": 0})
        KRLAPI(client).write_sen_pint(3)
        assert client.receive_variables["SenPIntW"] == 3

    def test_raises_when_write_not_declared(self):
        with pytest.raises(RSIVariableError, match="SenPIntW"):
            KRLAPI(_client(receive={})).write_sen_pint(1)


# ----------------------------------------------------------- the context

@pytest.fixture(scope="module")
def parsed():
    return ConfigParser(context("max"))


class TestMaxContext:
    """The Max context must actually declare what those methods need."""

    @pytest.mark.parametrize("tag", ["PosCorrMon", "AxisCorrMon", "MACur",
                                     "AnIn1", "SenPInt1", "OvPro"])
    def test_declares_send_tag(self, parsed, tag):
        assert tag in parsed.send_variables

    @pytest.mark.parametrize("tag", ["AnOut1", "SenPIntW", "OvProW"])
    def test_declares_receive_tag(self, parsed, tag):
        assert tag in parsed.receive_variables

    def test_keeps_everything_joints_had(self, parsed):
        joints = ConfigParser(context("joints"))
        assert set(joints.send_variables) <= set(parsed.send_variables)
        assert set(joints.receive_variables) <= set(parsed.receive_variables)

    def test_has_no_external_axis_variables(self, parsed):
        # The whole point of Max: everything a 6-axis robot can bind, and
        # nothing that needs external axes.
        for absent in ("EKorr", "EIPos", "ESPos", "MECur"):
            assert absent not in parsed.send_variables
            assert absent not in parsed.receive_variables

    def test_axiscorrmon_exposes_only_robot_axes(self, parsed):
        assert set(parsed.send_variables["AxisCorrMon"]) == {
            "A1", "A2", "A3", "A4", "A5", "A6"}


# ------------------------------------------------------------------- STATUS

class TestRobotStatus:
    """STATUS decoding. No shipped context wires a STATUS object - its Type is
    an enum whose number must come from RSIVisual rather than be guessed - so
    the API must return None rather than pretend, and decode correctly once
    someone does add one."""

    def test_none_when_no_status_object_is_wired(self):
        assert MonitoringAPI(_client(send={})).get_robot_status() is None

    def test_raw_value_when_no_meaning_requested(self):
        api = MonitoringAPI(_client(send={"Status1": 3}))
        assert api.get_robot_status(1) == 3

    @pytest.mark.parametrize("value,expected", [
        (0, "OFF"), (3, "CYCLE"), (9, "ERROR")])
    def test_decodes_sensor_interface_state(self, value, expected):
        api = MonitoringAPI(_client(send={"Status1": value}))
        assert api.get_robot_status(1, "Sensor") == expected

    @pytest.mark.parametrize("value,expected", [
        (1, "T1"), (2, "T2"), (3, "AUT"), (4, "EXT")])
    def test_decodes_operating_mode(self, value, expected):
        api = MonitoringAPI(_client(send={"Status1": value}))
        assert api.get_robot_status(1, "Mode_Op") == expected

    def test_ipo_state_is_decoded_as_a_bit_field(self):
        # 65 = 64 (CP) | 1 (ACTIVE) - not the 65th member of an enum.
        api = MonitoringAPI(_client(send={"Status1": 65}))
        assert api.get_robot_status(1, "IPO_State") == "ACTIVE|CP"

    def test_unknown_value_is_reported_not_hidden(self):
        api = MonitoringAPI(_client(send={"Status1": 99}))
        assert "99" in api.get_robot_status(1, "Sensor")

    def test_several_status_objects_are_addressed_by_index(self):
        api = MonitoringAPI(_client(send={"Status1": 3, "Status2": 1}))
        assert api.get_robot_status(1, "Sensor") == "CYCLE"
        assert api.get_robot_status(2, "Mode_Op") == "T1"
