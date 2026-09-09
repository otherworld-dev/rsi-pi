"""End-to-end offline verification of the RSIPI_Max feature set.

tests/test_max_features.py exercises the API against stub clients, which
proves the guards and the arithmetic but never puts a byte on the wire.
These tests run the real client against the emulated controller over UDP
loopback, so a value has to survive serialisation, the telegram, the
controller's own state model and the reply before it is read back.

That distinction matters here: every one of these features is unverified on
hardware, so the offline path is the only evidence there is that the plumbing
works at all.
"""
import socket
import threading
import time
import xml.etree.ElementTree as ET

import pytest

from RSIPI import RSIAPI, context
from RSIPI.rsi_echo_server import EchoServer

CLIENT_PORT = 59418


def _free_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


@pytest.fixture(scope="module")
def max_config(tmp_path_factory):
    """The Max config, pointed at loopback so nothing touches a real network."""
    tree = ET.parse(context("max"))
    root = tree.getroot()
    root.find("CONFIG/IP_NUMBER").text = "127.0.0.1"
    root.find("CONFIG/PORT").text = str(CLIENT_PORT)
    path = tmp_path_factory.mktemp("max") / "RSI_EthernetConfig_Max.xml"
    tree.write(str(path))
    return str(path)


@pytest.fixture(scope="module")
def stack(max_config):
    if not _free_port(CLIENT_PORT):
        pytest.skip(f"UDP port {CLIENT_PORT} is busy")

    server = EchoServer(max_config, mode="relative")
    server.start()

    api = RSIAPI(max_config, rsi_mode="relative")
    api.start()
    if not api.wait_for_connection(15):
        api.stop()
        server.stop()
        pytest.skip("no loopback connection to the emulated controller")

    yield api, server

    api.stop()
    server.stop()


def _settle(seconds=0.6):
    """Let a few 4 ms cycles carry the change both ways."""
    time.sleep(seconds)


class TestAppliedCorrectionMonitorsOverTheWire:
    """The monitor must track the correction the controller actually applied.

    Both values are compared only after the motion has STOPPED. In relative
    mode a held correction accumulates every 4 ms cycle, and the client's copy
    lags the controller by one sync interval, so comparing them mid-move
    compares two moving targets.
    """

    def test_monitor_reports_the_correction_actually_applied(self, stack):
        api, server = stack
        assert api.monitoring.get_applied_correction() != {}, \
            "PosCorrMon missing - wrong context?"

        api.motion.update_cartesian(X=1.0)
        _settle(0.3)
        api.motion.update_cartesian(X=0.0)   # stop accumulating
        _settle()

        applied = api.monitoring.get_applied_correction()
        assert applied["X"] > 0.5, "monitor never reported the correction"
        assert applied["X"] == pytest.approx(server.state["PosCorrMon"]["X"], abs=1e-6)
        # And it must agree with where the robot actually ended up.
        assert applied["X"] == pytest.approx(
            server.state["RIst"]["X"] - server.base_pose["RIst"]["X"], abs=1e-6)

    def test_joint_monitor_reports_joint_corrections(self, stack):
        api, server = stack
        api.motion.update_joints(A1=0.5)
        _settle(0.3)
        api.motion.update_joints(A1=0.0)
        _settle()

        applied = api.monitoring.get_applied_joint_correction()
        assert applied["A1"] > 0.1
        assert applied["A1"] == pytest.approx(server.state["AxisCorrMon"]["A1"], abs=1e-6)

    def test_monitor_stays_zero_when_nothing_is_commanded(self, stack):
        """The point of the monitor: it reports what was APPLIED, so an axis
        that was never corrected must read zero even while others move."""
        api, server = stack
        assert api.monitoring.get_applied_correction()["Y"] == pytest.approx(0.0)


class TestOverrideOverTheWire:
    def test_reads_the_controllers_override(self, stack):
        api, _server = stack
        assert api.monitoring.get_override() == 100

    def test_setting_override_reaches_the_controller_and_comes_back(self, stack):
        api, server = stack
        api.monitoring.set_override(30)
        _settle()

        assert server.state["OvPro"] == 30, "override never reached the controller"
        assert api.monitoring.get_override() == 30, "read-back never returned"
        api.monitoring.set_override(100)
        _settle()


class TestSenPintOverTheWire:
    def test_write_then_read_round_trip(self, stack):
        api, server = stack
        api.krl.write_sen_pint(7)
        _settle()

        assert server.state["SenPInt1"] == 7
        assert api.krl.read_sen_pint() == 7

    def test_integer_stays_an_integer(self, stack):
        """SEN_PINT is the INTEGER channel - a float on the wire would be wrong."""
        api, server = stack
        api.krl.write_sen_pint(42)
        _settle()
        assert isinstance(server.state["SenPInt1"], int)
        assert api.krl.read_sen_pint() == 42


class TestAnalogueOverTheWire:
    def test_reads_an_analogue_input_from_the_controller(self, stack):
        api, server = stack
        server.state["AnIn1"] = 3.25
        _settle()
        assert api.io.read_analog() == pytest.approx(3.25, abs=1e-3)

    def test_writing_an_analogue_output_reaches_the_controller(self, stack):
        api, server = stack
        api.io.set_analog(0.75)
        _settle()
        # AnOut1 is a RECEIVE variable, so it lands in the server's own view of
        # what the sensor sent rather than in its state model.
        assert server.last_received is not None
        assert "AnOut1" in server.last_received


class TestMotorCurrents:
    def test_motor_currents_arrive_without_costing_a_channel(self, stack):
        api, server = stack
        server.state["MACur"]["A1"] = 12.5
        _settle()
        assert api.monitoring.get_force()["A1"] == pytest.approx(12.5, abs=1e-3)
