"""Tests for the external-axis (EKorr/EIPos) feature set added on top of the
Cartesian/joint correction paths:

  - rsi_limit_parser.parse_rsi_limits() reading AXISCORREXT -> EKorr.E1-E6
    limits (accepting both LowerLimE1 and LowerLim1 spellings),
  - motion_api.move_external_axis() writing EKorr through the same
    SafetyManager-validated path as update_cartesian/update_joints,
  - the ONLYSEND guard in RSIClient.publish_corrections(),
  - an end-to-end loopback proving a client-side EKorr write reaches the
    (emulated) controller and advances its EIPos state.

rsi_stack (tests/conftest.py) is hard-coded to the minimal
RSI_EthernetConfig.xml, which does not declare EKorr - the loopback test
below therefore builds its own Full-config rig locally.
"""
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import pytest

from RSIPI.context import CONTEXT_DIR

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.exceptions import RSIStateError, RSIVariableError
from RSIPI.motion_api import MotionAPI
from RSIPI.rsi_client import RSIClient
from RSIPI.rsi_limit_parser import parse_rsi_limits
from RSIPI.safety_manager import SafetyManager

from tests.conftest import RecordingEchoServer

REPO_ROOT = os.path.join(os.path.dirname(__file__), '..')
FULL_CONFIG_FILE = os.path.join(CONTEXT_DIR, 'RSI_EthernetConfig_Full.xml')
RSI_XML_FULL = os.path.join(CONTEXT_DIR, 'RSIPI_Full.rsi.xml')

ECHO_SERVER_PORT = 50000


def _full_client_port():
    root = ET.parse(FULL_CONFIG_FILE).getroot()
    return int(root.find("CONFIG/PORT").text.strip())


FULL_CLIENT_PORT = _full_client_port()


# ---------------------------------------------------------------------------
# stub client - plain dicts + a real SafetyManager, exactly what MotionAPI /
# ToolsAPI need from `client` (mirrors the pattern in tests/test_io_api.py).
# ---------------------------------------------------------------------------

class _StubClient:
    def __init__(self, send_vars=None, receive_vars=None):
        self.send_variables = send_vars if send_vars is not None else {}
        self.receive_variables = receive_vars if receive_vars is not None else {}
        self.safety_manager = SafetyManager()


# ===========================================================================
# parse_rsi_limits: AXISCORREXT -> EKorr.E1-E6
# ===========================================================================

class TestParseRsiLimitsExternalAxis:
    """AXISCORREXT limits become EKorr.E1-E6 - the variable move_external_axis
    actually validates against (not, say, ExtAct or ESPos)."""

    def test_ekorr_limits_from_axiscorrext(self):
        limits = parse_rsi_limits(RSI_XML_FULL)
        for i in range(1, 7):
            assert limits[f"EKorr.E{i}"] == (-5.0, 5.0)

    def test_other_correction_limits_still_present(self):
        """Sanity check the parser wasn't narrowed to only AXISCORREXT."""
        limits = parse_rsi_limits(RSI_XML_FULL)
        assert limits["RKorr.X"] == (-500.0, 500.0)
        assert limits["AKorr.A1"] == (-180.0, 180.0)


# ===========================================================================
# MotionAPI.move_external_axis
# ===========================================================================

class TestMoveExternalAxis:
    def test_raises_when_ekorr_not_declared(self):
        client = _StubClient(receive_vars={})
        motion = MotionAPI(client)

        with pytest.raises(RSIVariableError, match="EKorr"):
            motion.move_external_axis("E1", 2.5)

    def test_writes_ekorr_when_declared(self):
        client = _StubClient(receive_vars={"EKorr": {"E1": 0.0}})
        motion = MotionAPI(client)

        result = motion.move_external_axis("E1", 2.5)

        assert client.receive_variables["EKorr"]["E1"] == 2.5
        assert "EKorr.E1" in result


# ===========================================================================
# ONLYSEND guard on RSIClient.publish_corrections
# ===========================================================================

def _write_onlysend_config(tmp_path, source_config_file):
    """Copy an RSI_EthernetConfig.xml with <ONLYSEND> forced to TRUE."""
    tree = ET.parse(source_config_file)
    root = tree.getroot()
    onlysend_elem = root.find("CONFIG/ONLYSEND")
    assert onlysend_elem is not None, "source config has no <ONLYSEND> element"
    onlysend_elem.text = "TRUE"
    out_path = tmp_path / "onlysend_config.xml"
    tree.write(str(out_path))
    return str(out_path)


class TestOnlysendClientGuard:
    """publish_corrections() must refuse to write when the config is
    ONLYSEND=TRUE - the robot expects no reply at all in that mode.

    RSIClient spawns a NetworkProcess in __init__, but it only binds a
    socket after start_event is set (see network_handler.NetworkProcess.run);
    since start() is never called here, this never touches the network and
    cannot conflict with any port the rest of the suite is using.
    """

    def test_publish_corrections_raises_when_onlysend(self, tmp_path, config_file):
        onlysend_config = _write_onlysend_config(tmp_path, config_file)

        client = None
        try:
            client = RSIClient(onlysend_config)
            with pytest.raises(RSIStateError, match="ONLYSEND"):
                client.publish_corrections({"RKorr": {"X": 1.0}})
        except OSError as e:  # pragma: no cover - environment dependent
            pytest.skip(f"Could not construct RSIClient for ONLYSEND guard test: {e}")
        finally:
            if client is not None:
                client.stop()

    def test_update_variable_raises_when_onlysend(self, tmp_path, config_file):
        """The low-level write path must refuse too.

        update_variable() writes straight into receive_variables, which the
        network loop never reads in ONLYSEND - so without this guard a
        correction looks accepted and silently goes nowhere. Every
        higher-level write (update_cartesian, update_joints, set_output)
        funnels through here.
        """
        from RSIPI.tools_api import ToolsAPI

        onlysend_config = _write_onlysend_config(tmp_path, config_file)

        client = None
        try:
            client = RSIClient(onlysend_config)
            with pytest.raises(RSIStateError, match="ONLYSEND"):
                ToolsAPI(client).update_variable("RKorr.X", 1.0)
        except OSError as e:  # pragma: no cover - environment dependent
            pytest.skip(f"Could not construct RSIClient for ONLYSEND guard test: {e}")
        finally:
            if client is not None:
                client.stop()


# ===========================================================================
# Loopback: EKorr write -> EIPos advance (Full config on both ends)
# ===========================================================================
#
# KNOWN SRC BUG (reported in the test-writing summary, NOT fixed here):
# network_handler.NetworkProcess and rsi_echo_server.EchoServer both read
# UDP datagrams into a fixed `recvfrom(1024)` buffer. The canonical
# RSI_EthernetConfig_Full.xml's default <Rob> reply serializes to ~2.4KB and
# its <Sen> reply to ~2.2KB - mostly the 120 Tech.C*/Tech.T* sub-values that
# 12 "INTERNAL" ELEMENTs expand to via ConfigParser.internal_structure. Both
# directions exceed 1024 bytes, so on Windows every cycle raises
# "[WinError 10040] message ... larger than the internal message buffer",
# the 250-consecutive-error watchdog trips, and the NetworkProcess exits
# before the client ever sees a first packet (connected_event never gets
# set). This was confirmed directly against the shipped file - see the
# trimmed variant built below, which keeps the RECEIVE section (RKorr/
# AKorr/DiO/EKorr, with their canonical HOLDON attributes) and the SEND
# elements this test needs (RIst/EIPos/Delay) byte-for-byte as declared in
# the shipped config, only dropping the oversized Tech.* INTERNAL elements
# that are irrelevant to external-axis corrections - so the round trip below
# can actually complete over real UDP loopback.

def _build_udp_safe_full_config(tmp_path):
    """A trimmed copy of RSI_EthernetConfig_Full.xml that fits in the
    library's fixed 1024-byte recv buffers (see the module comment above).
    """
    tree = ET.parse(FULL_CONFIG_FILE)
    root = tree.getroot()

    keep_send_tags = {"DEF_RIst", "DEF_Delay", "DEF_EIPos"}
    send_elements = root.find("SEND/ELEMENTS")
    for element in list(send_elements):
        if element.get("TAG") not in keep_send_tags:
            send_elements.remove(element)

    receive_elements = root.find("RECEIVE/ELEMENTS")
    for element in list(receive_elements):
        if element.get("TAG", "").startswith("DEF_Tech."):
            receive_elements.remove(element)

    out_path = tmp_path / "full_config_udp_safe.xml"
    tree.write(str(out_path))
    return str(out_path)


def _start_full_client(config_path, mode="relative", **client_kwargs):
    client = RSIClient(config_path, rsi_mode=mode, **client_kwargs)
    thread = threading.Thread(target=client.start, daemon=True)
    client.thread = thread
    thread.start()
    return client


@pytest.fixture
def full_stack(require_ports, tmp_path):
    """RecordingEchoServer + RSIClient, both driven by a UDP-safe trim of
    the Full config (see the module comment above for why the trim exists).

    rsi_stack (tests/conftest.py) is hard-coded to the minimal
    RSI_EthernetConfig.xml, which has no EKorr/EIPos - external-axis
    round-trips need this local Full-config variant instead.
    """
    require_ports(ECHO_SERVER_PORT, FULL_CLIENT_PORT)

    config_path = _build_udp_safe_full_config(tmp_path)

    server = RecordingEchoServer(config_path, delay_ms=4, mode="relative")
    server.start()

    client = _start_full_client(config_path, "relative", cycle_time=0.004)

    if not client.wait_for_connection(15.0):
        server.stop()
        client.stop()
        pytest.fail(
            "RSIClient never received a packet from the Full-config echo "
            "server (waited 15s)")

    yield server, client

    # Server first: it stops within one 4ms cycle, whereas the network
    # process can sit in a 1s recvfrom timeout (matches conftest.rsi_stack).
    try:
        server.stop()
    except Exception:
        pass
    try:
        client.stop()
    except Exception:
        pass


class TestExternalAxisLoopback:
    """End-to-end: a client-side EKorr write reaches the (emulated)
    controller over real UDP loopback and advances its EIPos state, exactly
    like RKorr does for RIst."""

    def test_ekorr_round_trip_advances_eipos(self, full_stack):
        server, client = full_stack
        assert "EKorr" in client.receive_variables

        motion = MotionAPI(client)
        result = motion.move_external_axis("E1", 1.0)
        assert "EKorr.E1" in result

        # Relative mode integrates the held correction every accepted cycle,
        # so this grows well past 1.0 mm over half a second at ~4ms/cycle -
        # assert directionally rather than pin an exact cycle count.
        time.sleep(0.5)

        eipos_e1 = server.robot_state("EIPos")["E1"]
        assert eipos_e1 > 0
        assert eipos_e1 >= 1.0
        assert server.faulty_packets == 0


# ===========================================================================
# E-stop / zeroing must cover EKorr, not just RKorr and AKorr
# ===========================================================================

class TestExternalAxisIsACorrectionToo:
    """EKorr was missing from _CORRECTION_KEYS on both the client and the
    network process. Consequences: zero_corrections() left a held EKorr in
    place, and the E-stop substitution froze RKorr/AKorr but kept
    transmitting the external-axis correction."""

    def test_both_key_sets_include_ekorr(self):
        from RSIPI.network_handler import NetworkProcess
        assert "EKorr" in RSIClient._CORRECTION_KEYS
        assert "EKorr" in NetworkProcess._CORRECTION_KEYS

    def test_zero_corrections_zeroes_ekorr(self):
        client = None
        try:
            client = RSIClient(FULL_CONFIG_FILE)
            client.receive_variables["EKorr"] = {"E1": 2.5, "E2": 0.0}
            client.zero_corrections()
            assert client.receive_variables["EKorr"] == {"E1": 0.0, "E2": 0.0}
        except OSError as e:  # pragma: no cover - environment dependent
            pytest.skip(f"Could not construct RSIClient: {e}")
        finally:
            if client is not None:
                client.stop()
