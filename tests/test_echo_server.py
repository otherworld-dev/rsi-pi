"""Tests for the RSI echo server (controller emulation) and the ETHERNET Timeout parser.

The echo server emulates the robot side of an RSI link:
  - it owns the IPOC clock,
  - it validates the sensor's <Sen> reply (echoed IPOC + SENTYPE),
  - it counts invalid packets, reports them in DEF_Delay and breaks off RSI
    once the ETHERNET Timeout budget is exceeded.

No real UDP traffic is exercised: the server's socket is bound at construction
(0.0.0.0:50000) and closed during teardown, and transmissions are captured with
a recording stand-in.
"""
import os
import socket
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.config_parser import ConfigParser
from RSIPI.rsi_echo_server import EchoServer
from RSIPI.rsi_limit_parser import parse_ethernet_timeout

REPO_ROOT = os.path.join(os.path.dirname(__file__), '..')
CONFIG_FILE = os.path.join(REPO_ROOT, 'RSI_EthernetConfig.xml')
FULL_CONFIG_FILE = os.path.join(REPO_ROOT, 'rsi_config', 'RSI_EthernetConfig_Full.xml')
RSI_XML_FILE = os.path.join(REPO_ROOT, 'rsi_config', 'RSIPI_Full.rsi.xml')
RSI_FILE = os.path.join(REPO_ROOT, 'rsi_config', 'RSIPI_Full.rsi')

SENTYPE = "ImFree"  # matches <SENTYPE> in RSI_EthernetConfig.xml and the Full config


# --------------------------------------------------------------------------- helpers

def sen_xml(ipoc, sentype=SENTYPE, rkorr=None, extra=""):
    """Build a <Sen> reply exactly as the client would transmit it."""
    korr = ""
    if rkorr:
        attrs = " ".join(f'{k}="{v}"' for k, v in rkorr.items())
        korr = f"<RKorr {attrs} />"
    ipoc_element = "" if ipoc is None else f"<IPOC>{ipoc}</IPOC>"
    return f'<Sen Type="{sentype}">{korr}{extra}{ipoc_element}</Sen>'


class RecordingSocket:
    """Stand-in for the UDP socket that records datagrams instead of sending them."""

    def __init__(self):
        self.sent = []

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)

    def close(self):
        pass


@pytest.fixture
def make_server():
    """Factory creating EchoServers, guaranteeing their sockets are closed afterwards."""
    sockets = []
    servers = []

    def _make(**kwargs):
        kwargs.setdefault("config_file", CONFIG_FILE)
        try:
            server = EchoServer(**kwargs)
        except OSError as e:  # pragma: no cover - environment dependent
            pytest.skip(f"Could not bind UDP 0.0.0.0:50000 for the echo server: {e}")
        servers.append(server)
        sockets.append(server.udp_socket)  # keep the real socket for teardown
        return server

    yield _make

    for server in servers:
        server.running = False
    for sock in sockets:
        try:
            sock.close()
        except Exception:
            pass


@pytest.fixture
def server(make_server):
    """Default echo server: relative mode, 4 ms cycle."""
    return make_server(delay_ms=4)


# ------------------------------------------------------------ parse_ethernet_timeout

class TestParseEthernetTimeout:
    """The ETHERNET object's Timeout parameter is the faulty-packet budget."""

    def test_reads_timeout_from_rsi_xml(self):
        """The .rsi.xml (RSIObject) format exposes Timeout=100."""
        assert parse_ethernet_timeout(RSI_XML_FILE) == 100

    def test_reads_timeout_from_rsi(self):
        """The .rsi (namespaced RSIVisual) format exposes Timeout=100."""
        assert parse_ethernet_timeout(RSI_FILE) == 100

    def test_returns_none_without_ethernet_object(self, tmp_path):
        """A signal-flow file with no ETHERNET/Timeout yields None."""
        f = tmp_path / "no_ethernet.rsi.xml"
        f.write_text(
            '<RSIObjects>'
            '  <RSIObject ObjType="POSCORR" ObjID="POSCORR1">'
            '    <Parameters><Parameter Name="LowerLimX" ParamValue="-10" /></Parameters>'
            '  </RSIObject>'
            '</RSIObjects>'
        )
        assert parse_ethernet_timeout(str(f)) is None

    def test_returns_none_for_unparsable_file(self, tmp_path):
        """A broken/missing file yields None instead of raising."""
        broken = tmp_path / "broken.rsi.xml"
        broken.write_text("<RSIObjects><unclosed>")
        assert parse_ethernet_timeout(str(broken)) is None
        assert parse_ethernet_timeout(str(tmp_path / "does_not_exist.rsi.xml")) is None


# ------------------------------------------------------------------ config structure

class TestDelayStructure:
    """DEF_Delay travels in attribute notation, so it must expand to a dict."""

    def test_delay_expands_to_attribute_dict(self):
        parser = ConfigParser(CONFIG_FILE)
        assert parser.send_variables["Delay"] == {"D": 0}


# ------------------------------------------------------------------ reply validation

class TestValidReply:
    """A reply echoing the transmitted IPOC with the right SENTYPE is applied."""

    def test_corrections_applied(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        start_x = server.state["RIst"]["X"]

        accepted = server.process_reply(
            sen_xml(server.last_sent_ipoc, rkorr={"X": 1.5, "Y": -2.0}))

        assert accepted is True
        assert server.faulty_packets == 0
        assert server.state["RIst"]["X"] == pytest.approx(start_x + 1.5)
        assert server.state["RIst"]["Y"] == pytest.approx(-2.0)

    def test_reply_before_first_transmission_is_tolerated(self, server):
        """Nothing has been sent yet, so there is no IPOC to echo."""
        assert server.last_sent_ipoc is None
        assert server.process_reply(sen_xml(999, rkorr={"Z": 3.0})) is True
        assert server.faulty_packets == 0
        assert server.state["RIst"]["Z"] == pytest.approx(3.0)

    def test_scalar_state_update_applied(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        server.process_reply(
            sen_xml(server.last_sent_ipoc, extra="<DiL>7</DiL>"))
        assert server.state["DiL"] == 7


class TestInvalidReply:
    """Invalid packets are counted and their payload is discarded entirely."""

    def test_ipoc_mismatch_rejected(self, server):
        """The old client behaviour (IPOC + 4) is now rejected."""
        server.udp_socket = RecordingSocket()
        server.send_once()
        start_x = server.state["RIst"]["X"]

        accepted = server.process_reply(
            sen_xml(server.last_sent_ipoc + 4, rkorr={"X": 5.0}))

        assert accepted is False
        assert server.faulty_packets == 1
        assert server.state["RIst"]["X"] == pytest.approx(start_x)

    def test_wrong_sentype_rejected(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        start_x = server.state["RIst"]["X"]

        accepted = server.process_reply(
            sen_xml(server.last_sent_ipoc, sentype="NotMe", rkorr={"X": 5.0}))

        assert accepted is False
        assert server.faulty_packets == 1
        assert server.state["RIst"]["X"] == pytest.approx(start_x)

    def test_missing_ipoc_rejected(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        assert server.process_reply(sen_xml(None, rkorr={"X": 5.0})) is False
        assert server.faulty_packets == 1

    def test_malformed_xml_rejected(self, server):
        assert server.process_reply("<Sen Type=\"ImFree\"><RKorr X=") is False
        assert server.faulty_packets == 1


class TestBreakOff:
    """Exceeding the ETHERNET Timeout budget stops the server, like a real robot."""

    def test_budget_breach_stops_loop(self, make_server):
        server = make_server(delay_ms=4, timeout_packets=3)
        server.udp_socket = RecordingSocket()
        server.send_once()
        bad = sen_xml(server.last_sent_ipoc + 4, rkorr={"X": 1.0})

        for _ in range(3):
            server.process_reply(bad)
        assert server.faulty_packets == 3
        assert server.running is True  # still within budget

        server.process_reply(bad)
        assert server.faulty_packets == 4
        assert server.running is False

    def test_timeout_packets_overrides_rsi_file(self, make_server):
        server = make_server(delay_ms=4, rsi_file=RSI_XML_FILE, timeout_packets=7)
        assert server.timeout_packets == 7

    def test_budget_read_from_rsi_file(self, make_server):
        server = make_server(delay_ms=4, rsi_file=RSI_XML_FILE)
        assert server.timeout_packets == 100

    def test_default_budget(self, server):
        assert server.timeout_packets == 100


# ------------------------------------------------------------------- clock ownership

class TestClockOwnership:
    """The controller owns IPOC; the sensor may never move it."""

    def test_increment_matches_cycle_time(self, make_server):
        server = make_server(delay_ms=12)
        assert server.ipoc_increment == 12
        server.udp_socket = RecordingSocket()

        first = server.ipoc_value
        server.send_once()
        assert server.last_sent_ipoc == first
        assert server.ipoc_value == first + 12

        server.send_once()
        assert server.last_sent_ipoc == first + 12
        assert server.ipoc_value == first + 24

    def test_transmitted_ipoc_matches_last_sent(self, server):
        recorder = RecordingSocket()
        server.udp_socket = recorder
        server.send_once()
        payload = recorder.sent[0][0].decode()
        assert f"<IPOC>{server.last_sent_ipoc}</IPOC>" in payload

    def test_reply_never_overwrites_clock(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        before = server.ipoc_value

        # Valid reply carrying the echoed IPOC ...
        server.process_reply(sen_xml(server.last_sent_ipoc))
        assert server.ipoc_value == before

        # ... and a hostile one claiming a wildly different timestamp.
        server.process_reply(sen_xml(999999999))
        assert server.ipoc_value == before

        server.send_once()
        assert server.ipoc_value == before + server.ipoc_increment


# ------------------------------------------------------------------ Delay reporting

class TestDelayReporting:
    """DEF_Delay streams the late/invalid packet counter back to the sensor."""

    def test_delay_reports_faulty_packet_count(self, server):
        assert isinstance(server.state.get("Delay"), dict)
        initial = server.generate_message()
        assert "<Delay" in initial and 'D="0"' in initial

        server.udp_socket = RecordingSocket()
        server.send_once()
        bad = sen_xml(server.last_sent_ipoc + 4)
        server.process_reply(bad)
        server.process_reply(bad)

        message = server.generate_message()
        assert server.state["Delay"]["D"] == server.faulty_packets == 2
        assert 'D="2"' in message
        assert "<Delay" in message


# --------------------------------------------------------------- absolute mode

class TestAbsoluteMode:
    """Absolute corrections are applied against the start pose, not as world coordinates."""

    def test_absolute_uses_base_pose(self, make_server):
        server = make_server(delay_ms=4, mode="absolute")
        server.base_pose["RIst"]["X"] = 100.0
        server.state["RIst"]["X"] = 100.0

        server.process_reply(sen_xml(None, rkorr={"X": 5.0}))
        assert server.state["RIst"]["X"] == pytest.approx(105.0)

        # Absolute mode is not cumulative: the same correction gives the same pose.
        server.process_reply(sen_xml(None, rkorr={"X": 5.0}))
        assert server.state["RIst"]["X"] == pytest.approx(105.0)

    def test_relative_is_cumulative(self, make_server):
        server = make_server(delay_ms=4, mode="relative")
        start = server.state["RIst"]["X"]
        server.process_reply(sen_xml(None, rkorr={"X": 2.0}))
        server.process_reply(sen_xml(None, rkorr={"X": 2.0}))
        assert server.state["RIst"]["X"] == pytest.approx(start + 4.0)


# --------------------------------------------------------------------------- helpers (Full config)

def sen_xml_generic(ipoc, sentype=SENTYPE, corrections=None, extra=""):
    """Build a <Sen> reply carrying an arbitrary set of correction elements.

    Unlike sen_xml() (RKorr-only), this accepts any correction tag - needed
    for EKorr, which only the Full config declares.
    corrections: {"EKorr": {"E1": 2.5, ...}, ...}
    """
    corrections = corrections or {}
    korr = ""
    for tag, attrs in corrections.items():
        attr_str = " ".join(f'{k}="{v}"' for k, v in attrs.items())
        korr += f"<{tag} {attr_str} />"
    ipoc_element = "" if ipoc is None else f"<IPOC>{ipoc}</IPOC>"
    return f'<Sen Type="{sentype}">{korr}{extra}{ipoc_element}</Sen>'


@pytest.fixture
def full_server(make_server):
    """Echo server driven by RSI_EthernetConfig_Full.xml (declares EKorr/EIPos/HOLDON)."""
    return make_server(config_file=FULL_CONFIG_FILE, delay_ms=4)


# ------------------------------------------------------------------------- EKorr routing

class TestEKorrRouting:
    """EKorr corrections move EIPos (the real KUKA external-axis actual-position
    keyword the AXISCORREXT object applies to) - never ELPos."""

    def test_ekorr_moves_eipos_relative(self, full_server):
        server = full_server
        assert "EIPos" in server.state
        start = server.state["EIPos"]["E1"]

        server.udp_socket = RecordingSocket()
        server.send_once()
        accepted = server.process_reply(
            sen_xml_generic(server.last_sent_ipoc, corrections={"EKorr": {"E1": 2.5}}))

        assert accepted is True
        assert server.faulty_packets == 0
        assert server.state["EIPos"]["E1"] == pytest.approx(start + 2.5)

    def test_ekorr_absolute_mode_uses_base_pose(self, make_server):
        server = make_server(config_file=FULL_CONFIG_FILE, delay_ms=4, mode="absolute")
        server.base_pose["EIPos"]["E1"] = 10.0
        server.state["EIPos"]["E1"] = 10.0

        accepted = server.process_reply(
            sen_xml_generic(None, corrections={"EKorr": {"E1": 2.5}}))
        assert accepted is True
        assert server.state["EIPos"]["E1"] == pytest.approx(12.5)

        # Absolute mode is not cumulative: the same correction gives the same pose.
        server.process_reply(sen_xml_generic(None, corrections={"EKorr": {"E1": 2.5}}))
        assert server.state["EIPos"]["E1"] == pytest.approx(12.5)

    def test_elpos_is_not_part_of_the_config(self, full_server):
        """ELPos isn't declared anywhere in the Full config's SEND section -
        EIPos is the real target, confirming CORRECTION_TO_STATE routes
        EKorr there and nowhere else."""
        assert "ELPos" not in full_server.state


# ------------------------------------------------------------------------- HOLDON emulation

class TestHoldonEmulation:
    """Late/invalid cycles honor each RECEIVE element's HOLDON attribute
    (config_parser.holdon_map), per the RSI manual semantics documented on
    EchoServer._apply_holdon_late_cycle."""

    def test_relative_hold_on_reapplies_last_correction(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        start = server.state["RIst"]["X"]

        server.process_reply(sen_xml(server.last_sent_ipoc, rkorr={"X": 1.0}))
        after_valid = server.state["RIst"]["X"]
        assert after_valid == pytest.approx(start + 1.0)

        server._apply_holdon_late_cycle()
        assert server.state["RIst"]["X"] == pytest.approx(after_valid + 1.0)

    def test_relative_hold_off_adds_nothing(self, server):
        server.udp_socket = RecordingSocket()
        server.send_once()
        server.process_reply(sen_xml(server.last_sent_ipoc, rkorr={"X": 1.0}))
        after_valid = server.state["RIst"]["X"]

        server.holdon_map["RKorr.X"] = 0
        server._apply_holdon_late_cycle()
        assert server.state["RIst"]["X"] == pytest.approx(after_valid)

    def test_absolute_hold_off_returns_to_base(self, make_server):
        server = make_server(delay_ms=4, mode="absolute")
        server.base_pose["RIst"]["X"] = 100.0
        server.state["RIst"]["X"] = 100.0

        server.process_reply(sen_xml(None, rkorr={"X": 5.0}))
        assert server.state["RIst"]["X"] == pytest.approx(105.0)

        server.holdon_map["RKorr.X"] = 0
        server._apply_holdon_late_cycle()
        assert server.state["RIst"]["X"] == pytest.approx(100.0)

    def test_absolute_hold_on_stays_applied(self, make_server):
        server = make_server(delay_ms=4, mode="absolute")
        server.base_pose["RIst"]["X"] = 100.0
        server.state["RIst"]["X"] = 100.0

        server.process_reply(sen_xml(None, rkorr={"X": 5.0}))
        assert server.state["RIst"]["X"] == pytest.approx(105.0)

        server._apply_holdon_late_cycle()  # HOLDON=1 (default): offset stays applied
        assert server.state["RIst"]["X"] == pytest.approx(105.0)

    def test_consecutive_late_budget_caps_reapplication(self, make_server):
        server = make_server(delay_ms=4, timeout_packets=3)
        server.udp_socket = RecordingSocket()
        server.send_once()
        server.process_reply(sen_xml(server.last_sent_ipoc, rkorr={"X": 1.0}))
        after_valid = server.state["RIst"]["X"]
        assert server._consecutive_late == 0

        for _ in range(3):
            server._apply_holdon_late_cycle()
        within_budget = server.state["RIst"]["X"]
        assert within_budget == pytest.approx(after_valid + 3.0)

        # One more late cycle exceeds the timeout_packets budget: the held
        # value stops being re-applied.
        server._apply_holdon_late_cycle()
        assert server.state["RIst"]["X"] == pytest.approx(within_budget)

        # A subsequent valid reply resets the consecutive-late counter.
        server.process_reply(sen_xml(server.last_sent_ipoc, rkorr={"X": 1.0}))
        assert server._consecutive_late == 0


class TestHoldonMap:
    """config_parser.holdon_map: dotted RECEIVE tag -> 0/1, default 1."""

    def test_full_config_declares_holdon_one_on_corrections(self):
        parser = ConfigParser(FULL_CONFIG_FILE)
        assert parser.holdon_map["RKorr.X"] == 1
        assert "EKorr.E1" in parser.holdon_map
        assert parser.holdon_map["EKorr.E1"] == 1

    def test_tech_channels_declare_holdon_zero(self):
        parser = ConfigParser(FULL_CONFIG_FILE)
        assert parser.holdon_map["Tech.T1"] == 0

    def test_defaults_to_one_when_attribute_absent(self):
        parser = ConfigParser(FULL_CONFIG_FILE)
        # DEF_EStr has no HOLDON attribute in the Full config.
        assert parser.holdon_map.get("EStr") == 1


# ------------------------------------------------------------------------- ONLYSEND

class TestOnlySend:
    """ONLYSEND: the controller streams data and expects no reply at all."""

    def test_defaults_to_config_value(self, make_server):
        # RSI_EthernetConfig.xml declares ONLYSEND=FALSE.
        server = make_server()
        assert server.onlysend is False

    def test_constructor_argument_overrides_config(self, make_server):
        server = make_server(onlysend=True)
        assert server.onlysend is True

    def test_receive_and_process_never_reaches_process_reply(self, server, monkeypatch):
        """A reply arriving while ONLYSEND is set is spurious: it must be
        neither applied nor counted as faulty, and process_reply() must not
        even be called."""
        server.onlysend = True
        calls = []
        monkeypatch.setattr(server, "process_reply", lambda xml: calls.append(xml) or True)

        helper = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            helper.sendto(b'<Sen Type="ImFree"><IPOC>1</IPOC></Sen>', ("127.0.0.1", 50000))
            time.sleep(0.05)
            result = server.receive_and_process()
        finally:
            helper.close()

        assert result is None
        assert calls == []
        assert server.faulty_packets == 0
