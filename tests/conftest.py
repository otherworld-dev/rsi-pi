"""Shared pytest configuration and fixtures for the RSIPI test-suite.

Two jobs:

1. Make ``src/`` importable so ``from RSIPI... import ...`` works without the
   package being pip-installed (the existing test modules rely on this).
2. Provide the loopback test-bed used by the safety / lifecycle / trajectory
   tests: a recording EchoServer (controller emulation) plus a connected
   RSIClient, and a factory for building an *unstarted* NetworkProcess so its
   per-cycle stages can be unit-tested without any UDP traffic.

Naming follows the KUKA/robot perspective used throughout the codebase:
    send_variables    = what the robot SENDS to us (RIst, ASPos, IPOC, ...)
    receive_variables = what the robot RECEIVES from us (RKorr, AKorr, DiO, ...)
"""

import multiprocessing
import os
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET

import pytest

# --------------------------------------------------------------------------- paths

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SRC_DIR = os.path.join(REPO_ROOT, "src")

# Front of sys.path: the package is not installed in this environment.
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

CONFIG_FILE = os.path.join(REPO_ROOT, "RSI_EthernetConfig.xml")

# Ports fixed by the config / echo server implementation.
ECHO_SERVER_PORT = 50000  # EchoServer binds 0.0.0.0:50000


def _config_port():
    """UDP port the client binds, as declared in RSI_EthernetConfig.xml."""
    root = ET.parse(CONFIG_FILE).getroot()
    return int(root.find("CONFIG/PORT").text.strip())


CLIENT_PORT = _config_port()


# --------------------------------------------------------------------------- ports

def udp_port_available(port, host="0.0.0.0"):
    """True when a UDP socket can be bound to *port* right now."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def skip_unless_ports_free(*ports):
    """pytest.skip() when any of *ports* is already in use."""
    busy = [p for p in ports if not udp_port_available(p)]
    if busy:
        pytest.skip(
            "UDP port(s) {} in use - loopback RSI test cannot run".format(
                ", ".join(str(p) for p in busy))
        )


@pytest.fixture
def require_ports():
    """Fixture wrapper around :func:`skip_unless_ports_free`."""
    return skip_unless_ports_free


@pytest.fixture
def client_port():
    """UDP port the RSI client binds (from RSI_EthernetConfig.xml)."""
    return CLIENT_PORT


@pytest.fixture
def echo_port():
    """UDP port the echo server binds."""
    return ECHO_SERVER_PORT


# --------------------------------------------------------------------- echo server

from RSIPI.config_parser import ConfigParser  # noqa: E402
from RSIPI.network_handler import NetworkProcess  # noqa: E402
from RSIPI.rsi_client import RSIClient  # noqa: E402
from RSIPI.rsi_echo_server import EchoServer  # noqa: E402


def parse_sen_reply(xml_string):
    """Extract the correction dicts + IPOC carried by one <Sen> reply."""
    parsed = {"RKorr": None, "AKorr": None, "IPOC": None, "raw": xml_string}
    try:
        root = ET.fromstring(xml_string)
    except ET.ParseError:
        return parsed
    for element in root:
        if element.tag in ("RKorr", "AKorr"):
            values = {}
            for axis, value in element.attrib.items():
                try:
                    values[axis] = float(value)
                except (TypeError, ValueError):
                    pass
            parsed[element.tag] = values
        elif element.tag == "IPOC":
            try:
                parsed["IPOC"] = int((element.text or "").strip())
            except ValueError:
                parsed["IPOC"] = None
    return parsed


class RecordingEchoServer(EchoServer):
    """EchoServer that records every reply it *accepts*.

    Validation, faulty-packet accounting and correction application are left
    entirely to the base class - the override only observes, so a rejected
    packet (bad IPOC echo / SENTYPE) is never recorded, exactly like the
    corrections it carries are never applied.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._records = []
        self._records_lock = threading.Lock()
        self._stopped = False

    # -- recording ---------------------------------------------------------

    def process_reply(self, xml_string):
        parsed = parse_sen_reply(xml_string)
        accepted = super().process_reply(xml_string)
        if accepted:
            with self._records_lock:
                self._records.append(parsed)
        return accepted

    def records(self):
        """Snapshot (list copy) of all accepted replies, oldest first."""
        with self._records_lock:
            return list(self._records)

    def clear_records(self):
        with self._records_lock:
            self._records.clear()

    def record_count(self):
        with self._records_lock:
            return len(self._records)

    def axis_values(self, axis="X", corr_key="RKorr", start=0):
        """Wire values of one correction axis across accepted replies."""
        values = []
        for record in self.records()[start:]:
            correction = record.get(corr_key) or {}
            values.append(float(correction.get(axis, 0.0)))
        return values

    def wait_for_records(self, count, timeout=5.0):
        """Block until at least *count* replies have been accepted."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.record_count() >= count:
                return True
            time.sleep(0.005)
        return self.record_count() >= count

    # -- state helpers -----------------------------------------------------

    def seed_state(self, key, values):
        """Seed a robot state variable (and its absolute-mode reference)."""
        self.state[key] = dict(values)
        self.base_pose[key] = dict(values)

    def robot_state(self, key):
        value = self.state.get(key)
        return dict(value) if isinstance(value, dict) else value

    # -- lifecycle ---------------------------------------------------------

    def stop(self):
        """Idempotent stop - fixtures may stop the server more than once."""
        if self._stopped:
            return
        self._stopped = True
        try:
            super().stop()
        except Exception:  # pragma: no cover - teardown must never explode
            pass


# ------------------------------------------------------------------- loopback rig

def _start_client(mode, client_kwargs):
    """Create an RSIClient and run its control loop in a daemon thread."""
    client = RSIClient(CONFIG_FILE, rsi_mode=mode, **client_kwargs)
    thread = threading.Thread(target=client.start, daemon=True)
    # Hand the thread over so client.stop() joins it as well.
    client.thread = thread
    thread.start()
    return client


@pytest.fixture
def rsi_stack():
    """Factory fixture: ``make(...) -> (RecordingEchoServer, RSIClient)``.

    Starts a recording echo server on 0.0.0.0:50000 and an RSIClient bound to
    the config port, then blocks until the first robot packet has been seen.
    Skips the test when either UDP port is occupied.

    Keyword arguments:
        mode:            'relative' (default) or 'absolute' - used for BOTH
                         the client's rsi_mode and the echo server's mode
                         unless server_mode is given.
        server_mode:     override the echo server's correction mode.
        seed_pose:       dict seeding the server's RIst (and its absolute-mode
                         reference pose) before the server starts.
        delay_ms:        echo server interpolation cycle (default 4).
        connect_timeout: seconds to wait for the first packet (default 15).
    Any other keyword argument is forwarded to RSIClient().
    """
    created = []

    def _make(mode="relative", server_mode=None, seed_pose=None, delay_ms=4,
              connect_timeout=15.0, **client_kwargs):
        skip_unless_ports_free(ECHO_SERVER_PORT, CLIENT_PORT)

        client_kwargs.setdefault("cycle_time", 0.004)

        server = RecordingEchoServer(
            CONFIG_FILE, delay_ms=delay_ms, mode=server_mode or mode)
        if seed_pose:
            server.seed_state("RIst", seed_pose)
        server.start()

        client = None
        try:
            client = _start_client(mode, client_kwargs)
        finally:
            created.append((server, client))

        if not client.wait_for_connection(connect_timeout):
            pytest.fail(
                "RSIClient never received a packet from the echo server "
                "(waited {}s)".format(connect_timeout))
        return server, client

    yield _make

    for server, client in reversed(created):
        # Server first: it stops within one 4ms cycle, whereas the network
        # process can sit in a 1s recvfrom timeout.
        try:
            server.stop()
        except Exception:
            pass
        try:
            if client is not None:
                client.stop()
        except Exception:
            pass


# ------------------------------------------------------- network process (no UDP)

@pytest.fixture
def make_network_process():
    """Factory for an *unstarted* NetworkProcess usable for unit tests.

    The process object is never start()ed: only its pure per-cycle stages
    (_enforce_limits / _apply_rate_limit / _handle_estop_transition) are
    exercised, against plain dicts instead of Manager proxies.
    """
    created = []

    def _make(rsi_mode="relative", max_cartesian_rate=0.0, max_joint_rate=0.0,
              send_variables=None, receive_variables=None, limits=None,
              override=False, config_parser=None, port=0):
        parser = config_parser or ConfigParser(CONFIG_FILE)
        process = NetworkProcess(
            "127.0.0.1",
            port,
            dict(parser.send_variables) if send_variables is None else send_variables,
            dict(parser.receive_variables) if receive_variables is None else receive_variables,
            multiprocessing.Event(),
            parser,
            multiprocessing.Event(),
            multiprocessing.Queue(),
            rsi_mode=rsi_mode,
            max_cartesian_rate=max_cartesian_rate,
            max_joint_rate=max_joint_rate,
        )
        if limits is not None:
            process.safety_manager.limits = dict(limits)
        process.safety_manager.override = override
        process._build_clamp_table()
        created.append(process)
        return process

    yield _make

    for process in created:
        try:
            process.command_queue.close()
        except Exception:
            pass


# ------------------------------------------------------------------- misc fixtures

@pytest.fixture
def repo_root():
    return REPO_ROOT


@pytest.fixture
def src_dir():
    return SRC_DIR


@pytest.fixture
def config_file():
    return CONFIG_FILE


def wait_until(predicate, timeout=5.0, interval=0.005):
    """Poll *predicate* until truthy; returns the final truthiness."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


@pytest.fixture
def poll_until():
    """Fixture wrapper around :func:`wait_until`."""
    return wait_until
