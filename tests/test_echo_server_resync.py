"""Both ends of the link recover from a stall instead of locking one packet behind.

The regression: each cycle the echo server read ONE datagram, sent the next
packet and slept. A reply that missed its window stayed queued, so the next
cycle read that stale reply (rejected), the fresh reply queued behind it, and
every later cycle read a reply exactly one packet old. Every packet then
counted as faulty and the Timeout budget was gone in under a second - from one
late reply.

The server now reads the whole cycle's worth of datagrams in order, drops
replies to earlier packets, and accepts the current reply in the same cycle.
A cycle left unanswered counts once, when the client is next heard from.

The client had the mirror-image problem: after a stall it answered every
queued robot packet in turn, and the first of those doomed replies carried
(and acked) any correction published meanwhile, which was then lost without
an error. It now answers only the newest packet.

Layers:
  * receive_and_process() and NetworkProcess._newest_packet() against real
    sockets, with datagrams queued by hand;
  * a scripted sensor that sends exactly one reply late, over loopback;
  * RSIPI's own client, frozen mid-stream the way a GC pause or scheduler
    stall would freeze it - plus an opt-in soak (RSIPI_SOAK_SECONDS) for the
    minutes-long runs where the lock-step showed up.
"""
import os
import re
import socket
import threading
import time

import pytest

from RSIPI.fault_injection import suspended
from RSIPI.rsi_echo_server import EchoServer
from tests.conftest import (
    CLIENT_PORT,
    CONFIG_FILE,
    ECHO_SERVER_PORT,
    RecordingEchoServer,
    skip_unless_ports_free,
    wait_until,
)

SENTYPE = "ImFree"  # matches <SENTYPE> in RSI_EthernetConfig.xml

IPOC_PATTERN = re.compile(rb"<IPOC>(\d+)</IPOC>")

# Soak: duration from the environment; stalls spaced so that even a long soak
# injects at most MAX_SOAK_STALLS. The break-off budget is a running total, so
# the stall count, not the duration, is what has to stay inside it.
SOAK_SECONDS = float(os.environ.get("RSIPI_SOAK_SECONDS") or 0)
MAX_SOAK_STALLS = 20
MIN_SOAK_STALL_INTERVAL = 10.0

# Accepted replies per second below which streaming counts as stalled. The
# nominal rate is 250/s; the server's Python loop on Windows manages ~200.
MIN_ACCEPTED_PER_SECOND = 100


# --------------------------------------------------------------------------- helpers

def sen(ipoc, x=0.0):
    """A <Sen> reply echoing *ipoc*, carrying an RKorr X correction."""
    return (f'<Sen Type="{SENTYPE}"><RKorr X="{x}" Y="0" Z="0" A="0" B="0" C="0" />'
            f"<IPOC>{ipoc}</IPOC></Sen>")


class NoSendSocket:
    """The server's real socket with transmission switched off.

    Receiving stays real. Sending is dropped so no datagram goes to an unbound
    client port - on Windows the ICMP reply to that surfaces as a
    ConnectionResetError on the next recvfrom.
    """

    def __init__(self, sock):
        self._sock = sock

    def sendto(self, data, address):
        return len(data)

    def __getattr__(self, name):
        return getattr(self._sock, name)


@pytest.fixture
def server():
    """Echo server on 0.0.0.0:50000 whose clock advances without transmitting."""
    skip_unless_ports_free(ECHO_SERVER_PORT)
    echo = EchoServer(CONFIG_FILE, delay_ms=4)
    real_socket = echo.udp_socket
    echo.udp_socket = NoSendSocket(real_socket)
    yield echo
    echo.running = False
    real_socket.close()


@pytest.fixture
def feed():
    """feed(*xml): queue datagrams on the server's socket, in order."""
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _feed(*payloads):
        for payload in payloads:
            sender.sendto(payload.encode(), ("127.0.0.1", ECHO_SERVER_PORT))
        time.sleep(0.02)  # let loopback deliver before the server reads

    yield _feed
    sender.close()


def stall_client_past_one_window(server, client):
    """Freeze RSIPI's network process until the server has given up on a packet.

    The server only sends its next packet once the previous one's window has
    closed (reply accepted, or timed out). Holding the client until the server
    has sent two packets past the freeze guarantees that at least the first
    of them went unanswered in time - a stall longer than two cycles by
    construction, however coarse the host's timers are.

    Returns (packets the server sent during the stall, stall seconds).
    """
    increment = server.ipoc_increment
    started = time.monotonic()
    with suspended(client.network_process.pid):
        first_unanswered = server.ipoc_value
        moved_on = wait_until(
            lambda: server.ipoc_value >= first_unanswered + 2 * increment,
            timeout=2.0, interval=0.0005)
        sent = (server.ipoc_value - first_unanswered) // increment
    stall = time.monotonic() - started
    assert moved_on, "server stopped transmitting while the client was frozen"
    return sent, stall


# ------------------------------------------------------------- receive_and_process

def attach(server, feed):
    """One answered cycle: from here on the server knows a client is there."""
    server.send_once()
    feed(sen(server.last_sent_ipoc))
    assert server.receive_and_process() is True


def miss_cycle(server):
    """Send a packet and let its window close with nothing received."""
    server.send_once()
    assert server.receive_and_process() is None
    return server.last_sent_ipoc


class TestLateReplyInTheQueue:
    """A late reply is dropped without hiding the current one, and its cycle
    counts once - when the client is next heard from."""

    def test_current_reply_behind_a_late_one_is_accepted(self, server, feed, capsys):
        attach(server, feed)
        late_ipoc = miss_cycle(server)
        assert server.faulty_packets == 0  # not yet: the client may be gone
        server.send_once()
        start_x = server.state["RIst"]["X"]

        feed(sen(late_ipoc, x=5.0), sen(server.last_sent_ipoc, x=1.0))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 1
        # Only the current reply's correction is applied; the late one is dropped.
        assert server.state["RIst"]["X"] == pytest.approx(start_x + 1.0)
        assert "no reply in time for 1 cycle(s)" in capsys.readouterr().out

    def test_a_backlog_of_late_replies_is_not_counted_twice(self, server, feed):
        attach(server, feed)
        backlog = [miss_cycle(server) for _ in range(3)]
        server.send_once()

        feed(*(sen(ipoc) for ipoc in backlog), sen(server.last_sent_ipoc))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 3

    def test_client_that_skips_the_backlog_still_costs_the_missed_cycles(self, server, feed):
        """RSIPI's client answers only the newest packet after a stall."""
        attach(server, feed)
        for _ in range(3):
            miss_cycle(server)
        server.send_once()

        feed(sen(server.last_sent_ipoc))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 3

    def test_late_reply_alone_then_current_reply_next_cycle(self, server, feed):
        attach(server, feed)
        late_ipoc = miss_cycle(server)
        server.send_once()
        start_x = server.state["RIst"]["X"]

        # Only the late reply arrives: the missed cycle counts, this one is
        # unanswered in turn, and the late correction is not applied.
        feed(sen(late_ipoc, x=5.0))
        assert server.receive_and_process() is None
        assert server.faulty_packets == 1
        assert server.state["RIst"]["X"] == pytest.approx(start_x)

        server.send_once()
        feed(sen(server.last_sent_ipoc, x=1.0))
        assert server.receive_and_process() is True
        assert server.faulty_packets == 2

    def test_consistently_late_client_breaks_off(self, server, feed):
        """Every reply one cycle late is every cycle lost, as on a controller."""
        server.timeout_packets = 10
        attach(server, feed)
        previous = server.last_sent_ipoc
        for _ in range(12):
            server.send_once()
            feed(sen(previous))
            previous = server.last_sent_ipoc
            server.receive_and_process()
            if not server.running:
                break
        assert server.running is False
        assert server.faulty_packets == 11

    def test_silence_before_attach_is_never_counted(self, server, feed):
        """The documented divergence: a server with no client yet stays up."""
        for _ in range(3):
            miss_cycle(server)
        server.send_once()
        feed(sen(server.last_sent_ipoc))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 0

    def test_silence_longer_than_timeout_is_a_reconnect(self, server, feed, capsys):
        """A controller would have broken off; this server lets the client restart."""
        server.timeout_packets = 2  # tolerates 8 ms of silence
        attach(server, feed)
        miss_cycle(server)
        time.sleep(0.05)
        server.send_once()
        feed(sen(server.last_sent_ipoc))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 0
        assert server.running is True
        assert "treated as a reconnect" in capsys.readouterr().out

    def test_invalid_packet_does_not_end_the_cycle(self, server, feed):
        server.send_once()
        feed(sen(server.last_sent_ipoc).replace(SENTYPE, "NotMe"),
             sen(server.last_sent_ipoc))

        assert server.receive_and_process() is True
        assert server.faulty_packets == 1

    def test_made_up_ipoc_is_a_mismatch_not_a_late_reply(self, server, feed, capsys):
        server.send_once()
        server.send_once()
        # Lower than the last packet but never sent: not on the IPOC grid,
        # and before the first packet respectively.
        feed(sen(server.last_sent_ipoc - 1), sen(server.first_ipoc - 4))

        assert server.receive_and_process() is False
        assert server.faulty_packets == 2
        assert capsys.readouterr().out.count("IPOC mismatch") == 2

    def test_break_off_stops_reading(self, server, feed):
        server.timeout_packets = 1
        server.send_once()
        start_x = server.state["RIst"]["X"]
        bad = sen(server.last_sent_ipoc).replace(SENTYPE, "NotMe")

        feed(bad, bad, bad, sen(server.last_sent_ipoc, x=1.0))

        assert server.receive_and_process() is False
        assert server.running is False
        assert server.faulty_packets == 2
        assert server.state["RIst"]["X"] == pytest.approx(start_x)


# ------------------------------------------------------------- client read-ahead

@pytest.fixture
def queued_process(make_network_process):
    """An unstarted NetworkProcess with a real socket, plus a robot-side sender."""
    process = make_network_process()
    process.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    process.udp_socket.bind(("127.0.0.1", 0))
    process.udp_socket.settimeout(1.0)
    robot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def queue(*ipocs):
        for ipoc in ipocs:
            robot.sendto(f"<Rob><IPOC>{ipoc}</IPOC></Rob>".encode(),
                         process.udp_socket.getsockname())
        time.sleep(0.02)

    yield process, queue
    robot.close()
    process.udp_socket.close()


class TestClientAnswersTheNewestPacket:

    def test_backlog_is_skipped_to_the_newest(self, queued_process):
        process, queue = queued_process
        queue(100, 104, 108)
        first, _ = process.udp_socket.recvfrom(65535)

        newest = process._newest_packet(first)

        assert b"<IPOC>108</IPOC>" in newest
        assert process._skipped_packets == 2
        assert process.controller_ip_and_port is not None

    def test_nothing_queued_changes_nothing(self, queued_process):
        process, queue = queued_process
        queue(100)
        first, _ = process.udp_socket.recvfrom(65535)

        assert process._newest_packet(first) == first
        assert process._skipped_packets == 0


# ------------------------------------------------------------- scripted sensor

class ScriptedSensor(threading.Thread):
    """A minimal sensor: answers every robot packet by echoing its IPOC.

    hold_one_reply() makes it sit on the reply to the next packet until the
    packet after it arrives - proof the server has given up on it - and then
    send that reply late, just ahead of the current one. Exactly one late
    reply, whatever the host's timer resolution.
    """

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.stop_event = threading.Event()
        self.late_reply_sent = threading.Event()
        self._hold = threading.Event()

    def hold_one_reply(self):
        self._hold.set()

    def run(self):
        held = None
        while not self.stop_event.is_set():
            try:
                data, address = self.sock.recvfrom(65535)
            except OSError:  # timeout, or a reset before the server bound
                continue
            match = IPOC_PATTERN.search(data)
            if not match:
                continue
            reply = sen(int(match.group(1))).encode()
            if held is not None:
                self.sock.sendto(held, address)
                held = None
                self.late_reply_sent.set()
            if self._hold.is_set():
                self._hold.clear()
                held = reply
                continue
            self.sock.sendto(reply, address)

    def stop(self):
        self.stop_event.set()
        self.join(timeout=2)
        self.sock.close()


@pytest.fixture
def scripted_link():
    skip_unless_ports_free(ECHO_SERVER_PORT, CLIENT_PORT)
    # Sensor first: a server transmitting to an unbound port gets a reset
    # on Windows and backs off for half a second.
    sensor = ScriptedSensor(CLIENT_PORT)
    sensor.start()
    echo = RecordingEchoServer(CONFIG_FILE, delay_ms=4)
    echo.start()
    yield echo, sensor
    echo.stop()
    sensor.stop()


class TestOneLateReplyOverLoopback:

    def test_one_late_reply_costs_one_packet(self, scripted_link):
        server, sensor = scripted_link
        assert server.wait_for_records(100, timeout=5)
        assert server.faulty_packets == 0

        sensor.hold_one_reply()
        assert sensor.late_reply_sent.wait(timeout=2)
        accepted_after_late = server.record_count()

        # Long enough for the old lock-step to exhaust Timeout=100.
        time.sleep(1.0)

        assert server.running is True
        assert server.faulty_packets == 1
        assert server.record_count() - accepted_after_late >= MIN_ACCEPTED_PER_SECOND


# ------------------------------------------------------------- RSIPI's own client

class TestRsipiClientStall:

    def test_client_stall_costs_about_one_packet(self, rsi_stack):
        server, client = rsi_stack(mode="relative")
        assert server.wait_for_records(250, timeout=10)
        before = server.faulty_packets

        sent, stall = stall_client_past_one_window(server, client)

        # Each packet whose window closed during the freeze counts once: at
        # least one, never more than were sent while the client was frozen
        # (plus one it may have been holding when it froze).
        assert wait_until(lambda: server.faulty_packets > before, timeout=1.0)
        time.sleep(0.25)
        rise = server.faulty_packets - before
        assert 1 <= rise <= sent + 1, (
            f"rise={rise}, packets sent during the {stall * 1000:.1f} ms stall={sent}")

        # Resynchronised: streaming continues and the count stops climbing.
        settled = server.faulty_packets
        accepted = server.record_count()
        time.sleep(1.0)
        assert server.running is True
        assert server.faulty_packets == settled
        assert server.record_count() - accepted >= MIN_ACCEPTED_PER_SECOND

        # The client passed over the packets it could no longer answer in time.
        assert client.metrics_dict.get("skipped_packets", 0) >= 1

    def test_correction_published_during_a_stall_is_applied(self, rsi_stack):
        """The exactly-once delta must ride on a reply the controller accepts.

        Answering the backlog in order put it on the reply to the oldest
        queued packet - certain to be rejected - and acked it anyway, so the
        step was lost without an error.
        """
        server, client = rsi_stack(mode="relative")
        assert server.wait_for_records(250, timeout=10)
        client._oneshot_active.value = True
        mark = server.record_count()

        published = {}
        with suspended(client.network_process.pid):
            first_unanswered = server.ipoc_value
            # Published from a thread: if the freeze caught the network
            # process holding the correction lock, this waits for the resume.
            publisher = threading.Thread(target=lambda: published.update(
                seq=client.publish_corrections({"RKorr": {"X": 0.5}})))
            publisher.start()
            assert wait_until(
                lambda: server.ipoc_value >= first_unanswered + 2 * server.ipoc_increment,
                timeout=2.0, interval=0.0005)
        publisher.join(timeout=5)

        assert client.wait_correction_applied(published["seq"], timeout=5.0)
        time.sleep(0.25)
        applied = sum(server.axis_values("X", start=mark))
        # Zero at the source before releasing the latch, as _execute_deltas does.
        client.receive_variables["RKorr"] = {
            axis: 0.0 for axis in client.receive_variables["RKorr"]}
        client._oneshot_active.value = False
        assert applied == pytest.approx(0.5), (
            f"controller accepted {applied} mm of a 0.5 mm step")


@pytest.mark.soak
@pytest.mark.skipif(SOAK_SECONDS <= 0,
                    reason="soak: set RSIPI_SOAK_SECONDS (e.g. 600) to run")
def test_soak_client_with_periodic_stalls(rsi_stack):
    """Minutes of streaming with RSIPI's client, frozen past a window at intervals.

    Every second: the server is still running and still accepting replies.
    Every stall: the faulty count rises by at least one and at most the
    packets sent while frozen. At the end: the total is inside the budget.
    """
    server, client = rsi_stack(mode="relative")
    assert server.wait_for_records(250, timeout=10)
    stall_interval = max(MIN_SOAK_STALL_INTERVAL, SOAK_SECONDS / MAX_SOAK_STALLS)

    started = time.monotonic()
    next_stall = started + stall_interval
    stalls = []  # (packets sent while frozen, faulty rise)
    accepted_total = 0
    slowest_second = None
    server.clear_records()

    while time.monotonic() - started < SOAK_SECONDS:
        second_started = time.monotonic()
        stalled_this_second = False
        if second_started >= next_stall:
            before = server.faulty_packets
            sent, stall = stall_client_past_one_window(server, client)
            time.sleep(0.25)
            rise = server.faulty_packets - before
            assert 1 <= rise <= sent + 1, (
                f"stall {len(stalls) + 1}: rise={rise}, sent={sent}, "
                f"{stall * 1000:.1f} ms")
            stalls.append((sent, rise))
            next_stall += stall_interval
            stalled_this_second = True

        time.sleep(max(0.0, 1.0 - (time.monotonic() - second_started)))
        accepted = server.record_count()
        server.clear_records()  # keeps memory flat over a long run
        accepted_total += accepted

        elapsed = time.monotonic() - started
        assert server.running is True, (
            f"RSI broke off after {elapsed:.0f} s with {server.faulty_packets} faulty packets")
        if not stalled_this_second:
            assert accepted >= MIN_ACCEPTED_PER_SECOND, (
                f"only {accepted} replies accepted in second {elapsed:.0f}")
            slowest_second = accepted if slowest_second is None else min(slowest_second, accepted)

    stall_faults = sum(rise for _, rise in stalls)
    print(f"\nsoak: {SOAK_SECONDS:.0f} s, {accepted_total} replies accepted, "
          f"slowest second {slowest_second}, {len(stalls)} stalls costing "
          f"{stall_faults}, {server.faulty_packets - stall_faults} other faulty packets, "
          f"total {server.faulty_packets}/{server.timeout_packets}")
    assert stalls, "soak too short to inject a stall"
    assert server.faulty_packets <= server.timeout_packets
