"""The client's shared state in shared memory, and the snapshot budget that put it there.

The network process reads every value it sends on every cycle, inside a 4 ms
reply window. From a multiprocessing.Manager dict that read was one pipe round
trip per key (1.8 ms median, 4.3 ms worst case on the drilling laptop), and on
the KR C4 about 10 % of replies were late while a trajectory ran, each one a
feed step the robot never applied. The robot's state went back through the
same Manager, so the client only saw every tenth cycle.

TestSnapshotBudget times the per-cycle read against a publisher running at the
trajectory executor's rate and fails if IPC comes back into it. The rest pin
down what the store has to keep true for the rest of RSIPI: the dict
interface the API namespaces use, a (seq, corrections) pair the network
process sees whole, readers that always find the newest complete frame
whatever a writer is doing, and a network process that does not outlive the
client.
"""

import ctypes
import multiprocessing
import threading
import time
import xml.etree.ElementTree as ET

import pytest

from RSIPI.diagnostics_api import DiagnosticsAPI
from RSIPI.motion_api import MotionAPI
from RSIPI.shared_variables import SharedVariables
from RSIPI.timing_metrics import SNAPSHOT_BUDGET

from tests.conftest import CONFIG_FILE, udp_port_available, wait_until

# The Drill context's RECEIVE variables.
DRILL_RECEIVE = {
    "EStr": "", "Tech": {"T2": 0.0},
    "RKorr": {a: 0.0 for a in "XYZABC"}, "FREE": 0, "DiO": 0,
    "AKorr": {f"A{i}": 0.0 for i in range(1, 7)},
    "SenP1": 0.0, "SenP2": 0.0, "SenP3": 0.0, "AnOut1": 0.0,
    "SenPIntW": 0, "OvProW": 100,
}


def _publish_every(store, period, stop):
    """Child process: publish RKorr.X = its own seq, like a trajectory executor."""
    expected = store.seq
    next_at = time.perf_counter()
    while not stop.is_set():
        expected += 1
        assert store.publish({"RKorr": {"X": float(expected)}}) == expected
        if period:
            next_at += period
            gap = next_at - time.perf_counter()
            if gap > 0:
                time.sleep(gap)


@pytest.fixture
def publisher():
    """Factory: start a publishing child process; stopped at teardown."""
    running = []

    def _start(store, period):
        stop = multiprocessing.Event()
        process = multiprocessing.Process(
            target=_publish_every, args=(store, period, stop), daemon=True)
        process.start()
        running.append((process, stop))
        assert wait_until(lambda: store.seq > 0, timeout=20), \
            "publisher process never published"
        return process

    yield _start

    for process, stop in running:
        stop.set()
        process.join(timeout=5)
        if process.is_alive():
            process.terminate()


# ===========================================================================
# The regression: the per-cycle snapshot stays inside its budget
# ===========================================================================
class TestSnapshotBudget:

    def test_snapshot_stays_in_budget_while_a_trajectory_publishes(
            self, make_network_process, publisher):
        store = SharedVariables(DRILL_RECEIVE)
        process = make_network_process(receive_variables=store)
        publisher(store, period=0.004)

        def one_round(cycles=1000):
            took = []
            next_at = time.perf_counter()
            for _ in range(cycles):
                next_at += 0.002
                while time.perf_counter() < next_at:
                    time.sleep(0.0005)
                started = time.perf_counter()
                seq, variables = process._snapshot_corrections()
                took.append(time.perf_counter() - started)
                assert variables["RKorr"]["X"] == float(seq)
            took.sort()
            return took[cycles // 2], took[int(cycles * 0.99)], took[-1]

        # Best of three: a busy machine can push one round's p99 over, but a
        # snapshot that talks to another process is over in every round (one
        # Manager round trip alone has a 0.3 ms median).
        reports = []
        for _ in range(3):
            p50, p99, worst = one_round()
            reports.append("p50 %.3f ms, p99 %.3f ms, max %.3f ms" % (
                p50 * 1e3, p99 * 1e3, worst * 1e3))
            if p99 < SNAPSHOT_BUDGET:
                break
        print("snapshot " + reports[-1])
        assert p99 < SNAPSHOT_BUDGET, (
            "per-cycle snapshot over its %.2f ms budget in every round (%s) - it "
            "runs inside each 4 ms reply window and must not talk to another "
            "process" % (SNAPSHOT_BUDGET * 1e3, "; ".join(reports)))

    def test_client_shares_nothing_through_a_manager(self, rsi_stack):
        _server, client = rsi_stack(mode="relative")
        for name in ("send_variables", "receive_variables", "metrics_dict"):
            assert isinstance(getattr(client, name), SharedVariables), name
            assert isinstance(getattr(client.network_process, name), SharedVariables), name
        assert not hasattr(client, "manager")

    def test_robot_state_reaches_the_client_every_cycle(self, rsi_stack):
        """Through the Manager it was passed on every tenth cycle, so the
        client's view of the robot was up to 40 ms old."""
        server, client = rsi_stack(mode="relative")
        seen = []
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            ipoc = client.send_variables["IPOC"]
            if not seen or ipoc != seen[-1]:
                seen.append(ipoc)
            # The read no longer blocks on anything, so give the echo
            # server's thread (this same process) the GIL.
            time.sleep(0.001)
        steps = sorted(b - a for a, b in zip(seen, seen[1:]))
        assert len(seen) > 100, "only %d distinct IPOCs seen in 1 s" % len(seen)
        assert steps[0] == server.ipoc_increment
        assert steps[len(steps) // 2] == server.ipoc_increment, \
            "the usual step between the states seen was %r IPOC" % steps[len(steps) // 2]

    def test_get_stats_reports_the_snapshot_p99(self, rsi_stack):
        _server, client = rsi_stack(mode="relative")
        motion = MotionAPI(client)
        motion.execute_trajectory(
            [{"X": 0.1} for _ in range(150)], space="cartesian", points="delta")

        diagnostics = DiagnosticsAPI(client)
        assert wait_until(
            lambda: diagnostics.get_stats().get("total_cycles", 0) >= 200, timeout=10)
        stats = diagnostics.get_stats()

        assert 0.0 < stats["snapshot_p99"] < SNAPSHOT_BUDGET, stats
        assert stats["snapshot_p50"] <= stats["snapshot_p99"] <= stats["snapshot_max"]
        assert "snapshots_over_budget" in stats and "stale_snapshots" in stats
        assert "Snapshot: p99" in diagnostics.format_stats()


# ===========================================================================
# (seq, corrections) is one publication
# ===========================================================================
class TestAtomicPair:

    def test_publish_merges_and_bumps_seq(self):
        store = SharedVariables(DRILL_RECEIVE)
        assert store.snapshot()[0] == 0

        assert store.publish({"RKorr": {"X": 1.5}}) == 1
        assert store.publish({"RKorr": {"Z": -2.0}, "AKorr": {"A3": 0.1}}) == 2

        seq, variables = store.snapshot()
        assert seq == 2
        assert variables["RKorr"] == {"X": 1.5, "Y": 0.0, "Z": -2.0,
                                      "A": 0.0, "B": 0.0, "C": 0.0}
        assert variables["AKorr"]["A3"] == 0.1
        assert variables["OvProW"] == 100

    def test_plain_writes_do_not_bump_seq(self):
        store = SharedVariables(DRILL_RECEIVE)
        store["DiO"] = 4
        store.update({"OvProW": 50, "EStr": "drilling"})
        seq, variables = store.snapshot()
        assert seq == 0
        assert (variables["DiO"], variables["OvProW"], variables["EStr"]) == (4, 50, "drilling")

    def test_seq_carries_over_to_a_new_store(self):
        old = SharedVariables(DRILL_RECEIVE)
        old.publish({"RKorr": {"X": 1.0}})
        old.publish({"RKorr": {"X": 1.0}})
        new = SharedVariables(DRILL_RECEIVE, seq=old.seq)
        assert new.snapshot() == (2, DRILL_RECEIVE)
        assert new.publish({"RKorr": {"X": 1.0}}) == 3

    def test_pair_is_never_torn_across_processes(self, publisher):
        """The publisher writes X = the seq that publish will get. A reader
        that ever sees them differ has paired a seq with another waypoint's
        corrections, which is how a waypoint gets acked without being sent."""
        store = SharedVariables(DRILL_RECEIVE)
        publisher(store, period=0)

        seen = set()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            seq, variables = store.snapshot()
            assert variables["RKorr"]["X"] == float(seq)
            seen.add(seq)
        assert len(seen) > 100, "reader saw only %d distinct publications" % len(seen)


# ===========================================================================
# The reader never waits
# ===========================================================================
def _slot_address(store, slot):
    return ctypes.addressof(store._buf) + slot * store._capacity


class TestReaderNeverWaits:

    def test_reader_gets_the_newest_frame_while_a_writer_is_frozen_mid_commit(self):
        store = SharedVariables(DRILL_RECEIVE)
        store.publish({"RKorr": {"X": 1.0}})

        # A writer frozen mid-commit: lock held, the idle slot half written.
        assert store._wlock.acquire(timeout=1)
        idle = (store._hdr[0] + 1) & 1
        ctypes.memmove(_slot_address(store, idle), b"\xff" * 64, 64)
        try:
            # Even a reader with nothing cached, as in a process just started.
            store._cache = (-1, b"")
            started = time.perf_counter()
            seq, variables = store.snapshot()
            assert time.perf_counter() - started < 0.05
            assert (seq, variables["RKorr"]["X"]) == (1, 1.0)
            assert store.stale_snapshots == 0
        finally:
            store._wlock.release()

        store.publish({"RKorr": {"X": 2.0}})
        assert store.snapshot()[1]["RKorr"]["X"] == 2.0

    def test_interrupted_commit_changes_nothing(self, monkeypatch):
        import RSIPI.shared_variables as module
        store = SharedVariables(DRILL_RECEIVE)
        store.publish({"RKorr": {"X": 1.0}})
        before = store.snapshot()

        def interrupted(_frame):
            raise KeyboardInterrupt
        monkeypatch.setattr(module.zlib, "crc32", interrupted)
        with pytest.raises(KeyboardInterrupt):
            store["DiO"] = 9
        monkeypatch.undo()

        store._cache = (-1, b"")
        assert store.snapshot() == before
        store["DiO"] = 7            # the lock was released, so the next write lands
        assert store["DiO"] == 7 and store.seq == 1

    def test_replace_without_blocking_skips_when_a_writer_holds_the_lock(self):
        store = SharedVariables({"RIst": {"X": 0.0}, "IPOC": 0}, seq=5)
        assert store.replace({"RIst": {"X": 1.0}, "IPOC": 4}, block=False) is True
        assert store.snapshot() == (5, {"RIst": {"X": 1.0}, "IPOC": 4})

        assert store._wlock.acquire(timeout=1)
        try:
            started = time.perf_counter()
            assert store.replace({"IPOC": 8}, block=False) is False
            assert time.perf_counter() - started < 0.05
        finally:
            store._wlock.release()
        assert store["IPOC"] == 4

    def test_writer_gives_up_instead_of_hanging(self):
        store = SharedVariables(DRILL_RECEIVE)
        store.write_timeout = 0.05
        assert store._wlock.acquire(timeout=1)
        try:
            with pytest.raises(TimeoutError):
                store["DiO"] = 1
        finally:
            store._wlock.release()
        store["DiO"] = 1
        assert store["DiO"] == 1

    def test_corrupt_publication_is_refused_and_the_next_write_repairs_it(self):
        """Nothing RSIPI does produces this. If memory is ever corrupted, a
        reader must keep the last good frame rather than send garbage."""
        store = SharedVariables(DRILL_RECEIVE)
        store.publish({"RKorr": {"X": 1.0}})
        good = store.snapshot()

        # A publication whose bytes do not match its CRC.
        gen = store._hdr[0] + 1
        ctypes.memmove(_slot_address(store, gen & 1), b"\xff" * 64, 64)
        store._hdr[1 + 2 * (gen & 1)] = 64
        store._hdr[0] = gen

        assert store.snapshot() == good
        assert store.stale_snapshots == 1

        store["DiO"] = 7
        seq, variables = store.snapshot()
        assert seq == 1
        assert variables["DiO"] == 7 and variables["RKorr"]["X"] == 1.0

    def test_oversized_value_is_refused_and_nothing_changes(self):
        store = SharedVariables(DRILL_RECEIVE)
        with pytest.raises(ValueError, match="too large"):
            store["EStr"] = "x" * (store._capacity + 1)
        assert store.snapshot() == (0, DRILL_RECEIVE)


# ===========================================================================
# The dict interface the API namespaces rely on
# ===========================================================================
class TestMappingInterface:

    def test_reads(self):
        store = SharedVariables(DRILL_RECEIVE)
        assert "RKorr" in store and "Nope" not in store
        assert store["OvProW"] == 100
        assert store.get("Nope") is None and store.get("Nope", 5) == 5
        assert len(store) == len(DRILL_RECEIVE)
        assert list(store) == list(DRILL_RECEIVE)
        assert dict(store) == DRILL_RECEIVE
        assert dict(store.items()) == DRILL_RECEIVE
        assert store.copy() == DRILL_RECEIVE and store == DRILL_RECEIVE
        with pytest.raises(KeyError):
            store["Nope"]

    def test_values_read_out_are_copies(self):
        store = SharedVariables(DRILL_RECEIVE)
        store["RKorr"]["X"] = 9.0
        store.snapshot()[1]["RKorr"]["X"] = 9.0
        assert store["RKorr"]["X"] == 0.0

    def test_writes(self):
        store = SharedVariables(DRILL_RECEIVE)
        store["RKorr"] = {**store["RKorr"], "Y": 3.0}
        assert store["RKorr"]["Y"] == 3.0
        del store["FREE"]
        assert "FREE" not in store
        assert store.pop("DiO") == 0 and "DiO" not in store
        assert store.setdefault("New", 1) == 1 and store["New"] == 1

    def test_initial_contents_are_not_aliased(self):
        initial = {"RKorr": {"X": 0.0}}
        store = SharedVariables(initial)
        initial["RKorr"]["X"] = 5.0
        assert store["RKorr"]["X"] == 0.0

    def test_empty_store_fills_by_replace(self):
        """How the metrics are shared: empty until the first publication."""
        store = SharedVariables()
        assert dict(store) == {} and store.get("total_cycles", 0) == 0
        store.replace({"total_cycles": 100, "warnings": ["High jitter"]})
        assert dict(store) == {"total_cycles": 100, "warnings": ["High jitter"]}


# ===========================================================================
# The network process does not outlive the client
# ===========================================================================
def _client_that_never_stops(config_file, started):
    from RSIPI.rsi_client import RSIClient
    client = RSIClient(config_file)
    threading.Thread(target=client.start, daemon=True).start()
    started.set()
    time.sleep(120)


class TestOrphanedNetworkProcess:

    def test_parent_gone_is_only_reported_for_a_dead_parent(self, make_network_process):
        process = make_network_process()

        class Parent:
            def __init__(self, alive):
                self.alive = alive

            def is_alive(self):
                return self.alive

        assert process._parent_gone(None) is False        # run in the main process
        assert process._parent_gone(Parent(True)) is False
        assert process._parent_gone(Parent(False)) is True

    def test_network_process_stops_when_its_client_is_killed(self, tmp_path):
        """Killed outright, the client cannot stop its network process, which
        would go on answering the robot with the last corrections. A broken
        Manager pipe used to end it, some of the time; with no Manager it
        watches its parent instead. It holds the client port while it lives."""
        # Its own loopback address, never the config's. On the lab PC the
        # config's 10.10.10.10:64000 is the robot's socket, and the wildcard
        # probe does not see a bind to that one address anyway.
        host, port = "127.0.0.1", 59491
        tree = ET.parse(CONFIG_FILE)
        tree.getroot().find("CONFIG/IP_NUMBER").text = host
        tree.getroot().find("CONFIG/PORT").text = str(port)
        config = tmp_path / "RSI_EthernetConfig_loopback.xml"
        tree.write(config)
        if not udp_port_available(port, host):
            pytest.skip("UDP %s:%d is in use" % (host, port))

        started = multiprocessing.Event()
        parent = multiprocessing.Process(
            target=_client_that_never_stops, args=(str(config), started))
        parent.start()
        try:
            assert started.wait(30)
            assert wait_until(lambda: not udp_port_available(port, host), timeout=30), \
                "the network process never bound the client port"
            parent.kill()
            parent.join(10)
            assert wait_until(lambda: udp_port_available(port, host), timeout=10), \
                "the network process outlived its killed parent"
        finally:
            if parent.is_alive():
                parent.kill()
