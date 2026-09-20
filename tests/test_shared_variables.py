"""receive_variables in shared memory, and the snapshot budget that put them there.

The network process reads every value it sends on every cycle, inside a 4 ms
reply window. From a multiprocessing.Manager dict that read was one pipe round
trip per key (1.8 ms median, 4.3 ms worst case on the drilling laptop), and on
the KR C4 about 10 % of replies were late while a trajectory ran, each one a
feed step the robot never applied.

TestSnapshotBudget times that read against a publisher running at the
trajectory executor's rate and fails if IPC comes back into it. The rest pin
down what the store has to keep true for the rest of RSIPI: the dict
interface the API namespaces use, and a (seq, corrections) pair the network
process sees whole.
"""

import ctypes
import multiprocessing
import time

import pytest

from RSIPI.diagnostics_api import DiagnosticsAPI
from RSIPI.motion_api import MotionAPI
from RSIPI.shared_variables import SharedVariables
from RSIPI.timing_metrics import SNAPSHOT_BUDGET

from tests.conftest import wait_until

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

    def test_client_keeps_receive_variables_out_of_the_manager(self, rsi_stack):
        _server, client = rsi_stack(mode="relative")
        assert isinstance(client.receive_variables, SharedVariables)
        assert isinstance(client.network_process.receive_variables, SharedVariables)

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
class TestReaderNeverWaits:

    def test_snapshot_returns_the_last_frame_while_a_writer_is_stalled(self):
        store = SharedVariables(DRILL_RECEIVE)
        store.publish({"RKorr": {"X": 1.0}})
        before = store.snapshot()

        # A writer preempted mid-commit: lock held, generation odd.
        assert store._wlock.acquire(timeout=1)
        store._hdr[0] += 1
        try:
            started = time.perf_counter()
            assert store.snapshot() == before
            assert time.perf_counter() - started < 0.05
            assert store.stale_snapshots == 1
        finally:
            store._hdr[0] += 1
            store._wlock.release()

        store.publish({"RKorr": {"X": 2.0}})
        assert store.snapshot()[1]["RKorr"]["X"] == 2.0

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

    def test_torn_block_is_detected_and_the_next_write_repairs_it(self):
        store = SharedVariables(DRILL_RECEIVE)
        store.publish({"RKorr": {"X": 1.0}})
        good = store.snapshot()

        # An interrupted commit: bytes changed, generation even again, but
        # the CRC still describes the old frame.
        ctypes.memmove(ctypes.addressof(store._buf), b"\xff" * 64, 64)
        store._hdr[0] += 2

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
