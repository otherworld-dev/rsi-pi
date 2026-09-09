"""Lifecycle behaviour: watchdog reporting, clean interpreter exit, reconnect.

Covers the regressions that only show up around the edges of a session:
  * the health snapshot published to the parent was structurally always
    unhealthy, so a real communication loss was indistinguishable from noise;
  * an uncaught exception in user code used to leave the interpreter hanging
    on non-daemon children / an un-run cleanup;
  * a latched E-stop had to survive a reconnect (a replacement network
    process must enforce it from its very first packet);
  * AutoReconnectManager.stop() could be reached from the monitor thread
    itself and blow up with "cannot join current thread".
"""

import os
import subprocess
import sys
import threading
import time

import pytest

from RSIPI.auto_reconnect import AutoReconnectManager, ReconnectStrategy


CRASH_SCRIPT = '''\
"""Start RSI communication, then die with an uncaught exception."""
import sys
import time

sys.path.insert(0, {src!r})

from RSIPI.rsi_api import RSIAPI


def main():
    api = RSIAPI({config!r})
    api.start()
    time.sleep(1.0)
    raise RuntimeError("simulated user crash")


if __name__ == "__main__":
    main()
'''


def write_correction(client, corr_key="RKorr", **axes):
    current = client.receive_variables.get(corr_key)
    merged = dict(current) if isinstance(current, dict) else {}
    for axis, value in axes.items():
        merged[axis] = float(value)
    client.receive_variables[corr_key] = merged


# ===========================================================================
# Watchdog reporting
# ===========================================================================
class TestWatchdog:

    def test_watchdog_timeout_is_published_when_robot_goes_silent(self, rsi_stack):
        server, client = rsi_stack(mode="relative")

        # Healthy link first: metrics are published every ~100 cycles.
        assert server.wait_for_records(120, timeout=10)

        server.stop()  # robot stops talking

        deadline = time.monotonic() + 8.0
        metrics = {}
        while time.monotonic() < deadline:
            metrics = dict(client.metrics_dict)
            if metrics.get("watchdog_timeout"):
                break
            time.sleep(0.05)

        assert metrics.get("watchdog_timeout") is True, (
            "watchdog_timeout never became True after the robot went silent "
            "(metrics=%r)" % (metrics,))
        assert metrics.get("is_healthy") is False, \
            "is_healthy must be False while the watchdog is timed out"
        assert metrics.get("total_cycles", 0) > 0, \
            "metrics were never populated from real traffic"


# ===========================================================================
# Interpreter exit
# ===========================================================================
class TestInterpreterExit:

    def test_uncaught_exception_does_not_hang_the_interpreter(
            self, tmp_path, repo_root, src_dir, config_file,
            require_ports, client_port):
        require_ports(client_port)

        script = tmp_path / "rsi_crash.py"
        script.write_text(
            CRASH_SCRIPT.format(src=src_dir, config=config_file), encoding="utf-8")

        started = time.monotonic()
        try:
            completed = subprocess.run(
                [sys.executable, str(script)],
                cwd=str(repo_root),
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            pytest.fail(
                "interpreter hung for 15s after an uncaught exception - "
                "a non-daemon child or an un-run cleanup is blocking exit")

        elapsed = time.monotonic() - started
        assert elapsed < 15
        # Any exit code is fine; the traceback must be the one we raised.
        assert "simulated user crash" in completed.stderr, completed.stderr[-2000:]


# ===========================================================================
# Reconnect
# ===========================================================================
class TestReconnect:

    def test_reconnect_preserves_latched_estop(self, rsi_stack):
        server, client = rsi_stack(mode="relative")

        write_correction(client, X=1.0)
        assert server.wait_for_records(server.record_count() + 30, timeout=10)
        assert server.axis_values("X")[-1] == pytest.approx(1.0)

        client.emergency_stop()
        time.sleep(0.2)

        client.reconnect(restart=True)
        assert client.wait_for_connection(20), "reconnect never re-established the link"

        # The replacement network process must enforce the latch immediately.
        assert bool(client._estop_active.value) is True, \
            "E-stop latch was lost across reconnect()"

        server.clear_records()
        assert server.wait_for_records(40, timeout=10)
        values = server.axis_values("X")
        assert all(v == 0.0 for v in values), (
            "corrections resumed after reconnect while E-stop was latched: %r"
            % (sorted({v for v in values if v != 0.0}),))

        # ...and traffic resumes normally once the E-stop is cleared.
        client.emergency_reset()
        seq = client.publish_corrections({"RKorr": {"X": 3.0}})
        assert client.wait_correction_applied(seq, timeout=5.0)

        mark = server.record_count()
        assert server.wait_for_records(mark + 10, timeout=10)
        assert server.axis_values("X", start=mark)[-1] == pytest.approx(3.0), \
            "corrections did not flow again after emergency_reset()"


# ===========================================================================
# AutoReconnectManager
# ===========================================================================
class TestAutoReconnectManagerStop:

    def test_stop_from_monitor_thread_does_not_raise(self):
        """stop() reached from the monitor thread must not self-join."""
        manager = AutoReconnectManager(
            client=None, enabled=True, strategy=ReconnectStrategy.IMMEDIATE)
        manager._running = True

        outcome = {}

        def worker():
            # Simulate being *inside* the monitor thread when stop() is called
            # (the _attempt_reconnection -> client teardown -> stop() path).
            manager._monitor_thread = threading.current_thread()
            try:
                manager.stop()
            except BaseException as exc:  # noqa: BLE001 - the point of the test
                outcome["error"] = exc
            else:
                outcome["ok"] = True

        thread = threading.Thread(target=worker)
        thread.start()
        thread.join(timeout=10)

        assert not thread.is_alive(), "stop() blocked when called from the monitor thread"
        assert "error" not in outcome, \
            "stop() raised from the monitor thread: %r" % (outcome.get("error"),)
        assert outcome.get("ok") is True
        assert manager._running is False
        assert manager._stop_event.is_set()

    def test_stop_from_another_thread_still_joins(self):
        """The guard must not disable the normal join path."""
        manager = AutoReconnectManager(
            client=None, enabled=True, strategy=ReconnectStrategy.IMMEDIATE)
        manager._running = True

        started = threading.Event()

        def idle():
            started.set()
            manager._stop_event.wait(10)

        monitor = threading.Thread(target=idle, daemon=True)
        manager._monitor_thread = monitor
        monitor.start()
        assert started.wait(5)

        manager.stop()

        assert not monitor.is_alive(), "stop() did not join the monitor thread"

    def test_stop_is_a_noop_when_not_running(self):
        manager = AutoReconnectManager(client=None, enabled=True)
        manager.stop()  # must not raise even though no thread exists
        assert manager._running is False


class TestStartSurfacesFailure:
    """start() must not report success when the client thread died.

    It runs client.start() in a background thread, and client.start() blocks
    in its control loop while healthy - so a thread that has already finished
    did not survive start-up. Without this check the caller gets
    "RSI started in background" and only discovers the fault when
    wait_for_connection() times out, which at a robot means minutes spent
    blaming the pendant for a fault on the PC side.
    """

    def test_healthy_start_still_returns(self, config_file):
        from RSIPI.rsi_api import RSIAPI
        api = RSIAPI(config_file)
        try:
            assert "started" in api.start().lower()
            assert api._thread.is_alive()
        finally:
            api.stop()

    def test_a_failed_start_raises_instead_of_reporting_success(self, config_file):
        from RSIPI.rsi_api import RSIAPI
        from RSIPI.exceptions import RSIClientNotReady
        api = RSIAPI(config_file)
        try:
            api.start()
            # Starting again is an invalid state transition, so client.start()
            # raises inside the thread.
            with pytest.raises(RSIClientNotReady):
                api.start()
        finally:
            api.stop()
