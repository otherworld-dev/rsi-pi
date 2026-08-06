"""E-stop and send-time safety enforcement in the network process.

Two layers are covered:

* unit level - the per-cycle stages of NetworkProcess (_enforce_limits,
  _apply_rate_limit, _handle_estop_transition) driven directly, with no UDP
  traffic and no child process;
* loopback level - a real RSIClient talking to the recording echo server, so
  what actually reaches the wire is asserted rather than what the API intends.

Wire values are read from the <Sen> replies the echo server *accepted*: a
rejected packet's corrections are never applied by a real controller either.
"""

import time

import pytest

from RSIPI.motion_api import MotionAPI


CARTESIAN_AXES = ("X", "Y", "Z", "A", "B", "C")


# --------------------------------------------------------------------------- helpers

def write_correction(client, corr_key="RKorr", **axes):
    """Direct write into receive_variables (bypasses parent-side validation).

    This is the path a streaming user takes - and the one that must still be
    clamped/substituted at send time.
    """
    current = client.receive_variables.get(corr_key)
    merged = dict(current) if isinstance(current, dict) else {}
    for axis, value in axes.items():
        merged[axis] = float(value)
    client.receive_variables[corr_key] = merged


def zeros(*axes):
    return {axis: 0.0 for axis in (axes or CARTESIAN_AXES)}


# ===========================================================================
# Unit: _enforce_limits (send-time clamping)
# ===========================================================================
class TestEnforceLimits:
    """NetworkProcess._enforce_limits clamps, never raises."""

    def test_clamps_dotted_correction_path(self, make_network_process):
        process = make_network_process(limits={"RKorr.X": (-5.0, 5.0)})
        robot_in = {"RKorr": {"X": 50.0, "Y": 1.0}}

        process._enforce_limits(robot_in)

        assert robot_in["RKorr"]["X"] == pytest.approx(5.0)
        assert robot_in["RKorr"]["Y"] == pytest.approx(1.0)  # inside limits, untouched

    def test_clamps_negative_side(self, make_network_process):
        process = make_network_process(limits={"RKorr.X": (-5.0, 5.0)})
        robot_in = {"RKorr": {"X": -50.0}}

        process._enforce_limits(robot_in)

        assert robot_in["RKorr"]["X"] == pytest.approx(-5.0)

    def test_clamps_scalar_variable(self, make_network_process):
        process = make_network_process(limits={"DiO": (0.0, 10.0)})
        robot_in = {"DiO": 99}

        process._enforce_limits(robot_in)

        assert robot_in["DiO"] == pytest.approx(10.0)

    def test_override_disables_clamping(self, make_network_process):
        process = make_network_process(limits={"RKorr.X": (-5.0, 5.0)}, override=True)
        robot_in = {"RKorr": {"X": 50.0}}

        process._enforce_limits(robot_in)

        assert robot_in["RKorr"]["X"] == pytest.approx(50.0)

    def test_booleans_are_not_clamped_as_numbers(self, make_network_process):
        """bool is a subclass of int - digital outputs must pass through."""
        process = make_network_process(limits={"Digout": (0.0, 0.0)})
        robot_in = {"Digout": True}

        process._enforce_limits(robot_in)

        assert robot_in["Digout"] is True


# ===========================================================================
# Unit: _apply_rate_limit (absolute-mode ramping / prev tracking)
# ===========================================================================
class TestApplyRateLimit:
    """Absolute mode ramps toward the target and always tracks prev."""

    def test_absolute_mode_ramps_by_at_most_max_rate(self, make_network_process):
        process = make_network_process(rsi_mode="absolute", max_cartesian_rate=0.1)
        prev = {}
        target = 1.0
        transmitted = []

        for _ in range(5):
            robot_in = {"RKorr": {"X": target, "Y": 0.0}}
            process._apply_rate_limit(robot_in, prev)
            transmitted.append(robot_in["RKorr"]["X"])
            # prev must mirror what was actually transmitted this cycle
            assert prev["RKorr"]["X"] == pytest.approx(robot_in["RKorr"]["X"])

        assert transmitted[0] == pytest.approx(0.1)
        previous = 0.0
        for value in transmitted:
            assert value - previous <= 0.1 + 1e-9
            assert value > previous  # ramping toward the target, not jumping
            previous = value
        assert transmitted[-1] == pytest.approx(0.5)
        assert transmitted[-1] < target

    def test_absolute_mode_ramp_reaches_target_and_stops(self, make_network_process):
        process = make_network_process(rsi_mode="absolute", max_cartesian_rate=0.1)
        prev = {}
        target = 1.0

        for _ in range(15):
            robot_in = {"RKorr": {"X": target}}
            process._apply_rate_limit(robot_in, prev)

        assert robot_in["RKorr"]["X"] == pytest.approx(target)
        assert prev["RKorr"]["X"] == pytest.approx(target)

    def test_zero_rate_only_records_prev(self, make_network_process):
        process = make_network_process(rsi_mode="absolute", max_cartesian_rate=0.0)
        prev = {}
        robot_in = {"RKorr": {"X": 7.5, "Y": -3.0}}

        process._apply_rate_limit(robot_in, prev)

        # No ramp: the target goes out untouched...
        assert robot_in["RKorr"]["X"] == pytest.approx(7.5)
        assert robot_in["RKorr"]["Y"] == pytest.approx(-3.0)
        # ...but prev is still tracked, so an E-stop can freeze it.
        assert prev["RKorr"]["X"] == pytest.approx(7.5)
        assert prev["RKorr"]["Y"] == pytest.approx(-3.0)

    def test_relative_mode_clamps_delta_directly(self, make_network_process):
        process = make_network_process(rsi_mode="relative", max_cartesian_rate=0.1)
        prev = {}
        robot_in = {"RKorr": {"X": 5.0}}

        process._apply_rate_limit(robot_in, prev)

        assert robot_in["RKorr"]["X"] == pytest.approx(0.1)


# ===========================================================================
# Unit: _handle_estop_transition (rising edge clears the source)
# ===========================================================================
class TestEstopTransition:
    """The rising edge must clear/freeze corrections in receive_variables."""

    def test_relative_rising_edge_zeroes_source_and_prev(self, make_network_process):
        process = make_network_process(rsi_mode="relative")
        process.receive_variables = {"RKorr": {"X": 1.5, "Y": -2.0}}
        prev = {"RKorr": {"X": 1.5, "Y": -2.0}}
        process.estop_active.value = True

        active = process._handle_estop_transition(prev, False)

        assert active is True
        assert process.receive_variables["RKorr"] == {"X": 0.0, "Y": 0.0}
        assert prev["RKorr"] == {"X": 0.0, "Y": 0.0}

    def test_absolute_rising_edge_freezes_last_offset(self, make_network_process):
        process = make_network_process(rsi_mode="absolute")
        process.receive_variables = {"RKorr": {"X": 9.0, "Y": 0.0}}
        prev = {"RKorr": {"X": 4.0, "Y": 0.0}}
        process.estop_active.value = True

        active = process._handle_estop_transition(prev, False)

        assert active is True
        # Frozen at the last transmitted offset - NOT zeroed (that would
        # command a return-to-path motion) and NOT the stale 9.0 target.
        assert process.receive_variables["RKorr"] == {"X": 4.0, "Y": 0.0}
        assert prev["RKorr"] == {"X": 4.0, "Y": 0.0}

    def test_no_rising_edge_leaves_source_untouched(self, make_network_process):
        """Already-latched E-stop must not keep rewriting receive_variables."""
        process = make_network_process(rsi_mode="relative")
        process.receive_variables = {"RKorr": {"X": 1.5}}
        prev = {"RKorr": {"X": 0.0}}
        process.estop_active.value = True

        active = process._handle_estop_transition(prev, True)

        assert active is True
        assert process.receive_variables["RKorr"] == {"X": 1.5}

    def test_inactive_estop_reports_false(self, make_network_process):
        process = make_network_process(rsi_mode="relative")
        process.receive_variables = {"RKorr": {"X": 1.5}}
        prev = {"RKorr": {"X": 1.5}}

        assert process._handle_estop_transition(prev, False) is False
        assert process.receive_variables["RKorr"] == {"X": 1.5}


# ===========================================================================
# Loopback: what actually reaches the wire
# ===========================================================================
class TestEstopLoopback:

    def test_estop_zeroes_wire_and_reset_does_not_resume(self, rsi_stack):
        """E-stop zeroes the wire fast, and reset must NOT resume motion."""
        server, client = rsi_stack(mode="relative")

        # --- stream a constant correction -------------------------------
        write_correction(client, X=1.0)
        assert server.wait_for_records(server.record_count() + 40, timeout=10), \
            "echo server accepted too few replies - link not streaming"
        streaming = server.axis_values("X")
        assert streaming[-5:] == [pytest.approx(1.0)] * 5, \
            "correction is not reaching the wire before the E-stop"

        # --- E-stop ------------------------------------------------------
        mark = server.record_count()
        client.emergency_stop()
        assert server.wait_for_records(mark + 30, timeout=10)
        after_stop = server.axis_values("X", start=mark)

        first_zero = next((i for i, v in enumerate(after_stop) if v == 0.0), None)
        assert first_zero is not None, "E-stop never reached the wire"
        assert first_zero <= 5, (
            "E-stop took %d accepted replies to reach the wire" % first_zero)
        assert all(v == 0.0 for v in after_stop[first_zero:]), \
            "non-zero correction transmitted while E-stop was latched"

        # --- reset must not resume the stale correction ------------------
        mark = server.record_count()
        client.emergency_reset()
        assert server.wait_for_records(mark + 40, timeout=10)
        after_reset = server.axis_values("X", start=mark)
        assert after_reset, "no replies accepted after E-stop reset"
        assert all(v == 0.0 for v in after_reset), (
            "motion resumed from a stale correction after emergency_reset: %r"
            % (sorted({v for v in after_reset if v != 0.0}),))

        # --- an explicit new correction flows again ----------------------
        seq = client.publish_corrections({"RKorr": {"X": 2.0}})
        assert client.wait_correction_applied(seq, timeout=5.0)
        mark = server.record_count()
        assert server.wait_for_records(mark + 10, timeout=10)
        assert server.axis_values("X", start=mark)[-1] == pytest.approx(2.0)

    def test_estop_survives_direct_writes(self, rsi_stack):
        """Writing straight into receive_variables cannot defeat the E-stop."""
        server, client = rsi_stack(mode="relative")
        client.emergency_stop()
        time.sleep(0.2)

        server.clear_records()
        write_correction(client, X=5.0)
        assert server.wait_for_records(30, timeout=10)

        values = server.axis_values("X")
        assert all(v == 0.0 for v in values), \
            "direct write bypassed the E-stop substitution: %r" % (values[:10],)

    def test_send_time_clamp_of_direct_write(self, rsi_stack):
        """A runtime limit clamps values written straight into the dict."""
        server, client = rsi_stack(mode="relative")

        client.set_limit("RKorr.X", -1.0, 1.0)
        # The child drains its command queue every ~10 cycles; give it time
        # to install the limit before the offending value is written.
        time.sleep(1.0)

        server.clear_records()
        write_correction(client, X=50.0)
        assert server.wait_for_records(40, timeout=10)

        values = server.axis_values("X")
        assert max(values) <= 1.0 + 1e-6, (
            "value above the runtime limit reached the wire: max=%r" % max(values))
        assert max(values) >= 1.0 - 1e-6, \
            "clamped value never reached the wire at all"
        assert min(values) >= -1.0 - 1e-6

    def test_motion_api_updates_are_clamped_on_the_wire(self, rsi_stack):
        """The same clamp applies to values that went through MotionAPI."""
        server, client = rsi_stack(mode="relative")
        motion = MotionAPI(client)

        client.set_limit("RKorr.Y", -2.0, 2.0)
        time.sleep(1.0)
        client.set_safety_override(True)  # parent-side validation bypassed
        time.sleep(1.0)

        server.clear_records()
        motion.update_cartesian(Y=25.0)
        assert server.wait_for_records(40, timeout=10)

        # Override is honoured by BOTH layers: nothing is clamped.
        assert max(server.axis_values("Y")) == pytest.approx(25.0)

        client.set_safety_override(False)
        time.sleep(1.0)
        server.clear_records()
        assert server.wait_for_records(40, timeout=10)
        assert max(server.axis_values("Y")) <= 2.0 + 1e-6, \
            "limit not re-applied after the override was cleared"
