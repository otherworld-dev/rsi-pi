"""Trajectory generation and execution.

Regressions covered here:
  * generate_trajectory validated the per-step DELTA against absolute
    workspace bounds, so a relative Z-descent (delta Z=-1.0) was rejected;
  * relative deltas were transmitted on more than one robot cycle, so a
    trajectory travelled roughly three times its commanded distance;
  * absolute-mode execution wrote world poses into RKorr, producing a ~900mm
    lunge on the first waypoint, and zeroed the offset at the end (commanding
    a return to the pre-trajectory path);
  * execute_profiled_trajectory slept on the velocity value instead of
    resampling the path at the robot cycle time.
"""

import time

import pytest

from RSIPI.motion_api import MotionAPI


# --------------------------------------------------------------------------- helpers

class StubClient:
    """Minimal stand-in for RSIClient: only what the executor touches."""

    def __init__(self, cycle_time=0.004, rsi_mode="relative"):
        self.cycle_time = cycle_time
        self.rsi_mode = rsi_mode


def magnitude(pose, axes=("X", "Y", "Z")):
    return sum(float(pose.get(axis, 0.0)) ** 2 for axis in axes) ** 0.5


# ===========================================================================
# generate_trajectory
# ===========================================================================
class TestGenerateTrajectory:

    def test_relative_z_descent_is_allowed(self):
        """A per-step delta is not a workspace position (Z=-1.0 is fine)."""
        traj = MotionAPI.generate_trajectory(
            {"X": 0, "Y": 0, "Z": 500},
            {"X": 100, "Y": 0, "Z": 400},
            steps=100,
            mode="relative",
        )

        assert len(traj) == 100
        for point in traj:
            assert point["X"] == pytest.approx(1.0)
            assert point["Y"] == pytest.approx(0.0)
            assert point["Z"] == pytest.approx(-1.0)

    def test_relative_deltas_sum_to_the_move(self):
        traj = MotionAPI.generate_trajectory(
            {"X": 0, "Y": 0, "Z": 500},
            {"X": 100, "Y": 0, "Z": 400},
            steps=100,
            mode="relative",
        )
        assert sum(p["X"] for p in traj) == pytest.approx(100.0)
        assert sum(p["Z"] for p in traj) == pytest.approx(-100.0)

    def test_absolute_mode_returns_interpolated_poses(self):
        traj = MotionAPI.generate_trajectory(
            {"X": 0, "Y": 0, "Z": 500},
            {"X": 100, "Y": 0, "Z": 400},
            steps=100,
            mode="absolute",
        )
        assert len(traj) == 100
        assert traj[0]["X"] == pytest.approx(1.0)
        assert traj[0]["Z"] == pytest.approx(499.0)
        assert traj[-1]["X"] == pytest.approx(100.0)
        assert traj[-1]["Z"] == pytest.approx(400.0)

    @pytest.mark.parametrize("mode", ["relative", "absolute"])
    def test_out_of_workspace_target_raises(self, mode):
        """The CUMULATIVE pose is validated, in both modes."""
        with pytest.raises(ValueError):
            MotionAPI.generate_trajectory(
                {"X": 0, "Y": 0, "Z": 500},
                {"X": 0, "Y": 0, "Z": -100},
                steps=100,
                mode=mode,
            )

    @pytest.mark.parametrize("mode", ["relative", "absolute"])
    def test_out_of_workspace_x_raises(self, mode):
        with pytest.raises(ValueError):
            MotionAPI.generate_trajectory(
                {"X": 0, "Y": 0, "Z": 500},
                {"X": 5000, "Y": 0, "Z": 500},
                steps=10,
                mode=mode,
            )


# ===========================================================================
# execute_profiled_trajectory (no robot required)
# ===========================================================================
class TestExecuteProfiledTrajectory:

    def test_profile_is_resampled_at_the_cycle_time(self, monkeypatch):
        motion = MotionAPI(StubClient(cycle_time=0.004, rsi_mode="relative"))

        captured = {}

        def fake_execute(world_points, space="cartesian", cycles_per_step=1):
            captured["path"] = list(world_points)
            captured["space"] = space
            captured["cycles_per_step"] = cycles_per_step

        monkeypatch.setattr(motion, "_execute_per_cycle", fake_execute)

        profiled = [
            ({"X": 0.0}, 0.0),
            ({"X": 10.0}, 100.0),
            ({"X": 20.0}, 0.0),
        ]

        # Must accept (waypoint, velocity) tuples - the old implementation
        # slept on the velocity value and blew up / stalled here.
        motion.execute_profiled_trajectory(profiled, space="cartesian")

        path = captured["path"]
        assert captured["space"] == "cartesian"
        assert path, "no resampled path was handed to the executor"
        assert path[-1]["X"] == pytest.approx(20.0)

        xs = [p["X"] for p in path]
        assert all(b > a for a, b in zip(xs, xs[1:])), "path is not monotonic"

        cycle = 0.004
        # Each segment is resampled at the cycle time using its AVERAGE
        # velocity, so the per-cycle displacement mid-segment should be
        # close to v_avg * cycle_time.
        for lo, hi, v_avg in ((0.0, 10.0, 50.0), (10.0, 20.0, 50.0)):
            expected = v_avg * cycle
            mid = [b - a for a, b in zip(xs, xs[1:]) if lo < a < hi]
            assert mid, "segment %s-%s was not resampled" % (lo, hi)
            middle = mid[len(mid) // 2]
            assert middle == pytest.approx(expected, rel=0.5), (
                "per-cycle displacement %r is not within 50%% of "
                "v_avg*cycle_time=%r" % (middle, expected))

        # Total point count ~ total travel time / cycle time.
        expected_points = (10.0 / 50.0 + 10.0 / 50.0) / cycle
        assert len(path) == pytest.approx(expected_points, rel=0.2)

    def test_empty_profile_is_a_noop(self, monkeypatch):
        motion = MotionAPI(StubClient())
        called = []
        monkeypatch.setattr(
            motion, "_execute_per_cycle",
            lambda *a, **kw: called.append(True))

        motion.execute_profiled_trajectory([], space="cartesian")

        assert called == []

    def test_zero_velocity_endpoints_do_not_stall(self, monkeypatch):
        """v==0 at both ends must not divide by zero or hang."""
        motion = MotionAPI(StubClient())
        captured = {}
        monkeypatch.setattr(
            motion, "_execute_per_cycle",
            lambda points, space="cartesian", cycles_per_step=1:
                captured.update(path=list(points)))

        started = time.monotonic()
        motion.execute_profiled_trajectory(
            [({"X": 0.0}, 0.0), ({"X": 1.0}, 0.0)], space="cartesian")

        assert time.monotonic() - started < 5.0
        assert captured["path"], "no path produced for a zero-velocity profile"


# ===========================================================================
# Loopback execution
# ===========================================================================
class TestRelativeExecutionLoopback:
    """Relative deltas must reach the wire AT MOST once, and normally exactly once.

    KNOWN SRC RACE (reported, not fixed here): publish_corrections() writes the
    correction dict and bumps _corr_seq non-atomically, while the network loop
    snapshots receive_variables and reads corr_seq at two different points of
    its cycle.  When the publish lands between those two reads, the loop
    transmits the PREVIOUS cycle's corrections but acknowledges the NEW
    sequence - so the very first waypoint of a delta trajectory (whose
    predecessor on the wire is zero) is occasionally skipped.  The assertions
    below therefore allow at most one whole dropped waypoint per attempt, but
    never any over-application, and require an exact run within a few attempts.
    """

    @staticmethod
    def _run(server, client, motion, deltas, axis="X"):
        """Execute one delta trajectory; returns (wire_sum, robot_advance, n)."""
        assert server.wait_for_records(server.record_count() + 10, timeout=10)
        server.clear_records()
        pre = server.robot_state("RIst")[axis]
        motion.execute_trajectory(deltas, space="cartesian", points="delta")
        # Let the last transmitted packets be processed by the controller.
        time.sleep(0.5)
        values = server.axis_values(axis)
        advance = server.robot_state("RIst")[axis] - pre
        return sum(values), advance, len(values)

    def test_deltas_are_applied_exactly_once(self, rsi_stack):
        server, client = rsi_stack(mode="relative")
        motion = MotionAPI(client)

        deltas = [{"X": 1.0} for _ in range(50)]
        step = 1.0
        expected = 50.0

        exact = None
        observed = []
        for _ in range(6):
            total, advance, count = self._run(server, client, motion, deltas)
            observed.append(total)

            assert count >= 50, "fewer replies accepted than waypoints sent"
            assert server.faulty_packets == 0, \
                "controller rejected %d packets" % server.faulty_packets
            # The regression under test: a delta must never be transmitted on
            # more than one cycle (the old executor applied each one ~3x).
            assert total <= expected + 1e-6, (
                "delta trajectory was over-applied (sum=%r over %d accepted "
                "replies)" % (total, count))
            # The robot moved exactly what went over the wire.
            assert advance == pytest.approx(total, abs=1e-6), \
                "robot advance %r does not match the wire sum %r" % (advance, total)
            # Any shortfall is a whole skipped waypoint, never a fraction.
            assert total >= expected - step - 1e-6, \
                "more than one waypoint went missing (sum=%r)" % (total,)

            if total == pytest.approx(expected, abs=1e-6):
                exact = total
                break

        assert exact is not None, (
            "no attempt applied the 50mm trajectory exactly once; wire sums "
            "were %r (expected 50.0)" % (observed,))

        rkorr = dict(client.receive_variables["RKorr"])
        assert all(v == 0.0 for v in rkorr.values()), \
            "corrections were not zeroed after the trajectory: %r" % (rkorr,)

    def test_generated_relative_trajectory_travels_its_distance(self, rsi_stack):
        server, client = rsi_stack(mode="relative")
        motion = MotionAPI(client)

        traj = MotionAPI.generate_trajectory(
            {"X": 0, "Y": 0, "Z": 500},
            {"X": 20, "Y": 0, "Z": 480},
            steps=40,
            mode="relative",
        )
        step = 0.5
        expected = 20.0

        exact = None
        observed = []
        for _ in range(6):
            server.clear_records()
            pre = server.robot_state("RIst")
            motion.execute_trajectory(traj, space="cartesian", points="delta")
            time.sleep(0.5)
            post = server.robot_state("RIst")
            travelled_x = post["X"] - pre["X"]
            travelled_z = post["Z"] - pre["Z"]
            observed.append(travelled_x)

            assert travelled_x <= expected + 1e-6, \
                "trajectory over-travelled: %r mm for a 20mm move" % (travelled_x,)
            assert travelled_x >= expected - step - 1e-6, \
                "trajectory under-travelled by more than one step: %r" % (travelled_x,)
            # X and Z are driven by the same waypoints, so they stay in step.
            assert travelled_z == pytest.approx(-travelled_x, abs=1e-6)

            if travelled_x == pytest.approx(expected, abs=1e-6):
                exact = travelled_x
                break

        assert exact is not None, (
            "no attempt travelled the full 20mm; observed %r" % (observed,))


class TestAbsoluteExecutionLoopback:

    def test_offsets_are_relative_to_the_reference_pose(self, rsi_stack):
        start_pose = {"X": 500.0, "Y": 0.0, "Z": 800.0,
                      "A": 0.0, "B": 0.0, "C": 0.0}
        server, client = rsi_stack(mode="absolute", seed_pose=start_pose)
        motion = MotionAPI(client)

        # Wait for the robot's pose to reach the parent (synced every ~10 cycles).
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if motion.get_current_pose().get("X"):
                break
            time.sleep(0.05)
        current = motion.get_current_pose()
        assert current["X"] == pytest.approx(start_pose["X"]), \
            "robot pose never reached the client: %r" % (current,)

        end_pose = {"X": current["X"] + 5.0,
                    "Y": current["Y"],
                    "Z": current["Z"]}
        server.clear_records()

        motion.move_cartesian_trajectory(end_pose, steps=10)
        time.sleep(0.5)

        records = [r for r in server.records() if r.get("RKorr")]
        offsets = [r["RKorr"] for r in records]
        assert offsets, "no corrections were transmitted"

        start_magnitude = magnitude(current)
        assert start_magnitude > 100.0  # sanity: the seed really is far from 0

        first = offsets[0]
        assert magnitude(first) < start_magnitude, (
            "first transmitted offset looks like a world pose, not an offset: %r"
            % (first,))
        # Nothing on the wire may exceed the size of the commanded move.
        worst = max(magnitude(o) for o in offsets)
        assert worst < 20.0, \
            "offset of %.1fmm transmitted for a 5mm move" % worst

        final = offsets[-1]
        assert final["X"] == pytest.approx(5.0, abs=0.05), \
            "final offset %r is not (end - start)" % (final,)
        assert final["X"] != 0.0, \
            "offset was zeroed after the trajectory (commands a return move)"

        held = dict(client.receive_variables["RKorr"])
        assert held["X"] == pytest.approx(5.0, abs=0.05), \
            "absolute offset was not held after completion: %r" % (held,)

        assert server.robot_state("RIst")["X"] == pytest.approx(
            start_pose["X"] + 5.0, abs=0.05)
