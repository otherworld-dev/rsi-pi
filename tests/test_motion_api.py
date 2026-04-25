"""Tests for MotionAPI static/pure methods."""
import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.motion_api import (
    MotionAPI,
    _calculate_distance,
    _trapezoidal_profile,
    _s_curve_profile,
    _cubic_blend,
    _find_blend_point,
)


# ---------------------------------------------------------------------------
# generate_arc
# ---------------------------------------------------------------------------
class TestGenerateArc:
    """Tests for MotionAPI.generate_arc()."""

    def test_basic_arc_length(self):
        """Arc with 50 steps should return 50 waypoints."""
        arc = MotionAPI.generate_arc(
            center={"X": 0, "Y": 0, "Z": 0},
            radius=10.0,
            start_angle=0,
            end_angle=90,
            steps=50,
        )
        assert len(arc) == 50

    def test_first_point_on_circle(self):
        """First point should be at start_angle on the circle."""
        arc = MotionAPI.generate_arc(
            center={"X": 100, "Y": 0, "Z": 0},
            radius=50.0,
            start_angle=0,
            end_angle=90,
            steps=10,
        )
        assert arc[0]["X"] == pytest.approx(150.0)
        assert arc[0]["Y"] == pytest.approx(0.0)

    def test_last_point_at_end_angle(self):
        """Last point should be at end_angle on the circle."""
        arc = MotionAPI.generate_arc(
            center={"X": 0, "Y": 0, "Z": 0},
            radius=10.0,
            start_angle=0,
            end_angle=90,
            steps=10,
        )
        assert arc[-1]["X"] == pytest.approx(10.0 * math.cos(math.radians(90)))
        assert arc[-1]["Y"] == pytest.approx(10.0 * math.sin(math.radians(90)))

    def test_xz_plane(self):
        """Arc in XZ plane should vary X and Z, keep Y constant."""
        arc = MotionAPI.generate_arc(
            center={"X": 0, "Y": 5, "Z": 0},
            radius=10.0,
            start_angle=0,
            end_angle=90,
            steps=5,
            plane="XZ",
        )
        for pt in arc:
            assert pt["Y"] == 5

    def test_yz_plane(self):
        """Arc in YZ plane should vary Y and Z, keep X constant."""
        arc = MotionAPI.generate_arc(
            center={"X": 7, "Y": 0, "Z": 0},
            radius=10.0,
            start_angle=0,
            end_angle=180,
            steps=5,
            plane="YZ",
        )
        for pt in arc:
            assert pt["X"] == 7

    def test_preserves_orientation(self):
        """Orientation keys A, B, C should carry through from center."""
        arc = MotionAPI.generate_arc(
            center={"X": 0, "Y": 0, "Z": 0, "A": 10, "B": 20, "C": 30},
            radius=5.0,
            start_angle=0,
            end_angle=45,
            steps=3,
        )
        for pt in arc:
            assert pt["A"] == 10
            assert pt["B"] == 20
            assert pt["C"] == 30

    def test_invalid_plane_raises(self):
        with pytest.raises(ValueError, match="Unknown plane"):
            MotionAPI.generate_arc(
                center={"X": 0, "Y": 0, "Z": 0},
                radius=5.0, start_angle=0, end_angle=90, steps=5, plane="AB"
            )

    def test_zero_radius_raises(self):
        with pytest.raises(ValueError, match="positive"):
            MotionAPI.generate_arc(
                center={"X": 0, "Y": 0, "Z": 0},
                radius=0.0, start_angle=0, end_angle=90, steps=5
            )

    def test_one_step_raises(self):
        with pytest.raises(ValueError, match="at least 2"):
            MotionAPI.generate_arc(
                center={"X": 0, "Y": 0, "Z": 0},
                radius=5.0, start_angle=0, end_angle=90, steps=1
            )


# ---------------------------------------------------------------------------
# generate_circle
# ---------------------------------------------------------------------------
class TestGenerateCircle:
    """Tests for MotionAPI.generate_circle()."""

    def test_returns_360_degree_arc(self):
        """Circle should span full 360 degrees."""
        circle = MotionAPI.generate_circle(
            center={"X": 0, "Y": 0, "Z": 0},
            radius=10.0,
            steps=100,
        )
        assert len(circle) == 100
        # First and last points should be nearly identical (full revolution)
        assert circle[0]["X"] == pytest.approx(circle[-1]["X"], abs=0.5)
        assert circle[0]["Y"] == pytest.approx(circle[-1]["Y"], abs=0.5)

    def test_circle_points_on_radius(self):
        """All points should lie on the specified radius from center."""
        cx, cy = 50.0, 30.0
        radius = 20.0
        circle = MotionAPI.generate_circle(
            center={"X": cx, "Y": cy, "Z": 0},
            radius=radius,
            steps=36,
        )
        for pt in circle:
            dist = math.sqrt((pt["X"] - cx) ** 2 + (pt["Y"] - cy) ** 2)
            assert dist == pytest.approx(radius, abs=1e-10)


# ---------------------------------------------------------------------------
# generate_spiral
# ---------------------------------------------------------------------------
class TestGenerateSpiral:
    """Tests for MotionAPI.generate_spiral()."""

    def test_expanding_spiral_radius_increases(self):
        """Radius should increase from start_radius to end_radius."""
        spiral = MotionAPI.generate_spiral(
            center={"X": 0, "Y": 0, "Z": 0},
            start_radius=5.0,
            end_radius=50.0,
            pitch=0.0,
            revolutions=2,
            steps=50,
        )
        first_r = math.sqrt(spiral[0]["X"] ** 2 + spiral[0]["Y"] ** 2)
        last_r = math.sqrt(spiral[-1]["X"] ** 2 + spiral[-1]["Y"] ** 2)
        assert last_r > first_r

    def test_contracting_spiral_radius_decreases(self):
        """Radius should decrease when end_radius < start_radius."""
        spiral = MotionAPI.generate_spiral(
            center={"X": 0, "Y": 0, "Z": 0},
            start_radius=50.0,
            end_radius=5.0,
            pitch=0.0,
            revolutions=2,
            steps=50,
        )
        first_r = math.sqrt(spiral[0]["X"] ** 2 + spiral[0]["Y"] ** 2)
        last_r = math.sqrt(spiral[-1]["X"] ** 2 + spiral[-1]["Y"] ** 2)
        assert last_r < first_r

    def test_pitch_adds_height(self):
        """Positive pitch should increase Z across the spiral."""
        spiral = MotionAPI.generate_spiral(
            center={"X": 0, "Y": 0, "Z": 100},
            start_radius=10.0,
            end_radius=10.0,
            pitch=20.0,
            revolutions=1,
            steps=20,
        )
        assert spiral[-1]["Z"] > spiral[0]["Z"]

    def test_step_count(self):
        """Should return exactly the requested number of steps."""
        spiral = MotionAPI.generate_spiral(
            center={"X": 0, "Y": 0, "Z": 0},
            start_radius=10.0,
            end_radius=20.0,
            pitch=5.0,
            steps=33,
        )
        assert len(spiral) == 33

    def test_too_few_steps_raises(self):
        with pytest.raises(ValueError, match="at least 2"):
            MotionAPI.generate_spiral(
                center={"X": 0, "Y": 0, "Z": 0},
                start_radius=5, end_radius=10, pitch=1, steps=1
            )


# ---------------------------------------------------------------------------
# blend_trajectories
# ---------------------------------------------------------------------------
class TestBlendTrajectories:
    """Tests for MotionAPI.blend_trajectories()."""

    @staticmethod
    def _line(start_x, end_x, n=20):
        """Helper: straight line along X."""
        return [{"X": start_x + i * (end_x - start_x) / (n - 1), "Y": 0, "Z": 0} for i in range(n)]

    def test_blended_length(self):
        """Blended trajectory should have points from both segments plus blend zone."""
        t1 = self._line(0, 100, 20)
        t2 = self._line(100, 200, 20)
        blended = MotionAPI.blend_trajectories(t1, t2, blend_radius=10.0, blend_steps=10)
        # Should have some points; exact count depends on blend_radius vs traj length
        assert len(blended) > 0

    def test_blended_starts_at_traj1_start(self):
        """First point of blended should match first point of traj1."""
        t1 = self._line(0, 100, 20)
        t2 = self._line(100, 200, 20)
        blended = MotionAPI.blend_trajectories(t1, t2, blend_radius=5.0, blend_steps=10)
        assert blended[0]["X"] == pytest.approx(0.0)

    def test_blended_ends_at_traj2_end(self):
        """Last point of blended should match last point of traj2."""
        t1 = self._line(0, 100, 20)
        t2 = self._line(100, 200, 20)
        blended = MotionAPI.blend_trajectories(t1, t2, blend_radius=5.0, blend_steps=10)
        assert blended[-1]["X"] == pytest.approx(200.0)

    def test_empty_traj_raises(self):
        with pytest.raises(ValueError, match="non-empty"):
            MotionAPI.blend_trajectories([], [{"X": 0}], blend_radius=1.0)

    def test_zero_blend_radius_raises(self):
        with pytest.raises(ValueError, match="positive"):
            MotionAPI.blend_trajectories(
                [{"X": 0}], [{"X": 1}], blend_radius=0.0
            )


# ---------------------------------------------------------------------------
# transform_coordinates
# ---------------------------------------------------------------------------
class TestTransformCoordinates:
    """Tests for MotionAPI.transform_coordinates()."""

    def test_no_offset_returns_copy(self):
        """No frame_offset should return a copy of the input pose."""
        pose = {"X": 10, "Y": 20, "Z": 30}
        result = MotionAPI.transform_coordinates(pose)
        assert result == pose
        # Should be a copy, not the same object
        assert result is not pose

    def test_translational_offset(self):
        """Frame offset should add to position values."""
        pose = {"X": 100, "Y": 0, "Z": 500}
        offset = {"X": 500, "Y": 200, "Z": 0}
        result = MotionAPI.transform_coordinates(pose, frame_offset=offset)
        assert result["X"] == 600
        assert result["Y"] == 200
        assert result["Z"] == 500

    def test_rotational_offset(self):
        """Frame offset should add to orientation values."""
        pose = {"X": 0, "Y": 0, "Z": 0, "A": 10, "B": 20, "C": 30}
        offset = {"A": 5, "B": -10, "C": 15}
        result = MotionAPI.transform_coordinates(pose, frame_offset=offset)
        assert result["A"] == 15
        assert result["B"] == 10
        assert result["C"] == 45

    def test_partial_offset(self):
        """Offset with fewer keys should only affect matching axes."""
        pose = {"X": 100, "Y": 200, "Z": 300}
        offset = {"X": 10}
        result = MotionAPI.transform_coordinates(pose, frame_offset=offset)
        assert result["X"] == 110
        assert result["Y"] == 200  # unchanged
        assert result["Z"] == 300  # unchanged


# ---------------------------------------------------------------------------
# generate_velocity_profile
# ---------------------------------------------------------------------------
class TestGenerateVelocityProfile:
    """Tests for MotionAPI.generate_velocity_profile()."""

    @staticmethod
    def _simple_traj(n=50):
        """Evenly spaced points along X."""
        return [{"X": i * 1.0, "Y": 0, "Z": 0} for i in range(n)]

    def test_trapezoidal_returns_correct_length(self):
        traj = self._simple_traj(50)
        result = MotionAPI.generate_velocity_profile(traj, profile='trapezoidal')
        assert len(result) == 50

    def test_scurve_returns_correct_length(self):
        traj = self._simple_traj(50)
        result = MotionAPI.generate_velocity_profile(traj, profile='s-curve')
        assert len(result) == 50

    def test_trapezoidal_starts_and_ends_near_zero(self):
        """Velocity should be near zero at start and end."""
        traj = self._simple_traj(100)
        result = MotionAPI.generate_velocity_profile(
            traj, max_velocity=10.0, max_acceleration=5.0, profile='trapezoidal'
        )
        _, v_first = result[0]
        _, v_last = result[-1]
        assert v_first == pytest.approx(0.0, abs=0.5)
        assert v_last == pytest.approx(0.0, abs=0.5)

    def test_scurve_starts_and_ends_near_zero(self):
        traj = self._simple_traj(100)
        result = MotionAPI.generate_velocity_profile(
            traj, max_velocity=10.0, max_acceleration=5.0, profile='s-curve'
        )
        _, v_first = result[0]
        _, v_last = result[-1]
        assert v_first == pytest.approx(0.0, abs=0.5)
        assert v_last == pytest.approx(0.0, abs=0.5)

    def test_empty_trajectory(self):
        assert MotionAPI.generate_velocity_profile([]) == []

    def test_single_point(self):
        result = MotionAPI.generate_velocity_profile([{"X": 0}])
        assert len(result) == 1
        assert result[0][1] == 0.0

    def test_unknown_profile_raises(self):
        traj = self._simple_traj(10)
        with pytest.raises(ValueError, match="Unknown profile"):
            MotionAPI.generate_velocity_profile(traj, profile='banana')

    def test_trapezoidal_velocity_does_not_exceed_max(self):
        """No velocity should exceed max_velocity."""
        traj = self._simple_traj(100)
        max_v = 5.0
        result = MotionAPI.generate_velocity_profile(
            traj, max_velocity=max_v, max_acceleration=2.0, profile='trapezoidal'
        )
        for _, v in result:
            assert v <= max_v + 1e-9


# ---------------------------------------------------------------------------
# _cubic_blend edge cases
# ---------------------------------------------------------------------------
class TestCubicBlend:
    """Tests for _cubic_blend helper."""

    def test_steps_one_no_crash(self):
        """steps=1 should not cause division by zero."""
        result = _cubic_blend({"X": 0, "Y": 0}, {"X": 10, "Y": 10}, steps=1)
        assert len(result) == 1

    def test_steps_two(self):
        """steps=2 should interpolate from p1 to p2."""
        result = _cubic_blend({"X": 0}, {"X": 100}, steps=2)
        assert len(result) == 2
        assert result[0]["X"] == pytest.approx(0.0)
        assert result[-1]["X"] == pytest.approx(100.0)

    def test_midpoint_is_blended(self):
        """Midpoint (t=0.5) of cubic Hermite with zero velocities should be average."""
        result = _cubic_blend({"X": 0}, {"X": 100}, steps=3)
        # At t=0.5: h1=0.5, h2=0.5 -> blended = 50
        assert result[1]["X"] == pytest.approx(50.0)


# ---------------------------------------------------------------------------
# _trapezoidal_profile edge cases
# ---------------------------------------------------------------------------
class TestTrapezoidalProfileEdgeCases:
    """Edge-case tests for the _trapezoidal_profile helper."""

    def test_single_distance(self):
        """Profile with one distance segment should not crash."""
        velocities = _trapezoidal_profile([10.0], 10.0, 5.0, 2.0)
        assert len(velocities) == 2  # n = len(distances) + 1

    def test_zero_total_distance(self):
        """Zero total distance should produce zero velocities without error."""
        velocities = _trapezoidal_profile([0.0], 0.0, 5.0, 2.0)
        assert all(v == 0.0 for v in velocities)

    def test_triangular_profile_triggered(self):
        """Short distance forces triangular profile (no constant phase)."""
        # accel_distance = v^2 / (2*a) = 100/4 = 25
        # total = 10 < 2*25 => triangular
        velocities = _trapezoidal_profile([5.0, 5.0], 10.0, 10.0, 2.0)
        assert len(velocities) == 3


# ---------------------------------------------------------------------------
# _calculate_distance
# ---------------------------------------------------------------------------
class TestCalculateDistance:
    """Tests for _calculate_distance helper."""

    def test_zero_distance(self):
        assert _calculate_distance({"X": 0, "Y": 0}, {"X": 0, "Y": 0}) == 0.0

    def test_unit_distance(self):
        assert _calculate_distance({"X": 0}, {"X": 1}) == pytest.approx(1.0)

    def test_3d_distance(self):
        d = _calculate_distance({"X": 0, "Y": 0, "Z": 0}, {"X": 3, "Y": 4, "Z": 0})
        assert d == pytest.approx(5.0)

    def test_ignores_non_motion_keys(self):
        """Keys not in the motion set should be ignored."""
        d = _calculate_distance({"X": 0, "FOO": 0}, {"X": 3, "FOO": 1000})
        assert d == pytest.approx(3.0)
