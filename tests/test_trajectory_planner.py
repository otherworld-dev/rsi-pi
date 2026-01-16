"""Tests for trajectory planner."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.trajectory_planner import generate_trajectory


class TestGenerateTrajectory:
    """Tests for generate_trajectory()"""

    def test_basic_linear_interpolation(self):
        """Test basic linear interpolation between two points."""
        start = {"X": 0, "Y": 0, "Z": 0}
        end = {"X": 100, "Y": 200, "Z": 300}

        traj = generate_trajectory(start, end, steps=10, space="cartesian", mode="absolute")

        assert len(traj) == 10

        # First point should be 1/10 of the way
        assert traj[0]["X"] == pytest.approx(10.0)
        assert traj[0]["Y"] == pytest.approx(20.0)
        assert traj[0]["Z"] == pytest.approx(30.0)

        # Last point should be at end
        assert traj[-1]["X"] == pytest.approx(100.0)
        assert traj[-1]["Y"] == pytest.approx(200.0)
        assert traj[-1]["Z"] == pytest.approx(300.0)

    def test_relative_mode(self):
        """Test relative mode generates incremental deltas."""
        start = {"X": 0, "Y": 0}
        end = {"X": 100, "Y": 50}

        traj = generate_trajectory(start, end, steps=10, space="cartesian", mode="relative")

        # Each step should be the same delta
        for point in traj:
            assert point["X"] == pytest.approx(10.0)  # 100/10
            assert point["Y"] == pytest.approx(5.0)   # 50/10

    def test_relative_mode_with_resets(self):
        """Test relative mode with reset steps."""
        start = {"X": 0}
        end = {"X": 30}

        traj = generate_trajectory(start, end, steps=3, space="cartesian", mode="relative", include_resets=True)

        # Should have 6 points: delta, reset, delta, reset, delta, reset
        assert len(traj) == 6

        # Odd indices should be zero (reset points)
        assert traj[1]["X"] == 0.0
        assert traj[3]["X"] == 0.0
        assert traj[5]["X"] == 0.0

        # Even indices should be deltas
        assert traj[0]["X"] == pytest.approx(10.0)
        assert traj[2]["X"] == pytest.approx(10.0)
        assert traj[4]["X"] == pytest.approx(10.0)

    def test_joint_space(self):
        """Test trajectory generation in joint space."""
        start = {"A1": 0, "A2": 0}
        end = {"A1": 90, "A2": -45}

        traj = generate_trajectory(start, end, steps=9, space="joint", mode="absolute")

        assert len(traj) == 9

        # Final point should match end
        assert traj[-1]["A1"] == pytest.approx(90.0)
        assert traj[-1]["A2"] == pytest.approx(-45.0)

    def test_negative_movement(self):
        """Test trajectory with negative direction."""
        start = {"X": 100, "Y": 100}
        end = {"X": 0, "Y": -100}

        traj = generate_trajectory(start, end, steps=10, space="cartesian", mode="absolute")

        # Should decrease linearly
        assert traj[0]["X"] == pytest.approx(90.0)
        assert traj[0]["Y"] == pytest.approx(80.0)
        assert traj[-1]["X"] == pytest.approx(0.0)
        assert traj[-1]["Y"] == pytest.approx(-100.0)

    def test_single_step(self):
        """Test trajectory with single step goes directly to end."""
        start = {"X": 0}
        end = {"X": 100}

        traj = generate_trajectory(start, end, steps=1, space="cartesian", mode="absolute")

        assert len(traj) == 1
        assert traj[0]["X"] == pytest.approx(100.0)

    def test_invalid_mode_raises(self):
        """Test that invalid mode raises ValueError."""
        start = {"X": 0}
        end = {"X": 100}

        with pytest.raises(ValueError, match="mode must be"):
            generate_trajectory(start, end, mode="invalid")

    def test_invalid_space_raises(self):
        """Test that invalid space raises ValueError."""
        start = {"X": 0}
        end = {"X": 100}

        with pytest.raises(ValueError, match="space must be"):
            generate_trajectory(start, end, space="invalid")

    def test_absolute_mode_ignores_include_resets(self):
        """Test that absolute mode ignores include_resets flag."""
        start = {"X": 0}
        end = {"X": 100}

        # Even with include_resets=True, absolute mode should not add resets
        traj = generate_trajectory(start, end, steps=5, space="cartesian", mode="absolute", include_resets=True)

        assert len(traj) == 5  # No extra reset points

    def test_preserves_all_axes(self):
        """Test that all axes from start are preserved in trajectory."""
        start = {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0}
        end = {"X": 10, "Y": 20, "Z": 30, "A": 5, "B": 10, "C": 15}

        traj = generate_trajectory(start, end, steps=2, space="cartesian", mode="absolute")

        # Each point should have all axes
        for point in traj:
            assert set(point.keys()) == {"X", "Y", "Z", "A", "B", "C"}
