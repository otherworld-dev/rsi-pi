"""Tests for SafetyManager."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.safety_manager import SafetyManager


class TestValidate:
    """Tests for SafetyManager.validate()"""

    def test_validate_within_limits(self):
        """Test that values within limits pass through unchanged."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        result = sm.validate("RKorr.X", 3.0)
        assert result == 3.0

    def test_validate_at_boundary(self):
        """Test that values at exact boundaries pass."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        assert sm.validate("RKorr.X", -5.0) == -5.0
        assert sm.validate("RKorr.X", 5.0) == 5.0

    def test_validate_exceeds_max(self):
        """Test that values exceeding max raise ValueError."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        with pytest.raises(ValueError, match="out of bounds"):
            sm.validate("RKorr.X", 5.1)

    def test_validate_below_min(self):
        """Test that values below min raise ValueError."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        with pytest.raises(ValueError, match="out of bounds"):
            sm.validate("RKorr.X", -5.1)

    def test_validate_unlisted_path(self):
        """Test that paths without defined limits pass through."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        # RKorr.Y has no limit defined - should pass
        result = sm.validate("RKorr.Y", 1000.0)
        assert result == 1000.0

    def test_validate_with_override(self):
        """Test that override bypasses all limit checks."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)
        sm.override_safety(True)

        # Should pass despite being out of bounds
        result = sm.validate("RKorr.X", 100.0)
        assert result == 100.0


class TestEmergencyStop:
    """Tests for emergency stop functionality."""

    def test_estop_blocks_validation(self):
        """Test that e-stop blocks all validation."""
        sm = SafetyManager()
        sm.emergency_stop()

        with pytest.raises(RuntimeError, match="E-STOP"):
            sm.validate("RKorr.X", 0.0)

    def test_estop_reset(self):
        """Test that e-stop can be reset."""
        sm = SafetyManager()
        sm.emergency_stop()

        assert sm.is_stopped() is True

        sm.reset_stop()

        assert sm.is_stopped() is False
        # Should work again
        result = sm.validate("RKorr.X", 1.0)
        assert result == 1.0


class TestSetLimit:
    """Tests for runtime limit modification."""

    def test_set_new_limit(self):
        """Test adding a new limit at runtime."""
        sm = SafetyManager()

        sm.set_limit("RKorr.Y", -10.0, 10.0)

        assert sm.validate("RKorr.Y", 5.0) == 5.0
        with pytest.raises(ValueError):
            sm.validate("RKorr.Y", 15.0)

    def test_override_existing_limit(self):
        """Test overriding an existing limit."""
        limits = {"RKorr.X": (-5.0, 5.0)}
        sm = SafetyManager(limits)

        # Original limit blocks this
        with pytest.raises(ValueError):
            sm.validate("RKorr.X", 8.0)

        # Override limit
        sm.set_limit("RKorr.X", -10.0, 10.0)

        # Now it should pass
        assert sm.validate("RKorr.X", 8.0) == 8.0

    def test_get_limits(self):
        """Test retrieving all limits."""
        limits = {"RKorr.X": (-5.0, 5.0), "AKorr.A1": (-6.0, 6.0)}
        sm = SafetyManager(limits)

        retrieved = sm.get_limits()
        assert retrieved == limits
        # Should be a copy
        retrieved["new"] = (0, 1)
        assert "new" not in sm.get_limits()


class TestStaticChecks:
    """Tests for static limit checking methods."""

    def test_cartesian_limits_valid(self):
        """Test valid Cartesian pose passes check."""
        pose = {"X": 500, "Y": -200, "Z": 1000, "A": 0, "B": 0, "C": 0}
        assert SafetyManager.check_cartesian_limits(pose) is True

    def test_cartesian_limits_z_too_low(self):
        """Test Z below zero fails."""
        pose = {"X": 0, "Y": 0, "Z": -100}
        assert SafetyManager.check_cartesian_limits(pose) is False

    def test_cartesian_limits_x_too_high(self):
        """Test X exceeding max fails."""
        pose = {"X": 2000, "Y": 0, "Z": 500}
        assert SafetyManager.check_cartesian_limits(pose) is False

    def test_cartesian_limits_partial_pose(self):
        """Test partial pose (missing keys) passes if present keys are valid."""
        pose = {"X": 100, "Z": 500}  # Missing Y, A, B, C
        assert SafetyManager.check_cartesian_limits(pose) is True

    def test_joint_limits_valid(self):
        """Test valid joint pose passes check."""
        pose = {"A1": 0, "A2": -45, "A3": 90, "A4": 0, "A5": 0, "A6": 180}
        assert SafetyManager.check_joint_limits(pose) is True

    def test_joint_limits_a1_exceeded(self):
        """Test A1 exceeding limit fails."""
        pose = {"A1": 200}  # Limit is -185 to 185
        assert SafetyManager.check_joint_limits(pose) is False

    def test_joint_limits_a5_exceeded(self):
        """Test A5 exceeding its tighter limit fails."""
        pose = {"A5": 150}  # Limit is -130 to 130
        assert SafetyManager.check_joint_limits(pose) is False
