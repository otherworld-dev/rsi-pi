import logging
from typing import Dict, Tuple, Optional
from .exceptions import RSISafetyViolation, RSIEmergencyStop, RSILimitExceeded


class SafetyManager:
    """
    Enforces safety limits for RSI motion commands.

    Supports:
    - Emergency stop logic (halts all validation)
    - Limit enforcement for RKorr / AKorr / other variables
    - Runtime limit updates
    """

    def __init__(self, limits: Optional[Dict[str, Tuple[float, float]]] = None) -> None:
        """
        Initialize SafetyManager with optional safety limits.

        Args:
            limits: Optional safety limits in the form:
                {
                    'RKorr.X': (-5.0, 5.0),
                    'AKorr.A1': (-6.0, 6.0),
                    ...
                }
        """
        self.limits: Dict[str, Tuple[float, float]] = limits if limits is not None else {}
        self.e_stop: bool = False
        self.last_values: Dict[str, float] = {}  # Reserved for future tracking or override detection
        self.override: bool = False  # Track if safety checks are overridden

    def validate(self, path: str, value: float) -> float:
        """
        Validate a value against safety limits and emergency stop state.

        Args:
            path: Variable path (e.g., 'RKorr.X', 'AKorr.A1')
            value: Value to validate

        Returns:
            Validated value (unchanged if valid)

        Raises:
            RSIEmergencyStop: If emergency stop is active
            RSILimitExceeded: If value exceeds configured limits
        """
        if self.override:
            # Bypass all safety checks when override is active
            return value

        if self.e_stop:
            logging.warning(f"SafetyManager: {path} update blocked (E-STOP active)")
            raise RSIEmergencyStop(f"E-STOP active. Motion blocked for {path}.")

        if path in self.limits:
            min_val, max_val = self.limits[path]
            if not (min_val <= value <= max_val):
                logging.warning(f"SafetyManager: {path}={value} blocked (out of bounds {min_val} to {max_val})")
                raise RSILimitExceeded(f"{path}={value} is out of bounds ({min_val} to {max_val})")

        return value

    def emergency_stop(self) -> None:
        """Activate emergency stop: all motion validation will fail."""
        self.e_stop = True
        logging.critical("Emergency stop activated")

    def reset_stop(self) -> None:
        """Reset emergency stop, allowing motion again."""
        self.e_stop = False
        logging.info("Emergency stop reset")

    def set_limit(self, path: str, min_val: float, max_val: float) -> None:
        """
        Set or override a safety limit at runtime.

        Args:
            path: Variable path (e.g., 'RKorr.X')
            min_val: Minimum allowed value
            max_val: Maximum allowed value
        """
        self.limits[path] = (min_val, max_val)
        logging.info(f"Safety limit updated: {path} = ({min_val}, {max_val})")

    def get_limits(self) -> Dict[str, Tuple[float, float]]:
        """
        Get a copy of all current safety limits.

        Returns:
            Dictionary mapping variable paths to (min, max) tuples
        """
        return self.limits.copy()

    def is_stopped(self) -> bool:
        """
        Check if emergency stop is active.

        Returns:
            True if emergency stop is active
        """
        return self.e_stop

    def override_safety(self, enable: bool) -> None:
        """
        Enable or disable safety override (bypass all checks).

        Args:
            enable: True to enable override, False to disable

        Warning:
            Use with extreme caution. All safety checks are bypassed when enabled.
        """
        self.override = enable
        if enable:
            logging.warning("⚠️  SAFETY OVERRIDE ENABLED - All safety checks bypassed!")
        else:
            logging.info("Safety override disabled")

    def is_safety_overridden(self) -> bool:
        """
        Check if safety override is active.

        Returns:
            True if safety checks are bypassed
        """
        return self.override

    @staticmethod
    def check_cartesian_limits(pose: Dict[str, float]) -> bool:
        """
        Check if a Cartesian pose is within general robot limits.

        Typical bounds: ±1500 mm in XYZ, ±360° in orientation.

        Args:
            pose: Dictionary with keys like 'X', 'Y', 'Z', 'A', 'B', 'C'

        Returns:
            True if pose is within limits, False otherwise
        """
        limits = {
            "X": (-1500, 1500),
            "Y": (-1500, 1500),
            "Z": (0, 2000),
            "A": (-360, 360),
            "B": (-360, 360),
            "C": (-360, 360),
        }
        for key, (lo, hi) in limits.items():
            if key in pose and not (lo <= pose[key] <= hi):
                return False
        return True

    @staticmethod
    def check_joint_limits(pose: Dict[str, float]) -> bool:
        """
        Check if a joint-space pose is within KUKA limits.

        Typical KUKA ranges: A1–A6 in defined degrees.

        Args:
            pose: Dictionary with keys like 'A1', 'A2', ..., 'A6'

        Returns:
            True if pose is within limits, False otherwise
        """
        limits = {
            "A1": (-185, 185),
            "A2": (-185, 185),
            "A3": (-185, 185),
            "A4": (-350, 350),
            "A5": (-130, 130),
            "A6": (-350, 350),
        }
        for key, (lo, hi) in limits.items():
            if key in pose and not (lo <= pose[key] <= hi):
                return False
        return True