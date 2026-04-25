"""Safety management API namespace for RSIPI."""

import logging
from typing import Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class SafetyAPI:
    """
    Safety management interface for KUKA RSI robot control.

    Provides emergency stop control, limit configuration, and safety status monitoring.
    All limits are enforced by the SafetyManager before values are sent to the robot.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize SafetyAPI namespace.

        Args:
            client: RSIClient instance for accessing safety manager
        """
        self.client = client

    def stop(self) -> None:
        """
        Trigger emergency stop.

        Activates the safety manager's E-stop flag, blocking all motion commands
        until reset() is called. This is a software-level safety mechanism.

        Note:
            This is NOT a hardware E-stop and should not be relied upon for
            safety-critical applications. Always use proper hardware E-stops.
        """
        self.client.emergency_stop()
        logging.critical("Emergency stop activated via SafetyAPI")

    def reset(self) -> None:
        """
        Reset emergency stop and resume normal operation.

        Clears the E-stop flag, allowing motion commands to proceed again.
        Use with caution after ensuring the robot workspace is safe.
        """
        self.client.emergency_reset()
        logging.info("Emergency stop reset via SafetyAPI")

    def status(self) -> Dict[str, Any]:
        """
        Get comprehensive safety status information.

        Returns:
            Dictionary containing:
                - emergency_stop (bool): Whether E-stop is active
                - safety_override (bool): Whether safety checks are bypassed
                - limits (Dict[str, Tuple[float, float]]): Configured limits

        Example:
            >>> status = api.safety.status()
            >>> print(status['emergency_stop'])
            False
            >>> print(status['limits']['RKorr.X'])
            (-100.0, 100.0)
        """
        sm = self.client.safety_manager
        return {
            "emergency_stop": sm.is_stopped(),
            "safety_override": sm.is_safety_overridden(),
            "limits": sm.get_limits(),
        }

    def set_limit(self, variable: str, lower: float, upper: float) -> None:
        """
        Set or update safety limit bounds for a specific variable.

        Args:
            variable: Variable path (e.g., 'RKorr.X', 'AKorr.A1')
            lower: Minimum allowed value
            upper: Maximum allowed value

        Raises:
            ValueError: If lower >= upper

        Example:
            >>> api.safety.set_limit('RKorr.X', -50.0, 50.0)
            >>> api.safety.set_limit('AKorr.A1', -10.0, 10.0)
        """
        if lower >= upper:
            raise ValueError(f"Lower limit ({lower}) must be less than upper limit ({upper})")

        self.client.safety_manager.set_limit(variable, float(lower), float(upper))
        logging.info(f"Safety limit set for {variable}: [{lower}, {upper}]")

    def get_limits(self) -> Dict[str, tuple[float, float]]:
        """
        Get all configured safety limits.

        Returns:
            Dictionary mapping variable paths to (lower, upper) limit tuples

        Example:
            >>> limits = api.safety.get_limits()
            >>> for var, (lower, upper) in limits.items():
            ...     print(f"{var}: [{lower}, {upper}]")
        """
        return self.client.safety_manager.get_limits()

    def override(self, enable: bool) -> None:
        """
        Enable or disable safety limit override.

        WARNING: Use with EXTREME CAUTION. When enabled, all safety limit
        validation is bypassed, allowing potentially dangerous motion commands.

        Args:
            enable: True to bypass safety checks, False to re-enable

        Example:
            >>> # Temporarily disable limits for calibration
            >>> api.safety.override(True)
            >>> # ... perform calibration ...
            >>> api.safety.override(False)  # Re-enable safety
        """
        self.client.safety_manager.override_safety(enable)
        if enable:
            logging.warning("⚠️ SAFETY OVERRIDE ENABLED - All limit checks bypassed!")
        else:
            logging.info("Safety override disabled - limits re-enabled")

    def is_stopped(self) -> bool:
        """
        Check if emergency stop is currently active.

        Returns:
            True if E-stop is active, blocking all motion
        """
        return self.client.safety_manager.is_stopped()

    def is_overridden(self) -> bool:
        """
        Check if safety limit validation is currently bypassed.

        Returns:
            True if safety checks are disabled
        """
        return self.client.safety_manager.is_safety_overridden()
