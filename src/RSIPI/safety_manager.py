import logging


class SafetyManager:
    """
    Enforces safety limits for RSI motion commands.

    Supports:
    - Emergency stop logic (halts all validation)
    - Limit enforcement for RKorr / AKorr / other variables
    - Runtime limit updates
    """

    def __init__(self, limits=None):
        """
        Args:
            limits (dict): Optional safety limits in the form:
                {
                    'RKorr.X': (-5.0, 5.0),
                    'AKorr.A1': (-6.0, 6.0),
                    ...
                }
        """
        self.limits = limits if limits is not None else {}
        self.e_stop = False
        self.last_values = {}  # Reserved for future tracking or override detection
        self.override = False  # ➡️ Track if safety checks are overridden

    def validate(self, path: str, value: float) -> float:
        if self.override:
            # Bypass all safety checks when override is active
            return value

        if self.e_stop:
            logging.warning(f"SafetyManager: {path} update blocked (E-STOP active)")
            raise RuntimeError(f"SafetyManager: E-STOP active. Motion blocked for {path}.")

        if path in self.limits:
            min_val, max_val = self.limits[path]
            if not (min_val <= value <= max_val):
                logging.warning(f"SafetyManager: {path}={value} blocked (out of bounds {min_val} to {max_val})")
                raise ValueError(f"SafetyManager: {path}={value} is out of bounds ({min_val} to {max_val})")

        return value

    def emergency_stop(self):
        """Activates emergency stop: all motion validation will fail."""
        self.e_stop = True

    def reset_stop(self):
        """Resets emergency stop, allowing motion again."""
        self.e_stop = False

    def set_limit(self, path: str, min_val: float, max_val: float):
        """Sets or overrides a safety limit at runtime."""
        self.limits[path] = (min_val, max_val)

    def get_limits(self):
        """Returns a copy of all current safety limits."""
        return self.limits.copy()

    def is_stopped(self):
        """Returns whether the emergency stop is active."""
        return self.e_stop

    def override_safety(self, enable: bool):
        """Enable or disable safety override (bypass all checks)."""
        self.override = enable

    def is_safety_overridden(self) -> bool:
        """Returns whether safety override is active."""
        return self.override

    @staticmethod
    def check_cartesian_limits(pose):
        """
        Check if a Cartesian pose is within general robot limits.
        Typical bounds: ±1500 mm in XYZ, ±360° in orientation.
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
    def check_joint_limits(pose):
        """
        Check if a joint-space pose is within KUKA limits.
        Typical KUKA ranges: A1–A6 in defined degrees.
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