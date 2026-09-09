"""Monitoring and live data API namespace for RSIPI."""

import logging
import time
import datetime
from typing import Dict, Any, Union, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class MonitoringAPI:
    """
    Real-time monitoring interface for KUKA RSI robot data.

    Provides access to live position, velocity, force, and IPOC data
    in various formats for external processing and analysis.
    """

    _AXES = ("X", "Y", "Z")

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize MonitoringAPI namespace.

        Args:
            client: RSIClient instance for accessing receive variables
        """
        self.client = client
        # Previous sample for derived velocity/acceleration:
        # (ipoc, monotonic seconds, position dict, velocity dict)
        self._prev: Optional[tuple] = None

    def _derive_motion(self, position: Dict[str, float],
                       ipoc: Any) -> Dict[str, Dict[str, float]]:
        """
        Differentiate position into velocity and acceleration.

        RSI has no velocity or acceleration keyword - the controller only
        reports position - so these are derived here from successive
        samples. The robot's own IPOC clock provides the time base when it
        is available (it is a millisecond stamp, immune to scheduling
        jitter on this side); otherwise a monotonic clock is used.

        Accuracy notes:
        - Resolution is bounded by the ETHERNET object's Precision
          parameter (default 1 = 0.1 mm), so slow motion quantises badly.
        - The interval is the gap between *calls*, not the robot cycle, so
          call at a steady rate for meaningful numbers. A single call after
          a long pause reports the average over that pause.
        - Acceleration is a second difference and is correspondingly noisy;
          treat it as indicative.

        For per-cycle fidelity, wire POSACT into D (differentiator) objects
        in the RSI context and declare the results as SEND channels - then
        the values arrive as ordinary variables and this fallback is unused.

        Returns:
            {"velocity": {X, Y, Z} mm/s, "acceleration": {X, Y, Z} mm/s^2}
        """
        zero = {axis: 0.0 for axis in self._AXES}
        now = time.monotonic()
        try:
            ipoc_val = int(ipoc)
        except (TypeError, ValueError):
            ipoc_val = None

        prev = self._prev
        self._prev = (ipoc_val, now, dict(position), dict(zero))

        if prev is None:
            return {"velocity": dict(zero), "acceleration": dict(zero)}

        prev_ipoc, prev_time, prev_pos, prev_vel = prev
        if ipoc_val is not None and prev_ipoc is not None and ipoc_val > prev_ipoc:
            dt = (ipoc_val - prev_ipoc) / 1000.0      # IPOC is milliseconds
        else:
            dt = now - prev_time
        if dt <= 0:
            self._prev = prev                          # nothing usable; keep the old sample
            return {"velocity": dict(prev_vel), "acceleration": dict(zero)}

        velocity, acceleration = {}, {}
        for axis in self._AXES:
            try:
                delta = float(position.get(axis, 0.0)) - float(prev_pos.get(axis, 0.0))
            except (TypeError, ValueError):
                delta = 0.0
            velocity[axis] = delta / dt
            acceleration[axis] = (velocity[axis] - float(prev_vel.get(axis, 0.0))) / dt

        self._prev = (ipoc_val, now, dict(position), velocity)
        return {"velocity": velocity, "acceleration": acceleration}

    def get_live_data(self) -> Dict[str, Any]:
        """
        Retrieve comprehensive real-time RSI data.

        Returns:
            Dictionary containing:
                - position: TCP position (RIst) {X, Y, Z, A, B, C}
                - velocity: TCP velocity {X, Y, Z} in mm/s
                - acceleration: TCP acceleration {X, Y, Z} in mm/s^2
                - force: Joint motor currents (MACur) {A1-A6}
                - ipoc: Current interrupt point counter

        Velocity and acceleration are taken from `Velocity`/`Acceleration`
        SEND variables when the config declares them (e.g. POSACT wired
        through D objects in the RSI context); otherwise they are derived
        from successive position samples - see _derive_motion() for the
        accuracy caveats.

        Example:
            >>> data = api.monitoring.get_live_data()
            >>> print(f"Position: {data['position']}")
            Position: {'X': 600.5, 'Y': -200.3, 'Z': 1450.8, 'A': 0.0, 'B': 0.0, 'C': 0.0}
            >>> print(f"IPOC: {data['ipoc']}")
            IPOC: 123456
        """
        send = self.client.send_variables
        position = dict(send.get("RIst", {"X": 0, "Y": 0, "Z": 0}))
        ipoc = send.get("IPOC", "N/A")

        derived = self._derive_motion(position, ipoc)
        velocity = send.get("Velocity")
        acceleration = send.get("Acceleration")

        return {
            "position": position,
            "velocity": dict(velocity) if isinstance(velocity, dict) else derived["velocity"],
            "acceleration": (dict(acceleration) if isinstance(acceleration, dict)
                             else derived["acceleration"]),
            "force": dict(send.get("MACur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0})),
            "ipoc": ipoc,
        }

    def get_velocity(self) -> Dict[str, float]:
        """
        Current TCP velocity in mm/s.

        Returns:
            {X, Y, Z} in mm/s - from the config's Velocity variable if it
            declares one, otherwise differentiated from position.
        """
        return self.get_live_data()["velocity"]

    def get_acceleration(self) -> Dict[str, float]:
        """
        Current TCP acceleration in mm/s^2.

        Returns:
            {X, Y, Z} in mm/s^2 - a second difference of position unless the
            config declares an Acceleration variable, so treat as indicative.
        """
        return self.get_live_data()["acceleration"]

    def get_live_data_as_numpy(self) -> "np.ndarray":
        """
        Retrieve live RSI data as a NumPy array.

        Returns 2D array with rows: [position, velocity, acceleration, force]
        and columns padded to max length (6 for force axes).

        Returns:
            NumPy array (4 x max_length) with robot state data

        Example:
            >>> arr = api.monitoring.get_live_data_as_numpy()
            >>> print(arr.shape)
            (4, 6)
            >>> print(arr[0])  # Position row
            [600.5 -200.3 1450.8 0.0 0.0 0.0]
        """
        import numpy as np

        data = self.get_live_data()
        flat = []

        for section in ["position", "velocity", "acceleration", "force"]:
            values = list(data[section].values())
            flat.append(values)

        # Pad to uniform length
        max_len = max(len(row) for row in flat)
        for row in flat:
            row.extend([0.0] * (max_len - len(row)))

        return np.array(flat, dtype=np.float64)

    def get_live_data_as_dataframe(self) -> "pd.DataFrame":
        """
        Retrieve live RSI data as a Pandas DataFrame.

        Returns:
            DataFrame with single row containing current robot state

        Example:
            >>> df = api.monitoring.get_live_data_as_dataframe()
            >>> print(df.columns)
            Index(['position', 'velocity', 'acceleration', 'force', 'ipoc'])
            >>> print(df['ipoc'][0])
            123456
        """
        import pandas as pd

        data = self.get_live_data()
        return pd.DataFrame([data])

    def get_ipoc(self) -> Union[int, str]:
        """
        Get current IPOC (Interrupt Point Counter) value.

        The IPOC increments with each RSI cycle (typically every 4ms) and
        is used for synchronization between client and controller.

        Returns:
            Current IPOC value, or "N/A" if not available

        Example:
            >>> ipoc = api.monitoring.get_ipoc()
            >>> print(ipoc)
            123456
        """
        return self.client.send_variables.get("IPOC", "N/A")

    def get_position(self) -> Dict[str, float]:
        """
        Get current TCP position in Cartesian coordinates.

        Returns:
            Dictionary with X, Y, Z (mm) and A, B, C (degrees) orientation

        Example:
            >>> pos = api.monitoring.get_position()
            >>> print(f"TCP at X={pos['X']}, Y={pos['Y']}, Z={pos['Z']}")
            TCP at X=600.5, Y=-200.3, Z=1450.8
        """
        return dict(self.client.send_variables.get("RIst", {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0}))

    def get_force(self) -> Dict[str, float]:
        """
        Get current motor currents for all joints.

        Motor current is a proxy for force/torque applied at each joint.
        Units depend on robot model and configuration.

        Returns:
            Dictionary with A1-A6 motor current values

        Example:
            >>> force = api.monitoring.get_force()
            >>> print(f"Joint A1 current: {force['A1']}")
            Joint A1 current: 12.5
        """
        return dict(self.client.send_variables.get("MACur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0}))

    def get_applied_correction(self) -> Dict[str, float]:
        """
        Cartesian correction the controller has actually applied (POSCORRMON).

        This is the answer to "did the robot do what I asked?", which the
        commanded value cannot give you. A STOP object once silently stopped
        the controller applying any correction at all while RSI kept running
        perfectly - full packet rate, zero late packets, no error - and the
        only symptom was a pose that never changed. POSCORRMON would have
        shown that immediately.

        Needs a context wiring POSCORRMON (``max``); returns an empty dict
        otherwise.

        Returns:
            X, Y, Z (mm) and A, B, C (degrees) of applied correction

        Example:
            >>> api.motion.update_cartesian(X=5.0)
            >>> api.monitoring.get_applied_correction()
            {'X': 4.9, 'Y': 0.0, 'Z': 0.0, 'A': 0.0, 'B': 0.0, 'C': 0.0}
        """
        return dict(self.client.send_variables.get("PosCorrMon", {}))

    def get_applied_joint_correction(self) -> Dict[str, float]:
        """
        Joint correction the controller has actually applied (AXISCORRMON).

        The per-axis counterpart of :meth:`get_applied_correction`. Needs a
        context wiring AXISCORRMON (``max``); returns an empty dict otherwise.

        Returns:
            A1-A6 applied correction in degrees

        Example:
            >>> api.monitoring.get_applied_joint_correction()['A6']
            0.02
        """
        return dict(self.client.send_variables.get("AxisCorrMon", {}))

    #: POSCORR's Stat output, bit-coded above 1. B0 is always set while a
    #: correction is active; the rest name the limit being hit.
    CORRECTION_LIMIT_FLAGS = {
        2: "lower X", 4: "lower Y", 8: "lower Z",
        16: "upper X", 32: "upper Y", 64: "upper Z",
        128: "max rotation angle",
    }

    def get_correction_limit_status(self, raw: bool = False):
        """
        Is the controller clamping the Cartesian correction, and where?

        `POSCORR` limits the **cumulative** correction and, per the RSI
        reference, "if an input exceeds the valid range, the corresponding
        maximum value is used" - it clamps and reports nothing. Its `Stat`
        output is the only signal that this is happening, which is why a move
        can stop dead at exactly the limit with no error anywhere.

        Needs a context wiring `POSCORR`'s `Stat` output (`RSIPI_Max`);
        returns None otherwise.

        Args:
            raw: return the integer instead of the decoded dict

        Returns:
            ``{"active": bool, "limited": bool, "at_limit": [str, ...],
            "raw": int}``, or the int if *raw*, or None if not wired.

        Example:
            >>> api.monitoring.get_correction_limit_status()
            {'active': True, 'limited': True, 'at_limit': ['upper X'], 'raw': 17}
        """
        value = self.client.send_variables.get("PosCorrStat")
        if value is None:
            return None
        value = int(float(value))
        if raw:
            return value
        return {
            "active": bool(value & 1),
            "limited": value > 1,
            "at_limit": [name for bit, name in self.CORRECTION_LIMIT_FLAGS.items()
                         if value & bit],
            "raw": value,
        }

    #: STATUS object value tables, from the RSI element reference. The keys are
    #: the object's `Type` parameter; the robot returns one integer per object.
    STATUS_MEANINGS: Dict[str, Dict[int, str]] = {
        "Sensor": {0: "OFF", 1: "PRE_INIT", 2: "INIT", 3: "CYCLE", 4: "FREEZE",
                   5: "FREEZE_IGNORE", 6: "TERMINATE", 7: "TERMINATE_IGNORE",
                   8: "CLEAR_OFFSETS", 9: "ERROR"},
        "Mode_Op": {1: "T1", 2: "T2", 3: "AUT", 4: "EXT"},
        "ProState": {1: "FREE", 2: "RESET", 3: "ACTIVE", 4: "STOP", 5: "END"},
        "Pro_Mode": {1: "ISTEP", 2: "MSTEP", 3: "PSTEP", 4: "CSTEP",
                     5: "BSTEP", 6: "GO"},
        "IPO_Mode": {1: "Base", 2: "TCP"},
    }

    #: IPO_State is a bit field, not an enumeration.
    IPO_STATE_FLAGS = {1: "ACTIVE", 2: "CONTINUE", 4: "STOP", 8: "FSTOP",
                       16: "GSTOP", 32: "GSTOP_MOV", 64: "CP", 128: "SMOOTH"}

    def get_robot_status(self, index: int = 1,
                         meaning: Optional[str] = None) -> Optional[Any]:
        """
        Read a STATUS object's value, optionally decoded.

        A STATUS object reports one controller status chosen by its `Type`
        parameter, so a context may wire several. They appear as ``Status1``,
        ``Status2`` and so on.

        No shipped context includes a STATUS object - its `Type` is an enum
        whose numeric value must come from RSIVisual rather than be guessed -
        so this returns None unless you have added one.

        Args:
            index: which Status<N> channel to read
            meaning: decode the number using one of STATUS_MEANINGS, e.g.
                "Sensor" or "Mode_Op". Omit for the raw integer.

        Returns:
            The raw int, the decoded string, or None if not wired.

        Example:
            >>> api.monitoring.get_robot_status(1, "Sensor")
            'CYCLE'
            >>> api.monitoring.get_robot_status(2, "Mode_Op")
            'T1'
        """
        raw = self.client.send_variables.get(f"Status{index}")
        if raw is None:
            return None
        value = int(float(raw))
        if meaning is None:
            return value
        if meaning == "IPO_State":
            names = [n for bit, n in self.IPO_STATE_FLAGS.items() if value & bit]
            return "|".join(names) if names else "NONE"
        table = self.STATUS_MEANINGS.get(meaning, {})
        return table.get(value, f"UNKNOWN({value})")

    def get_override(self) -> Optional[int]:
        """
        Program override ($OV_PRO) as a percentage, or None if not wired.

        Needs a context with an OV_PRO object (``max``).

        Example:
            >>> api.monitoring.get_override()
            100
        """
        value = self.client.send_variables.get("OvPro")
        return None if value is None else int(float(value))

    def set_override(self, percent: int) -> str:
        """
        Set the program override ($OV_PRO) from Python.

        Slows or speeds the robot's programmed motion while RSI runs, which
        is a gentler lever than an E-stop when something looks wrong.

        Args:
            percent: 1-100. Values outside that are rejected rather than
                clamped - a silently clamped speed request is the kind of
                thing you only notice on the robot.

        Raises:
            RSIVariableError: if the config declares no OvProW channel, i.e.
                the context has no MAP2OV_PRO object
            ValueError: if percent is outside 1-100

        Example:
            >>> api.monitoring.set_override(30)
            'Override set to 30%'
        """
        from .exceptions import RSIVariableError

        if not 1 <= int(percent) <= 100:
            raise ValueError(f"Override must be 1-100%, got {percent}")
        if "OvProW" not in self.client.receive_variables:
            raise RSIVariableError(
                "This config declares no 'OvProW' channel - setting the "
                "override needs a context with a MAP2OV_PRO object, e.g. "
                "context('max')")

        from .tools_api import ToolsAPI
        ToolsAPI(self.client).update_variable("OvProW", int(percent))
        return f"Override set to {int(percent)}%"

    def watch_network(self, duration: Optional[float] = None, rate: float = 0.2) -> None:
        """
        Continuously print live position and IPOC data to console.

        Useful for monitoring network communication health and robot movement
        during testing and debugging.

        Args:
            duration: Watch duration in seconds (None = until Ctrl+C)
            rate: Update rate in seconds (default: 0.2 = 5 Hz)

        Example:
            >>> # Watch for 10 seconds at 5Hz
            >>> api.monitoring.watch_network(duration=10)
            [14:32:01] IPOC: 123456 | RIst: {'X': 600.5, 'Y': -200.3, 'Z': 1450.8}
            [14:32:01] IPOC: 123506 | RIst: {'X': 600.6, 'Y': -200.3, 'Z': 1450.8}
            ...

            >>> # Watch indefinitely (Ctrl+C to stop)
            >>> api.monitoring.watch_network()
        """
        logging.info("Watching network... Press Ctrl+C to stop.\n")
        start_time = time.time()

        try:
            while True:
                live_data = self.get_live_data()
                ipoc = live_data.get("ipoc", "N/A")
                rpos = live_data.get("position", {})
                timestamp = datetime.datetime.now().strftime('%H:%M:%S')
                print(f"[{timestamp}] IPOC: {ipoc} | RIst: {rpos}")
                time.sleep(rate)

                if duration and (time.time() - start_time) >= duration:
                    logging.info("Network watch duration completed.")
                    break

        except KeyboardInterrupt:
            logging.info("\nStopped network watch.")
