"""Monitoring and live data API namespace for RSIPI."""

import logging
import time
import datetime
from typing import Dict, Any, Union, Optional, TYPE_CHECKING
import numpy as np
import pandas as pd

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class MonitoringAPI:
    """
    Real-time monitoring interface for KUKA RSI robot data.

    Provides access to live position, velocity, force, and IPOC data
    in various formats for external processing and analysis.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize MonitoringAPI namespace.

        Args:
            client: RSIClient instance for accessing receive variables
        """
        self.client = client

    def get_live_data(self) -> Dict[str, Any]:
        """
        Retrieve comprehensive real-time RSI data.

        Returns:
            Dictionary containing:
                - position: TCP position (RIst) {X, Y, Z, A, B, C}
                - velocity: TCP velocity {X, Y, Z}
                - acceleration: TCP acceleration {X, Y, Z}
                - force: Joint motor currents (MaCur) {A1-A6}
                - ipoc: Current interrupt point counter

        Example:
            >>> data = api.monitoring.get_live_data()
            >>> print(f"Position: {data['position']}")
            Position: {'X': 600.5, 'Y': -200.3, 'Z': 1450.8, 'A': 0.0, 'B': 0.0, 'C': 0.0}
            >>> print(f"IPOC: {data['ipoc']}")
            IPOC: 123456
        """
        return {
            "position": dict(self.client.send_variables.get("RIst", {"X": 0, "Y": 0, "Z": 0})),
            "velocity": dict(self.client.send_variables.get("Velocity", {"X": 0, "Y": 0, "Z": 0})),
            "acceleration": dict(self.client.send_variables.get("Acceleration", {"X": 0, "Y": 0, "Z": 0})),
            "force": dict(self.client.send_variables.get("MaCur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0})),
            "ipoc": self.client.send_variables.get("IPOC", "N/A")
        }

    def get_live_data_as_numpy(self) -> np.ndarray:
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

    def get_live_data_as_dataframe(self) -> pd.DataFrame:
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
        return dict(self.client.send_variables.get("MaCur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0}))

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
                ipoc = live_data.get("IPOC", "N/A")
                rpos = live_data.get("position", {})
                timestamp = datetime.datetime.now().strftime('%H:%M:%S')
                print(f"[{timestamp}] IPOC: {ipoc} | RIst: {rpos}")
                time.sleep(rate)

                if duration and (time.time() - start_time) >= duration:
                    logging.info("Network watch duration completed.")
                    break

        except KeyboardInterrupt:
            logging.info("\nStopped network watch.")
