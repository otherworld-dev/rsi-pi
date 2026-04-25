"""
RSIPI - Robot Sensor Interface Python Integration

Main API orchestrator providing namespaced access to all RSI functionality.
"""

import logging
from threading import Thread
from typing import Optional, TYPE_CHECKING

from .motion_api import MotionAPI
from .io_api import IOAPI
from .krl_api import KRLAPI
from .safety_api import SafetyAPI
from .monitoring_api import MonitoringAPI
from .logging_api import LoggingAPI
from .diagnostics_api import DiagnosticsAPI
from .viz_api import VizAPI
from .tools_api import ToolsAPI

if TYPE_CHECKING:
    from .rsi_client import RSIClient, ClientState


class RSIAPI:
    """
    High-level API orchestrator for KUKA RSI robot control.

    Supports context manager usage for safe cleanup:
        >>> with RSIAPI('RSI_EthernetConfig.xml') as api:
        ...     api.start()
        ...     api.motion.update_cartesian(X=10)
    """

    def __init__(
        self,
        config_file: str = "RSI_EthernetConfig.xml",
        rsi_mode: str = 'relative',
        max_cartesian_rate: float = 0.0,
        max_joint_rate: float = 0.0,
        cycle_time: float = 0.004
    ) -> None:
        """
        Args:
            config_file: Path to RSI_EthernetConfig.xml
            rsi_mode: 'absolute' or 'relative' — must match KRL RSI_MOVECORR() mode
            max_cartesian_rate: Max mm/cycle for RKorr corrections (0 = no limit)
            max_joint_rate: Max degrees/cycle for AKorr corrections (0 = no limit)
            cycle_time: Expected RSI cycle time in seconds (0.004 = 4ms/250Hz, 0.012 = 12ms/83Hz)
        """
        self.config_file: str = config_file
        self.rsi_mode: str = rsi_mode
        self.max_cartesian_rate: float = max_cartesian_rate
        self.max_joint_rate: float = max_joint_rate
        self.cycle_time: float = cycle_time
        self.client: Optional['RSIClient'] = None
        self._thread: Optional[Thread] = None

        self._ensure_client()

        self.motion = MotionAPI(self.client)
        self.io = IOAPI(self.client)
        self.krl = KRLAPI(self.client)
        self.safety = SafetyAPI(self.client)
        self.monitoring = MonitoringAPI(self.client)
        self.logging = LoggingAPI(self.client)
        self.diagnostics = DiagnosticsAPI(self.client)
        self.viz = VizAPI(self.client)
        self.tools = ToolsAPI(self.client)

        logging.info("RSIAPI initialized with namespaced structure")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.stop()
        except Exception:
            pass
        return False

    def _ensure_client(self) -> None:
        if self.client is None:
            from .rsi_client import RSIClient
            self.client = RSIClient(
                self.config_file,
                rsi_mode=self.rsi_mode,
                max_cartesian_rate=self.max_cartesian_rate,
                max_joint_rate=self.max_joint_rate,
                cycle_time=self.cycle_time
            )

    @property
    def state(self) -> 'ClientState':
        return self.client.state

    def start(self) -> str:
        """Start RSI communication in background thread."""
        self._thread = Thread(target=self.client.start, daemon=True)
        self._thread.start()
        logging.info("RSI communication started in background thread")
        return "RSI started in background"

    def stop(self) -> str:
        """Stop RSI communication gracefully."""
        self.client.stop()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        self._thread = None
        logging.info("RSI communication stopped")
        return "RSI stopped"

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        """
        Block until the robot's first packet is received.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if connected, False if timeout
        """
        return self.client.wait_for_connection(timeout)

    def reconnect(self) -> str:
        """Restart network connection with fresh resources."""
        self.client.reconnect()
        # Start client in new thread
        self._thread = Thread(target=self.client.start, daemon=True)
        self._thread.start()
        logging.info("Network connection restarted")
        return "Network connection restarted"

    def is_running(self) -> bool:
        return self.client.is_running()

    def is_stopped(self) -> bool:
        return self.client.is_stopped()

    # Deprecated methods
    def start_rsi(self) -> str:
        logging.warning("start_rsi() is deprecated. Use api.start() instead.")
        return self.start()

    def stop_rsi(self) -> str:
        logging.warning("stop_rsi() is deprecated. Use api.stop() instead.")
        return self.stop()
