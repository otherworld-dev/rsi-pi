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
from .exceptions import RSIStateError

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

    #: How long start() waits before deciding the client thread died.
    _START_CHECK_SECONDS = 0.5

    def __init__(
        self,
        config_file: str,
        rsi_mode: str = 'relative',
        max_cartesian_rate: float = 0.0,
        max_joint_rate: float = 0.0,
        cycle_time: float = 0.004,
        rsi_limits_file: Optional[str] = None,
        enable_auto_reconnect: bool = False,
        auto_reconnect_retries: int = 5,
        auto_reconnect_delay: float = 5.0,
    ) -> None:
        """
        Args:
            config_file: Path to the RSI Ethernet config XML. Required, and it
                must be the SAME file the controller's ETHERNET object loads —
                the two ends have to agree on the telegram structure. There is
                deliberately no default: a relative one would resolve against
                the working directory and could silently pick up a config that
                does not match the robot.
            rsi_mode: 'absolute' or 'relative' — must match KRL RSI_MOVECORR() mode
            max_cartesian_rate: Max mm/cycle for RKorr corrections (0 = no limit)
            max_joint_rate: Max degrees/cycle for AKorr corrections (0 = no limit)
            cycle_time: Expected RSI cycle time in seconds (0.004 = 4ms/250Hz, 0.012 = 12ms/83Hz)
            rsi_limits_file: Optional path to .rsi.xml safety limits file
            enable_auto_reconnect: Enable automatic reconnection on communication loss
            auto_reconnect_retries: Maximum reconnection attempts (0 = unlimited)
            auto_reconnect_delay: Base delay between retries in seconds
        """
        self.config_file: str = config_file
        self.rsi_mode: str = rsi_mode
        self.max_cartesian_rate: float = max_cartesian_rate
        self.max_joint_rate: float = max_joint_rate
        self.cycle_time: float = cycle_time
        self.rsi_limits_file: Optional[str] = rsi_limits_file
        self.enable_auto_reconnect: bool = enable_auto_reconnect
        self.auto_reconnect_retries: int = auto_reconnect_retries
        self.auto_reconnect_delay: float = auto_reconnect_delay
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
                rsi_limits_file=self.rsi_limits_file,
                enable_auto_reconnect=self.enable_auto_reconnect,
                auto_reconnect_retries=self.auto_reconnect_retries,
                auto_reconnect_delay=self.auto_reconnect_delay,
                rsi_mode=self.rsi_mode,
                max_cartesian_rate=self.max_cartesian_rate,
                max_joint_rate=self.max_joint_rate,
                cycle_time=self.cycle_time
            )

    @property
    def state(self) -> 'ClientState':
        return self.client.state

    def start(self) -> str:
        """Start RSI communication in a background thread.

        Raises whatever the client raised if it failed to start, rather than
        reporting success and leaving the caller to discover it much later.
        A bad config, a busy UDP port or a bad state transition all surface
        here; without this, wait_for_connection() would sit for its full
        timeout and then blame the robot for a fault on this side.
        """
        self._thread_error: Optional[BaseException] = None

        def _run() -> None:
            try:
                self.client.start()
            except BaseException as exc:            # noqa: BLE001 - re-raised below
                self._thread_error = exc
                logging.exception("RSI client thread failed during start")

        self._thread = Thread(target=_run, daemon=True)
        # Hand the control thread to the client so client.stop() joins it too.
        self.client.thread = self._thread
        self._thread.start()

        # client.start() blocks in its control loop while healthy, so a thread
        # that has already finished did not survive start-up.
        self._thread.join(self._START_CHECK_SECONDS)
        if self._thread_error is not None:
            raise self._thread_error
        if not self._thread.is_alive():
            raise RSIStateError(
                "The RSI client thread exited immediately after start with no "
                "error - check the client state and the config.")

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
        # RSIClient.reconnect() restarts the control loop itself; spawning a
        # second thread here would double-start and raise RSIClientNotReady.
        self.client.reconnect()
        self._thread = self.client.thread
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
