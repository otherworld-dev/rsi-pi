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

    Provides namespaced access to all RSIPI functionality through specialized
    sub-APIs. This is the main entry point for most users.

    Namespaces:
        - motion: Motion control (Cartesian, joints, trajectories)
        - io: Digital I/O control
        - krl: KRL program manipulation utilities
        - safety: Safety management and limits
        - monitoring: Live data access and monitoring
        - logging: CSV data logging
        - diagnostics: Network and performance diagnostics (Phase 2)
        - viz: Static and live visualization
        - tools: Utilities, debugging, and inspection

    Core Methods (direct access):
        - start(): Start RSI communication
        - stop(): Stop RSI communication
        - reconnect(): Restart network connection
        - state: Current client state (property)

    Example:
        >>> api = RSIAPI('RSI_EthernetConfig.xml')
        >>> api.start()
        >>> api.motion.update_cartesian(X=10, Y=5, Z=0)
        >>> api.logging.start('test.csv')
        >>> # ... robot operation ...
        >>> api.logging.stop()
        >>> api.stop()
        >>> api.viz.plot_static('test.csv', '3d')
    """

    def __init__(self, config_file: str = "RSI_EthernetConfig.xml") -> None:
        """
        Initialize RSIAPI with configuration file.

        Creates RSIClient instance (lazy initialization) and sets up all
        namespace APIs for organized access to functionality.

        Args:
            config_file: Path to RSI_EthernetConfig.xml configuration file

        Example:
            >>> api = RSIAPI('config/RSI_EthernetConfig.xml')
            >>> print(api.state)
            ClientState.INITIALIZED
        """
        self.config_file: str = config_file
        self.client: Optional['RSIClient'] = None
        self._thread: Optional[Thread] = None

        # Initialize client
        self._ensure_client()

        # Initialize namespace APIs
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

    def _ensure_client(self) -> None:
        """
        Ensure RSIClient is initialized (lazy initialization).

        Imports and creates RSIClient only when needed, avoiding circular
        dependencies and improving startup time.
        """
        if self.client is None:
            from .rsi_client import RSIClient
            self.client = RSIClient(self.config_file)
            logging.debug("RSIClient initialized")

    @property
    def state(self) -> 'ClientState':
        """
        Get current client state.

        Returns:
            ClientState enum value (INITIALIZED, STARTING, RUNNING, STOPPING, STOPPED, ERROR)

        Example:
            >>> api = RSIAPI()
            >>> print(api.state)
            ClientState.INITIALIZED
            >>> api.start()
            >>> print(api.state)
            ClientState.RUNNING
        """
        return self.client.state

    def start(self) -> str:
        """
        Start RSI communication in background thread.

        Creates a daemon thread that runs the RSI client's main communication
        loop. The thread handles UDP message exchange with the robot controller.

        Returns:
            Status message

        Raises:
            RSIClientNotReady: If client is not in appropriate state to start

        Example:
            >>> api = RSIAPI()
            >>> api.start()
            'RSI started in background'
            >>> api.state
            ClientState.RUNNING

        Note:
            The background thread runs as a daemon, so it will automatically
            terminate when the main program exits.
        """
        self._thread = Thread(target=self.client.start, daemon=True)
        self._thread.start()
        logging.info("RSI communication started in background thread")
        return "RSI started in background"

    def stop(self) -> str:
        """
        Stop RSI communication.

        Gracefully shuts down the network process, closes sockets, and
        stops any active logging. Waits for background threads to complete.

        Returns:
            Status message

        Example:
            >>> api.stop()
            'RSI stopped'

        Note:
            This method blocks until the network process has fully shut down,
            which typically takes 1-3 seconds.
        """
        self.client.stop()
        logging.info("RSI communication stopped")
        return "RSI stopped"

    def reconnect(self) -> str:
        """
        Restart network connection without stopping RSI client.

        Terminates the current network process, creates a fresh one with new
        communication resources, and restarts the connection. Useful for
        recovering from network errors.

        Returns:
            Status message

        Example:
            >>> # Network issue detected
            >>> api.reconnect()
            'Network connection restarted'

        Note:
            This creates fresh multiprocessing Events and Queues. Any queued
            but unprocessed data in the old network process will be lost.
        """
        self.client.reconnect()
        logging.info("Network connection restarted")
        return "Network connection restarted"

    def is_running(self) -> bool:
        """
        Check if RSI client is in RUNNING state.

        Returns:
            True if actively communicating with robot

        Example:
            >>> api = RSIAPI()
            >>> api.is_running()
            False
            >>> api.start()
            >>> api.is_running()
            True
        """
        return self.client.is_running()

    def is_stopped(self) -> bool:
        """
        Check if RSI client is fully stopped.

        Returns:
            True if in STOPPED state

        Example:
            >>> api.stop()
            >>> api.is_stopped()
            True
        """
        return self.client.is_stopped()

    # Deprecated methods for backward compatibility (Phase 5.1 - to be removed)
    # These are kept temporarily to ease migration. Use namespaced methods instead.

    def start_rsi(self) -> str:
        """DEPRECATED: Use api.start() instead."""
        logging.warning("start_rsi() is deprecated. Use api.start() instead.")
        return self.start()

    def stop_rsi(self) -> str:
        """DEPRECATED: Use api.stop() instead."""
        logging.warning("stop_rsi() is deprecated. Use api.stop() instead.")
        return self.stop()
