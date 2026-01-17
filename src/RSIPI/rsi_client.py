import logging
import multiprocessing
import time
from enum import Enum, auto
from threading import Lock, Thread
from typing import Optional
from .config_parser import ConfigParser
from .network_handler import NetworkProcess
from .safety_manager import SafetyManager
from .exceptions import RSIStateError, RSIInvalidTransition, RSIClientNotReady
from .auto_reconnect import AutoReconnectManager, ReconnectStrategy


class ClientState(Enum):
    """Connection states for RSIClient."""
    INITIALIZED = auto()  # After __init__, network process spawned but not started
    STARTING = auto()     # Start signal sent, waiting for network to be ready
    RUNNING = auto()      # Actively communicating with robot
    STOPPING = auto()     # Shutdown in progress
    STOPPED = auto()      # Fully stopped, cannot be restarted (use reconnect)
    ERROR = auto()        # Error state


class RSIClient:
    """Main RSI API class that integrates network, config handling, and message processing."""

    # Valid state transitions
    _VALID_TRANSITIONS = {
        ClientState.INITIALIZED: {ClientState.STARTING, ClientState.STOPPING},
        ClientState.STARTING: {ClientState.RUNNING, ClientState.STOPPING, ClientState.ERROR},
        ClientState.RUNNING: {ClientState.STOPPING, ClientState.ERROR},
        ClientState.STOPPING: {ClientState.STOPPED, ClientState.ERROR},
        ClientState.STOPPED: {ClientState.INITIALIZED},  # Via reconnect
        ClientState.ERROR: {ClientState.STOPPING, ClientState.INITIALIZED},  # Via reconnect
    }

    def __init__(
        self,
        config_file: str,
        rsi_limits_file: Optional[str] = None,
        enable_auto_reconnect: bool = False,
        auto_reconnect_retries: int = 5,
        auto_reconnect_delay: float = 5.0
    ) -> None:
        """
        Initialize RSI client with configuration and safety limits.

        Args:
            config_file: Path to RSI_EthernetConfig.xml
            rsi_limits_file: Optional path to .rsi.xml safety limits file
            enable_auto_reconnect: Enable automatic reconnection on communication loss
            auto_reconnect_retries: Maximum reconnection attempts (0 = unlimited)
            auto_reconnect_delay: Base delay between retries in seconds
        """
        logging.info(f"Loading RSI configuration from {config_file}...")

        self._state: ClientState = ClientState.INITIALIZED
        self._state_lock: Lock = Lock()

        self.config_parser: ConfigParser = ConfigParser(config_file, rsi_limits_file)
        network_settings = self.config_parser.get_network_settings()

        self.manager: multiprocessing.Manager = multiprocessing.Manager()
        self.send_variables = self.manager.dict(self.config_parser.send_variables)
        self.receive_variables = self.manager.dict(self.config_parser.receive_variables)
        self.stop_event: multiprocessing.Event = multiprocessing.Event()
        self.start_event: multiprocessing.Event = multiprocessing.Event()
        self.command_queue: multiprocessing.Queue = multiprocessing.Queue()

        self.safety_manager: SafetyManager = SafetyManager(self.config_parser.safety_limits)

        # Shared logging state (readable from parent process)
        self._logging_active = multiprocessing.Value('b', False)

        # Shared metrics dictionary (Phase 2)
        self.metrics_dict = self.manager.dict()

        # Create NetworkProcess but don't start communication yet
        self.network_process: NetworkProcess = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event,
            self.command_queue,
            self.metrics_dict
        )
        # Share the logging_active flag
        self.network_process.logging_active = self._logging_active
        self.network_process.start()

        self.logger: Optional[any] = None  # Reserved for future use
        self.running: bool = False
        self.thread: Optional[Thread] = None

        # Auto-reconnect manager (Phase 2)
        self.auto_reconnect_manager: Optional[AutoReconnectManager] = None
        if enable_auto_reconnect:
            self.auto_reconnect_manager = AutoReconnectManager(
                client=self,
                enabled=True,
                max_retries=auto_reconnect_retries,
                retry_delay=auto_reconnect_delay,
                strategy=ReconnectStrategy.LINEAR_BACKOFF
            )
            logging.info("Auto-reconnect enabled")

    @property
    def state(self) -> ClientState:
        """Get current client state (thread-safe)."""
        with self._state_lock:
            return self._state

    def _transition_to(self, new_state: ClientState) -> bool:
        """
        Attempt to transition to a new state.

        Args:
            new_state: Target state to transition to

        Returns:
            True if transition was valid and completed, False otherwise
        """
        with self._state_lock:
            if new_state in self._VALID_TRANSITIONS.get(self._state, set()):
                old_state = self._state
                self._state = new_state
                logging.debug(f"State transition: {old_state.name} -> {new_state.name}")
                return True
            else:
                logging.warning(
                    f"Invalid state transition attempted: {self._state.name} -> {new_state.name}"
                )
                return False

    def start(self) -> None:
        """
        Send start signal to NetworkProcess and run control loop.

        Transitions through STARTING → RUNNING states and maintains
        control loop until stopped.

        Raises:
            RSIClientNotReady: If client is not in appropriate state to start
        """
        if not self._transition_to(ClientState.STARTING):
            error_msg = f"Cannot start from state {self.state.name}"
            logging.error(error_msg)
            raise RSIClientNotReady(error_msg)

        logging.info("RSIClient sending start signal to NetworkProcess...")
        self.start_event.set()

        if not self._transition_to(ClientState.RUNNING):
            error_msg = "Failed to transition to RUNNING state"
            logging.error(error_msg)
            raise RSIStateError(error_msg)

        self.running = True
        logging.info("RSI Client Started")

        # Start auto-reconnect monitor (Phase 2)
        if self.auto_reconnect_manager:
            self.auto_reconnect_manager.start()

        try:
            while self.running and not self.stop_event.is_set():
                time.sleep(2)
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logging.error(f"RSI Client encountered an error: {e}")
            self._transition_to(ClientState.ERROR)
            raise

    def stop(self) -> None:
        """Stop the network process and the client thread safely."""
        if self.state in (ClientState.STOPPED, ClientState.STOPPING):
            logging.debug("Already stopped or stopping")
            return

        if not self._transition_to(ClientState.STOPPING):
            logging.warning("Could not transition to STOPPING state")
            # Continue anyway to ensure cleanup

        logging.info("Stopping RSI Client...")

        self.running = False
        self.stop_event.set()

        if self.network_process and self.network_process.is_alive():
            self.network_process.join(timeout=3)
            if self.network_process.is_alive():
                logging.warning("Forcing network process termination...")
                self.network_process.terminate()
                self.network_process.join()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
            self.thread = None

        # Stop auto-reconnect monitor (Phase 2)
        if self.auto_reconnect_manager:
            self.auto_reconnect_manager.stop()

        self._transition_to(ClientState.STOPPED)
        logging.info("RSI Client Stopped")

    def reconnect(self) -> None:
        """
        Reconnect the network process safely.

        Stops existing connection, resets state, and creates fresh
        network process with new communication resources.
        """
        logging.info("Reconnecting RSI Client network...")

        # Stop if currently running
        if self.state in (ClientState.RUNNING, ClientState.STARTING):
            self.stop()

        if self.network_process and self.network_process.is_alive():
            self.stop_event.set()
            self.network_process.terminate()
            self.network_process.join()

        # Reset to initialized state
        with self._state_lock:
            self._state = ClientState.INITIALIZED

        # Fresh new events and queue
        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()
        self.command_queue = multiprocessing.Queue()

        # Reset metrics dictionary (Phase 2)
        self.metrics_dict.clear()

        # Create new network process
        network_settings = self.config_parser.get_network_settings()
        self.network_process = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event,
            self.command_queue,
            self.metrics_dict
        )
        self.network_process.logging_active = self._logging_active
        self.network_process.start()

        # Fresh control thread
        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

    def is_running(self) -> bool:
        """
        Check if client is in running state.

        Returns:
            True if currently running
        """
        return self.state == ClientState.RUNNING

    def is_stopped(self) -> bool:
        """
        Check if client is fully stopped.

        Returns:
            True if in STOPPED state
        """
        return self.state == ClientState.STOPPED

    def start_logging(self, filename: str) -> None:
        """
        Start CSV logging to the specified file.

        Args:
            filename: Path to output CSV file
        """
        self.command_queue.put({'action': 'start_logging', 'filename': filename})

    def stop_logging(self) -> None:
        """Stop CSV logging."""
        self.command_queue.put({'action': 'stop_logging'})

    def is_logging_active(self) -> bool:
        """
        Check if CSV logging is currently active.

        Returns:
            True if logging is active
        """
        return self._logging_active.value
