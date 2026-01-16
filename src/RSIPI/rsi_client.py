import logging
import multiprocessing
import time
from enum import Enum, auto
from threading import Lock
from .config_parser import ConfigParser
from .network_handler import NetworkProcess
from .safety_manager import SafetyManager
import threading


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

    def __init__(self, config_file, rsi_limits_file=None):
        logging.info(f"Loading RSI configuration from {config_file}...")

        self._state = ClientState.INITIALIZED
        self._state_lock = Lock()

        self.config_parser = ConfigParser(config_file, rsi_limits_file)
        network_settings = self.config_parser.get_network_settings()

        self.manager = multiprocessing.Manager()
        self.send_variables = self.manager.dict(self.config_parser.send_variables)
        self.receive_variables = self.manager.dict(self.config_parser.receive_variables)
        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()
        self.command_queue = multiprocessing.Queue()

        self.safety_manager = SafetyManager(self.config_parser.safety_limits)

        # Shared logging state (readable from parent process)
        self._logging_active = multiprocessing.Value('b', False)

        # Create NetworkProcess but don't start communication yet
        self.network_process = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event,
            self.command_queue
        )
        # Share the logging_active flag
        self.network_process.logging_active = self._logging_active
        self.network_process.start()

        self.logger = None
        self.running = False
        self.thread = None

    @property
    def state(self) -> ClientState:
        """Get current client state (thread-safe)."""
        with self._state_lock:
            return self._state

    def _transition_to(self, new_state: ClientState) -> bool:
        """
        Attempt to transition to a new state.

        Returns:
            True if transition was valid and completed, False otherwise.
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

    def start(self):
        """Send start signal to NetworkProcess and run control loop."""
        if not self._transition_to(ClientState.STARTING):
            logging.error("Cannot start: invalid state")
            return

        logging.info("RSIClient sending start signal to NetworkProcess...")
        self.start_event.set()

        if not self._transition_to(ClientState.RUNNING):
            logging.error("Failed to transition to RUNNING state")
            return

        self.running = True
        logging.info("RSI Client Started")

        try:
            while self.running and not self.stop_event.is_set():
                time.sleep(2)
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logging.error(f"RSI Client encountered an error: {e}")
            self._transition_to(ClientState.ERROR)

    def stop(self):
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

        self._transition_to(ClientState.STOPPED)
        logging.info("RSI Client Stopped")

    def reconnect(self):
        """Reconnects the network process safely."""
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
            self.command_queue
        )
        self.network_process.logging_active = self._logging_active
        self.network_process.start()

        # Fresh control thread
        self.thread = threading.Thread(target=self.start, daemon=True)
        self.thread.start()

    def is_running(self) -> bool:
        """Check if client is in running state."""
        return self.state == ClientState.RUNNING

    def is_stopped(self) -> bool:
        """Check if client is fully stopped."""
        return self.state == ClientState.STOPPED

    def start_logging(self, filename):
        """Start CSV logging to the specified file."""
        self.command_queue.put({'action': 'start_logging', 'filename': filename})

    def stop_logging(self):
        """Stop CSV logging."""
        self.command_queue.put({'action': 'stop_logging'})

    def is_logging_active(self) -> bool:
        """Check if CSV logging is currently active."""
        return self._logging_active.value
