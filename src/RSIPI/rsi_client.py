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

    _VALID_TRANSITIONS = {
        ClientState.INITIALIZED: {ClientState.STARTING, ClientState.STOPPING},
        ClientState.STARTING: {ClientState.RUNNING, ClientState.STOPPING, ClientState.ERROR},
        ClientState.RUNNING: {ClientState.STOPPING, ClientState.ERROR},
        ClientState.STOPPING: {ClientState.STOPPED, ClientState.ERROR},
        ClientState.STOPPED: {ClientState.INITIALIZED},
        ClientState.ERROR: {ClientState.STOPPING, ClientState.INITIALIZED},
    }

    def __init__(
        self,
        config_file: str,
        rsi_limits_file: Optional[str] = None,
        enable_auto_reconnect: bool = False,
        auto_reconnect_retries: int = 5,
        auto_reconnect_delay: float = 5.0,
        rsi_mode: str = 'relative',
        max_cartesian_rate: float = 0.0,
        max_joint_rate: float = 0.0,
        cycle_time: float = 0.004
    ) -> None:
        """
        Args:
            config_file: Path to RSI_EthernetConfig.xml
            rsi_limits_file: Optional path to .rsi.xml safety limits file
            enable_auto_reconnect: Enable automatic reconnection on communication loss
            auto_reconnect_retries: Maximum reconnection attempts (0 = unlimited)
            auto_reconnect_delay: Base delay between retries in seconds
            rsi_mode: 'absolute' or 'relative' — must match KRL RSI_MOVECORR() mode
            max_cartesian_rate: Max mm/cycle for RKorr corrections (0 = disabled)
            max_joint_rate: Max degrees/cycle for AKorr corrections (0 = disabled)
            cycle_time: Expected RSI cycle time in seconds (0.004 or 0.012)
        """
        logging.info("Loading RSI configuration from %s...", config_file)
        self.rsi_mode = rsi_mode
        self.max_cartesian_rate = max_cartesian_rate
        self.max_joint_rate = max_joint_rate
        self.cycle_time = cycle_time

        self._state: ClientState = ClientState.INITIALIZED
        self._state_lock: Lock = Lock()

        self.config_parser: ConfigParser = ConfigParser(config_file, rsi_limits_file)
        network_settings = self.config_parser.get_network_settings()

        # Validate config on startup
        self._validate_config()

        self.manager: multiprocessing.Manager = multiprocessing.Manager()
        self.send_variables = self.manager.dict(self.config_parser.send_variables)
        self.receive_variables = self.manager.dict(self.config_parser.receive_variables)
        self.stop_event: multiprocessing.Event = multiprocessing.Event()
        self.start_event: multiprocessing.Event = multiprocessing.Event()
        self.connected_event: multiprocessing.Event = multiprocessing.Event()
        self.command_queue: multiprocessing.Queue = multiprocessing.Queue()

        self.safety_manager: SafetyManager = SafetyManager(self.config_parser.safety_limits)

        self._logging_active = multiprocessing.Value('b', False)
        self._receive_dirty = multiprocessing.Value('b', True)  # Dirty flag for IPC optimization

        self.metrics_dict = self.manager.dict()

        self._create_network_process(network_settings)

        self.logger: Optional[any] = None
        self.running: bool = False
        self.thread: Optional[Thread] = None

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

    def _validate_config(self) -> None:
        """Validate config and warn about common misconfigurations."""
        send = self.config_parser.send_variables
        recv = self.config_parser.receive_variables

        # Check correction variables are in receive (what we send to robot)
        if "RKorr" not in recv and "AKorr" not in recv:
            logging.warning(
                "Config validation: Neither RKorr nor AKorr found in RECEIVE section. "
                "You won't be able to send motion corrections to the robot. "
                "Check your RSI_EthernetConfig.xml <RECEIVE> elements."
            )

        # Check position feedback is in send (what robot sends to us)
        if "RIst" not in send:
            logging.warning(
                "Config validation: RIst not found in SEND section. "
                "You won't receive Cartesian position feedback from the robot."
            )

        if "IPOC" not in send:
            logging.warning(
                "Config validation: IPOC not found in SEND section. "
                "IPOC synchronisation may not work correctly."
            )

        # Validate RSI mode
        if self.rsi_mode not in ('absolute', 'relative'):
            logging.warning(
                "Config validation: rsi_mode='%s' is not valid. "
                "Use 'absolute' or 'relative'. Defaulting to 'relative'.",
                self.rsi_mode
            )
            self.rsi_mode = 'relative'

        # Log summary
        send_keys = [k for k in send if k != "IPOC"]
        recv_keys = [k for k in recv if k not in ("IPOC", "FREE")]
        logging.info(
            "Config validated: SEND=[%s] RECEIVE=[%s] mode=%s",
            ", ".join(send_keys), ", ".join(recv_keys), self.rsi_mode
        )

    def _create_network_process(self, network_settings: dict) -> None:
        """Create and start the NetworkProcess with current settings."""
        self.network_process: NetworkProcess = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event,
            self.command_queue,
            self.metrics_dict,
            self.connected_event,
            rsi_mode=self.rsi_mode,
            max_cartesian_rate=self.max_cartesian_rate,
            max_joint_rate=self.max_joint_rate,
            cycle_time=self.cycle_time
        )
        self.network_process.logging_active = self._logging_active
        self.network_process.receive_dirty = self._receive_dirty
        self.network_process.start()

    @property
    def state(self) -> ClientState:
        """Get current client state (thread-safe)."""
        with self._state_lock:
            return self._state

    def _transition_to(self, new_state: ClientState) -> bool:
        with self._state_lock:
            if new_state in self._VALID_TRANSITIONS.get(self._state, set()):
                old_state = self._state
                self._state = new_state
                logging.debug("State transition: %s -> %s", old_state.name, new_state.name)
                return True
            else:
                logging.warning(
                    "Invalid state transition attempted: %s -> %s", self._state.name, new_state.name
                )
                return False

    def start(self) -> None:
        """
        Send start signal to NetworkProcess and run control loop.

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

        if self.auto_reconnect_manager:
            self.auto_reconnect_manager.start()

        try:
            while self.running and not self.stop_event.is_set():
                time.sleep(2)
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logging.error("RSI Client encountered an error: %s", e)
            self._transition_to(ClientState.ERROR)
            raise

    def stop(self) -> None:
        """Stop the network process and the client thread safely."""
        if self.state in (ClientState.STOPPED, ClientState.STOPPING):
            logging.debug("Already stopped or stopping")
            return

        if not self._transition_to(ClientState.STOPPING):
            logging.warning("Could not transition to STOPPING state")

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

        if self.auto_reconnect_manager:
            self.auto_reconnect_manager.stop()

        # Shutdown Manager to avoid resource leaks
        try:
            self.manager.shutdown()
        except Exception:
            pass

        self._transition_to(ClientState.STOPPED)
        logging.info("RSI Client Stopped")

    def reconnect(self) -> None:
        """
        Reconnect the network process safely.

        Stops existing connection, resets state, and creates fresh
        network process with new communication resources.
        """
        logging.info("Reconnecting RSI Client network...")

        if self.state in (ClientState.RUNNING, ClientState.STARTING):
            self.stop()

        if self.network_process and self.network_process.is_alive():
            self.stop_event.set()
            self.network_process.terminate()
            self.network_process.join()

        # Fresh Manager (old one was shut down in stop())
        self.manager = multiprocessing.Manager()
        self.send_variables = self.manager.dict(self.config_parser.send_variables)
        self.receive_variables = self.manager.dict(self.config_parser.receive_variables)
        self.metrics_dict = self.manager.dict()

        with self._state_lock:
            self._state = ClientState.INITIALIZED

        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()
        self.connected_event = multiprocessing.Event()
        self.command_queue = multiprocessing.Queue()
        self._receive_dirty = multiprocessing.Value('b', True)

        network_settings = self.config_parser.get_network_settings()
        self._create_network_process(network_settings)

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        """
        Block until the first valid packet is received from the robot.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if connected, False if timeout
        """
        return self.connected_event.wait(timeout=timeout)

    def emergency_stop(self) -> None:
        """Send E-stop command to network process to zero all corrections."""
        self.safety_manager.emergency_stop()
        self.command_queue.put({'action': 'estop'})
        logging.critical("Emergency stop activated")

    def emergency_reset(self) -> None:
        """Reset E-stop and resume normal corrections."""
        self.safety_manager.reset_stop()
        self.command_queue.put({'action': 'estop_reset'})
        logging.info("Emergency stop reset")

    def is_running(self) -> bool:
        return self.state == ClientState.RUNNING

    def is_stopped(self) -> bool:
        return self.state == ClientState.STOPPED

    def start_logging(self, filename: str) -> None:
        self.command_queue.put({'action': 'start_logging', 'filename': filename})

    def stop_logging(self) -> None:
        self.command_queue.put({'action': 'stop_logging'})

    def is_logging_active(self) -> bool:
        return self._logging_active.value
