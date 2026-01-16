import multiprocessing
import socket
import logging
import xml.etree.ElementTree as ET
import os
import datetime
from queue import Empty
from typing import Dict, Any, Tuple, Optional
from .xml_handler import XMLGenerator
from .safety_manager import SafetyManager
from .exceptions import RSINetworkError, RSITimeoutError, RSIPacketError, RSILoggingError


class CSVLogger(multiprocessing.Process):
    """
    Separate process for writing CSV logs without blocking the network loop.

    Runs in background and consumes log entries from a queue, writing them
    to CSV file with British date format timestamps.
    """

    def __init__(self, log_queue: multiprocessing.Queue, stop_event: multiprocessing.Event, filename: str) -> None:
        """
        Initialize CSV logger process.

        Args:
            log_queue: Queue containing log entry dictionaries
            stop_event: Event to signal shutdown
            filename: Path to output CSV file
        """
        super().__init__()
        self.log_queue: multiprocessing.Queue = log_queue
        self.stop_event: multiprocessing.Event = stop_event
        self.filename: str = filename
        self.daemon = True

    def run(self) -> None:
        """
        Write log entries from queue to CSV file.

        Creates directory if needed, writes header on first entry,
        timestamps each row with British date format (DD/MM/YYYY HH:MM:SS.mmm).
        """
        # Ensure logs directory exists
        log_dir = os.path.dirname(self.filename)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        header_written = False

        try:
            with open(self.filename, 'w', newline='') as f:
                while not self.stop_event.is_set():
                    try:
                        entry = self.log_queue.get(timeout=0.5)
                        if entry is None:  # Poison pill
                            break

                        # Write header on first entry
                        if not header_written:
                            headers = ['Timestamp'] + list(entry.keys())
                            f.write(','.join(headers) + '\n')
                            header_written = True

                        # Write data row
                        timestamp = datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f")[:-3]
                        values = [timestamp] + [str(v) for v in entry.values()]
                        f.write(','.join(values) + '\n')
                        f.flush()

                    except Empty:
                        continue
                    except Exception as e:
                        logging.error(f"CSV logging error: {e}")

        except Exception as e:
            logging.error(f"Failed to open log file {self.filename}: {e}")


class NetworkProcess(multiprocessing.Process):
    """
    Handles UDP communication and CSV logging in a separate process.

    Manages bidirectional UDP communication with KUKA robot controller,
    including IPOC synchronization, variable updates, and optional CSV logging.
    Runs in separate process to avoid GIL contention with main thread.
    """

    def __init__(
        self,
        ip: str,
        port: int,
        send_variables: Any,  # multiprocessing.Manager().dict()
        receive_variables: Any,  # multiprocessing.Manager().dict()
        stop_event: multiprocessing.Event,
        config_parser: Any,  # ConfigParser type
        start_event: multiprocessing.Event,
        command_queue: multiprocessing.Queue
    ) -> None:
        """
        Initialize network process.

        Args:
            ip: IP address to bind UDP socket to
            port: UDP port number
            send_variables: Shared dict for variables to send to robot
            receive_variables: Shared dict for variables received from robot
            stop_event: Event to signal shutdown
            config_parser: ConfigParser instance with network settings
            start_event: Event to signal when to start communication
            command_queue: Queue for receiving commands from parent process
        """
        super().__init__()
        self.send_variables = send_variables
        self.receive_variables = receive_variables
        self.stop_event: multiprocessing.Event = stop_event
        self.start_event: multiprocessing.Event = start_event
        self.config_parser = config_parser
        self.command_queue: multiprocessing.Queue = command_queue
        self.safety_manager: SafetyManager = SafetyManager(config_parser.safety_limits)

        self.client_address: Tuple[str, int] = (ip, port)
        self.logging_active: Any = multiprocessing.Value('b', False)  # c_bool wrapper

        self.controller_ip_and_port: Optional[Tuple[str, int]] = None
        self.udp_socket: Optional[socket.socket] = None

        # Logging infrastructure (created when logging starts)
        self.log_queue: Optional[multiprocessing.Queue] = None
        self.log_stop_event: Optional[multiprocessing.Event] = None
        self.csv_logger: Optional[CSVLogger] = None

    def run(self) -> None:
        """
        Start the network loop.

        Waits for start signal, then initializes socket and begins
        communication loop. Ensures cleanup on exit.
        """
        # Wait for start signal, but check stop_event periodically to allow clean shutdown
        while not self.start_event.wait(timeout=0.5):
            if self.stop_event.is_set():
                logging.info("Network process stopped before starting")
                return

        try:
            self._setup_socket()
            self._run_loop()
        finally:
            self._cleanup()

    def _setup_socket(self) -> None:
        """
        Create and bind the UDP socket.

        Falls back to 0.0.0.0 if specified IP is invalid.
        """
        if not self.is_valid_ip(self.client_address[0]):
            logging.warning(f"Invalid IP address '{self.client_address[0]}'. Falling back to '0.0.0.0'.")
            self.client_address = ('0.0.0.0', self.client_address[1])

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(self.client_address)
        logging.info(f"Network process bound on {self.client_address}")

    def _run_loop(self) -> None:
        """
        Main communication loop.

        Receives UDP messages from robot, processes them, sends responses,
        and optionally logs data to CSV.
        """
        while not self.stop_event.is_set():
            # Check for commands (non-blocking)
            self._process_commands()

            try:
                self.udp_socket.settimeout(5)
                data_received, self.controller_ip_and_port = self.udp_socket.recvfrom(1024)
                message = data_received.decode()
                self.process_received_data(message)
                send_xml = XMLGenerator.generate_send_xml(self.send_variables, self.config_parser.network_settings)
                self.udp_socket.sendto(send_xml.encode(), self.controller_ip_and_port)

                if self.logging_active.value and self.log_queue:
                    self._queue_log_entry()

            except socket.timeout:
                logging.warning("No message received within timeout period")
            except Exception as e:
                logging.error(f"Network process error: {e}")

    def _process_commands(self) -> None:
        """Process any pending commands from the parent process."""
        try:
            while True:
                cmd = self.command_queue.get_nowait()
                if cmd is None:
                    continue

                action = cmd.get('action')
                if action == 'start_logging':
                    self._start_logging(cmd.get('filename'))
                elif action == 'stop_logging':
                    self._stop_logging()

        except Empty:
            pass
        except Exception as e:
            logging.error(f"Error processing command: {e}")

    def _queue_log_entry(self) -> None:
        """Queue current state for CSV logging (non-blocking)."""
        try:
            entry = {}
            # Flatten send variables
            for key, value in dict(self.send_variables).items():
                if isinstance(value, dict):
                    for subkey, subval in value.items():
                        entry[f"Send.{key}.{subkey}"] = subval
                else:
                    entry[f"Send.{key}"] = value

            # Flatten receive variables
            for key, value in dict(self.receive_variables).items():
                if isinstance(value, dict):
                    for subkey, subval in value.items():
                        entry[f"Receive.{key}.{subkey}"] = subval
                else:
                    entry[f"Receive.{key}"] = value

            # Non-blocking put - drop entry if queue is full
            try:
                self.log_queue.put_nowait(entry)
            except:
                pass  # Queue full, skip this entry rather than block

        except Exception as e:
            logging.debug(f"Failed to queue log entry: {e}")

    def _start_logging(self, filename: str) -> None:
        """
        Start CSV logging to the specified file.

        Args:
            filename: Path to CSV output file
        """
        if self.logging_active.value:
            logging.warning("Logging already active")
            return

        self.log_queue = multiprocessing.Queue(maxsize=1000)
        self.log_stop_event = multiprocessing.Event()

        self.csv_logger = CSVLogger(self.log_queue, self.log_stop_event, filename)
        self.csv_logger.start()

        self.logging_active.value = True
        logging.info(f"CSV logging started: {filename}")

    def _stop_logging(self) -> None:
        """Stop CSV logging and cleanup resources."""
        if not self.logging_active.value:
            return

        self.logging_active.value = False

        if self.log_queue:
            try:
                self.log_queue.put_nowait(None)  # Poison pill
            except:
                pass

        if self.log_stop_event:
            self.log_stop_event.set()

        if self.csv_logger and self.csv_logger.is_alive():
            self.csv_logger.join(timeout=2)
            if self.csv_logger.is_alive():
                self.csv_logger.terminate()

        self.csv_logger = None
        self.log_queue = None
        self.log_stop_event = None
        logging.info("CSV logging stopped")

    def _cleanup(self) -> None:
        """Clean up resources on shutdown."""
        # Stop logging first
        self._stop_logging()

        if self.udp_socket:
            try:
                self.udp_socket.close()
                logging.info("Network socket closed")
            except Exception as e:
                logging.error(f"Error closing socket: {e}")
            self.udp_socket = None

    @staticmethod
    def is_valid_ip(ip: str) -> bool:
        """
        Check if an IP address is valid and bindable.

        Args:
            ip: IP address string to validate

        Returns:
            True if IP is valid and can be bound to
        """
        try:
            socket.inet_aton(ip)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.bind((ip, 0))
            return True
        except (socket.error, OSError):
            return False

    def process_received_data(self, xml_string: str) -> None:
        """
        Parse received XML message and update receive_variables.

        Handles IPOC synchronization by echoing back IPOC+4.

        Args:
            xml_string: XML message string from robot controller

        Raises:
            RSIPacketError: If XML parsing fails
        """
        try:
            root = ET.fromstring(xml_string)
            for element in root:
                if element.tag in self.receive_variables:
                    if len(element.attrib) > 0:
                        self.receive_variables[element.tag] = {k: float(v) for k, v in element.attrib.items()}
                    else:
                        self.receive_variables[element.tag] = element.text
                if element.tag == "IPOC":
                    received_ipoc = int(element.text)
                    self.receive_variables["IPOC"] = received_ipoc
                    self.send_variables["IPOC"] = received_ipoc + 4
        except ET.ParseError as e:
            logging.error(f"XML parse error in received message: {e}")
            raise RSIPacketError(f"Failed to parse received XML: {e}") from e
        except Exception as e:
            logging.error(f"Error processing received message: {e}")
            raise RSIPacketError(f"Unexpected error parsing packet: {e}") from e
