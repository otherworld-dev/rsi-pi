import multiprocessing
import socket
import logging
import threading
import xml.etree.ElementTree as ET
import os
import datetime
from queue import Empty, Queue as ThreadQueue
from typing import Dict, Any, Tuple, Optional
from .xml_handler import XMLGenerator, FastXMLGenerator
from .safety_manager import SafetyManager
from .exceptions import RSINetworkError, RSITimeoutError, RSIPacketError, RSILoggingError
from .timing_metrics import TimingMetrics


class CSVLogger(threading.Thread):
    """
    Background thread for writing CSV logs without blocking the network loop.

    Uses a thread instead of a subprocess to avoid Windows restrictions on
    daemon processes spawning child processes.
    """

    def __init__(self, log_queue: ThreadQueue, stop_event: threading.Event, filename: str) -> None:
        super().__init__(daemon=True)
        self.log_queue = log_queue
        self.stop_event = stop_event
        self.filename = filename

    def run(self) -> None:
        log_dir = os.path.dirname(self.filename)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        header_written = False

        try:
            with open(self.filename, 'w', newline='') as f:
                while not self.stop_event.is_set():
                    try:
                        entry = self.log_queue.get(timeout=0.5)
                        if entry is None:
                            break

                        if not header_written:
                            headers = ['Timestamp'] + list(entry.keys())
                            f.write(','.join(headers) + '\n')
                            header_written = True

                        timestamp = datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f")[:-3]
                        values = [timestamp] + [str(v) for v in entry.values()]
                        f.write(','.join(values) + '\n')
                        f.flush()

                    except Empty:
                        continue
                    except Exception as e:
                        logging.error("CSV logging error: %s", e)

        except Exception as e:
            logging.error("Failed to open log file %s: %s", self.filename, e)


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
        command_queue: multiprocessing.Queue,
        metrics_dict: Optional[Any] = None,  # multiprocessing.Manager().dict()
        connected_event: Optional[multiprocessing.Event] = None,
        rsi_mode: str = 'relative',
        max_cartesian_rate: float = 0.0,
        max_joint_rate: float = 0.0,
        cycle_time: float = 0.004
    ) -> None:
        super().__init__()
        self.send_variables = send_variables
        self.receive_variables = receive_variables
        self.stop_event: multiprocessing.Event = stop_event
        self.start_event: multiprocessing.Event = start_event
        self.config_parser = config_parser
        self.command_queue: multiprocessing.Queue = command_queue
        self.safety_manager: SafetyManager = SafetyManager(config_parser.safety_limits)
        self.connected_event = connected_event

        # RSI correction mode and rate limiting
        self.rsi_mode: str = rsi_mode  # 'absolute' or 'relative'
        self.max_cartesian_rate: float = max_cartesian_rate  # mm/cycle, 0 = disabled
        self.max_joint_rate: float = max_joint_rate  # degrees/cycle, 0 = disabled
        self.cycle_time: float = cycle_time  # expected cycle time for metrics

        self.client_address: Tuple[str, int] = (ip, port)
        self.logging_active: Any = multiprocessing.Value('b', False)  # c_bool wrapper
        self.estop_active: Any = multiprocessing.Value('b', False)

        self.controller_ip_and_port: Optional[Tuple[str, int]] = None
        self.udp_socket: Optional[socket.socket] = None

        # Logging infrastructure (created when logging starts)
        self.log_queue: Optional[ThreadQueue] = None
        self.log_stop_event: Optional[threading.Event] = None
        self.csv_logger: Optional[CSVLogger] = None

        # Timing metrics (Phase 2)
        self.metrics_dict = metrics_dict
        self.timing_metrics: Optional[TimingMetrics] = None

    def run(self) -> None:
        """
        Start the network loop.

        Waits for start signal, then initializes socket and begins
        communication loop. Ensures cleanup on exit.
        """
        # Initialize timing metrics in child process
        if self.metrics_dict is not None:
            self.timing_metrics = TimingMetrics(expected_cycle_time=self.cycle_time)
            logging.info("Timing metrics initialized (expected cycle: %.1fms)", self.cycle_time * 1000)

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
            logging.warning("Invalid IP address '%s'. Falling back to '0.0.0.0'.", self.client_address[0])
            self.client_address = ('0.0.0.0', self.client_address[1])

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.settimeout(5)
        self.udp_socket.bind(self.client_address)
        logging.info("Network process bound on %s", self.client_address)

    def _run_loop(self) -> None:
        """
        Main communication loop.

        Uses local dict snapshots to avoid per-key IPC overhead on
        multiprocessing.Manager dicts within the 4ms cycle.
        """
        update_counter = 0
        metrics_counter = 0
        cmd_counter = 0
        first_packet = True

        # Variable naming follows KUKA convention (robot's perspective):
        #   send_variables = what the robot SENDS to us (RIst, RSol, IPOC, etc.)
        #   receive_variables = what the robot RECEIVES from us (RKorr, DiO, EStr, etc.)

        # Local working copies — avoid Manager IPC in the hot path
        local_robot_out = dict(self.send_variables)
        local_robot_in = dict(self.receive_variables)
        network_settings = self.config_parser.network_settings

        # FastXMLGenerator for comparison testing
        fast_gen = FastXMLGenerator(local_robot_in, root_tag="Sen", type_attr=network_settings["sentype"])
        xml_mismatch_logged = False

        # Cache zero-correction template for E-stop
        zero_robot_in = dict(self.receive_variables)
        for key, value in zero_robot_in.items():
            if isinstance(value, dict):
                zero_robot_in[key] = {k: 0.0 for k in value}

        # Previous correction state for absolute mode ramping
        prev_corrections: Dict[str, Dict[str, float]] = {}
        for key in ('RKorr', 'AKorr'):
            if key in local_robot_in and isinstance(local_robot_in[key], dict):
                prev_corrections[key] = {k: 0.0 for k in local_robot_in[key]}

        while not self.stop_event.is_set():
            # Check for commands periodically (every 50 cycles ~200ms)
            cmd_counter += 1
            if cmd_counter >= 50:
                self._process_commands()
                cmd_counter = 0

            try:
                data_received, self.controller_ip_and_port = self.udp_socket.recvfrom(1024)
                message = data_received.decode()

                # Parse robot's outgoing data (ElementTree — handles any attribute order)
                try:
                    self._parse_received_data(message, local_robot_out)
                except RSIPacketError:
                    logging.warning("Parse failed, sending last known good response")

                # Signal connection on first valid packet
                if first_packet:
                    first_packet = False
                    if self.connected_event:
                        self.connected_event.set()

                # Snapshot receive_variables to pick up user changes (single IPC call)
                local_robot_in = dict(self.receive_variables)

                # Sync IPOC: robot sends it, we echo back IPOC+4
                if "IPOC" in local_robot_out:
                    ipoc = local_robot_out["IPOC"]
                    local_robot_in["IPOC"] = ipoc + 4

                # Rate-limit corrections
                self._apply_rate_limit(local_robot_in, prev_corrections)

                # E-stop: zero all corrections, keep IPOC sync
                if self.estop_active.value:
                    estop_response = dict(zero_robot_in)
                    estop_response["IPOC"] = local_robot_in.get("IPOC", 0)
                    send_xml = XMLGenerator.generate_send_xml(estop_response, network_settings)
                else:
                    send_xml = XMLGenerator.generate_send_xml(local_robot_in, network_settings)

                # Compare FastXMLGenerator output (debug — log first mismatch only)
                if not xml_mismatch_logged:
                    try:
                        fast_xml = fast_gen.generate(local_robot_in)
                        if fast_xml != send_xml:
                            xml_mismatch_logged = True
                            logging.warning("XML MISMATCH DETECTED")
                            logging.warning("ET output:   %s", send_xml[:200])
                            logging.warning("Fast output: %s", fast_xml[:200])
                        elif metrics_counter == 0:
                            # Log match confirmation once (on first sync cycle)
                            logging.info("XML generators match OK")
                            xml_mismatch_logged = True
                    except Exception as e:
                        logging.warning("FastXMLGenerator error: %s", e)
                        xml_mismatch_logged = True

                self.udp_socket.sendto(send_xml.encode(), self.controller_ip_and_port)

                # Sync robot's outgoing data → Manager dict periodically (every 10 cycles ~40ms)
                metrics_counter += 1
                if metrics_counter >= 10:
                    self.send_variables.update(local_robot_out)
                    metrics_counter = 0

                # Record timing metrics (Phase 2)
                if self.timing_metrics is not None:
                    self.timing_metrics.record_cycle(local_robot_out.get("IPOC", 0))

                    update_counter += 1
                    if update_counter >= 100:
                        self._update_metrics_dict()
                        update_counter = 0

                if self.logging_active.value and self.log_queue:
                    self._queue_log_entry(local_robot_out, local_robot_in)

            except socket.timeout:
                logging.warning("No message received within timeout period")
                if self.timing_metrics and self.timing_metrics.check_watchdog():
                    logging.error("Watchdog timeout - communication lost!")
            except Exception as e:
                logging.error("Network process error: %s", e)

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
                elif action == 'estop':
                    self.estop_active.value = True
                elif action == 'estop_reset':
                    self.estop_active.value = False

        except Empty:
            pass
        except Exception as e:
            logging.error("Error processing command: %s", e)

    def _apply_rate_limit(self, robot_in: dict, prev: Dict[str, Dict[str, float]]) -> None:
        """
        Apply per-cycle rate limiting to correction values in-place.

        In relative mode: clamp each value directly (it IS the per-cycle delta).
        In absolute mode: clamp the change from previous cycle and ramp toward target.

        Args:
            robot_in: Current outgoing corrections dict (modified in-place)
            prev: Previous cycle's correction values (updated in-place)
        """
        cartesian_keys = {'X', 'Y', 'Z', 'A', 'B', 'C'}
        joint_keys = {'A1', 'A2', 'A3', 'A4', 'A5', 'A6'}

        for corr_key, max_rate, axis_set in [
            ('RKorr', self.max_cartesian_rate, cartesian_keys),
            ('AKorr', self.max_joint_rate, joint_keys),
        ]:
            if max_rate <= 0:
                continue  # Rate limiting disabled for this type
            if corr_key not in robot_in or not isinstance(robot_in[corr_key], dict):
                continue

            corr = robot_in[corr_key]
            prev_corr = prev.get(corr_key, {})

            if self.rsi_mode == 'relative':
                # Each value is a per-cycle delta — clamp directly
                for axis in corr:
                    if axis in axis_set:
                        val = corr[axis]
                        corr[axis] = max(-max_rate, min(max_rate, val))
            else:
                # Absolute mode — clamp the change from previous
                for axis in corr:
                    if axis in axis_set:
                        target = corr[axis]
                        previous = prev_corr.get(axis, 0.0)
                        delta = target - previous
                        clamped_delta = max(-max_rate, min(max_rate, delta))
                        corr[axis] = previous + clamped_delta
                        prev_corr[axis] = corr[axis]

            robot_in[corr_key] = corr
            prev[corr_key] = prev_corr

    def _update_metrics_dict(self) -> None:
        """Update shared metrics dictionary with current timing statistics."""
        if self.metrics_dict is None or self.timing_metrics is None:
            return

        try:
            stats = self.timing_metrics.get_current_stats()
            health = self.timing_metrics.get_health_status()

            for key, value in stats.items():
                self.metrics_dict[key] = value

            self.metrics_dict['is_healthy'] = health['is_healthy']
            self.metrics_dict['warnings'] = health['warnings']
            self.metrics_dict['watchdog_timeout'] = health['watchdog_timeout']

        except Exception as e:
            logging.debug("Failed to update metrics dict: %s", e)

    def _queue_log_entry(self, local_robot_out: dict, local_robot_in: dict) -> None:
        """Queue current state for CSV logging using local dicts (no IPC)."""
        try:
            entry = {}
            for key, value in local_robot_out.items():
                if isinstance(value, dict):
                    for subkey, subval in value.items():
                        entry[f"Send.{key}.{subkey}"] = subval
                else:
                    entry[f"Send.{key}"] = value

            for key, value in local_robot_in.items():
                if isinstance(value, dict):
                    for subkey, subval in value.items():
                        entry[f"Receive.{key}.{subkey}"] = subval
                else:
                    entry[f"Receive.{key}"] = value

            try:
                self.log_queue.put_nowait(entry)
            except:
                pass  # Queue full, skip this entry rather than block

        except Exception as e:
            logging.debug("Failed to queue log entry: %s", e)

    def _start_logging(self, filename: str) -> None:
        """Start CSV logging to the specified file."""
        if self.logging_active.value:
            logging.warning("Logging already active")
            return

        self.log_queue = ThreadQueue(maxsize=1000)
        self.log_stop_event = threading.Event()

        self.csv_logger = CSVLogger(self.log_queue, self.log_stop_event, filename)
        self.csv_logger.start()

        self.logging_active.value = True
        logging.info("CSV logging started: %s", filename)

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

        self.csv_logger = None
        self.log_queue = None
        self.log_stop_event = None
        logging.info("CSV logging stopped")

    def _cleanup(self) -> None:
        """Clean up resources on shutdown."""
        self._stop_logging()

        if self.udp_socket:
            try:
                self.udp_socket.close()
                logging.info("Network socket closed")
            except Exception as e:
                logging.error("Error closing socket: %s", e)
            self.udp_socket = None

    @staticmethod
    def is_valid_ip(ip: str) -> bool:
        try:
            socket.inet_aton(ip)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.bind((ip, 0))
            return True
        except (socket.error, OSError):
            return False

    @staticmethod
    def _parse_received_data(xml_string: str, target: dict) -> None:
        """Parse received XML message into a local dict (no IPC)."""
        try:
            root = ET.fromstring(xml_string)
            for element in root:
                if element.tag in target:
                    if len(element.attrib) > 0:
                        existing = target.get(element.tag)
                        if isinstance(existing, dict):
                            for k, v in element.attrib.items():
                                existing[k] = float(v)
                        else:
                            target[element.tag] = {k: float(v) for k, v in element.attrib.items()}
                    else:
                        target[element.tag] = element.text
                if element.tag == "IPOC":
                    target["IPOC"] = int(element.text)
        except ET.ParseError as e:
            logging.error("XML parse error in received message: %s", e)
            raise RSIPacketError(f"Failed to parse received XML: {e}") from e
        except Exception as e:
            logging.error("Error processing received message: %s", e)
            raise RSIPacketError(f"Unexpected error parsing packet: {e}") from e

    def process_received_data(self, xml_string: str) -> None:
        """Legacy method kept for compatibility (e.g. echo server)."""
        self._parse_received_data(xml_string, self.send_variables)
        if "IPOC" in self.send_variables:
            self.receive_variables["IPOC"] = self.send_variables["IPOC"] + 4
