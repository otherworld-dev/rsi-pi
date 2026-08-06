import multiprocessing
import socket
import logging
import threading
import time
import xml.etree.ElementTree as ET
import os
import datetime
from queue import Empty, Queue as ThreadQueue
from typing import Dict, Any, Tuple, Optional
from .xml_handler import XMLGenerator
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
    including IPOC synchronization, variable updates, safety clamping,
    E-stop substitution, and optional CSV logging. Runs as a daemon
    process (CSVLogger is a thread, so the no-child-process daemon
    restriction is not violated) so an unclean parent exit can never
    hang the interpreter.
    """

    # Correction keys substituted on E-stop / one-shot; I/O and Tech pass through.
    _CORRECTION_KEYS = ('RKorr', 'AKorr')
    _CONSECUTIVE_ERROR_LIMIT = 250  # ~1s of continuous hot-loop failures

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
        cycle_time: float = 0.004,
        estop_active: Optional[Any] = None,   # shared Value('b'), parent-owned
        corr_seq: Optional[Any] = None,       # shared Value('q'), parent-owned
        corr_ack: Optional[Any] = None,       # shared Value('q'), parent-owned
        oneshot_active: Optional[Any] = None,  # shared Value('b'), parent-owned
        ipoc_value: Optional[Any] = None,     # shared Value('q'), parent-owned
        corr_lock: Optional[Any] = None,      # shared Lock, parent-owned
    ) -> None:
        super().__init__(daemon=True)
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

        # Parent-owned shared Values (created here only for direct/legacy users;
        # RSIClient always passes its own so they survive reconnect()).
        self.estop_active: Any = estop_active if estop_active is not None else multiprocessing.Value('b', False)
        self.corr_seq: Any = corr_seq if corr_seq is not None else multiprocessing.Value('q', 0)
        self.corr_ack: Any = corr_ack if corr_ack is not None else multiprocessing.Value('q', 0)
        self.oneshot_active: Any = oneshot_active if oneshot_active is not None else multiprocessing.Value('b', False)
        self.ipoc_value: Any = ipoc_value if ipoc_value is not None else multiprocessing.Value('q', 0)
        self.corr_lock: Any = corr_lock if corr_lock is not None else multiprocessing.Lock()

        self.controller_ip_and_port: Optional[Tuple[str, int]] = None
        self.udp_socket: Optional[socket.socket] = None

        # Logging infrastructure (created when logging starts)
        self.log_queue: Optional[ThreadQueue] = None
        self.log_stop_event: Optional[threading.Event] = None
        self.csv_logger: Optional[CSVLogger] = None

        # Timing metrics
        self.metrics_dict = metrics_dict
        self.timing_metrics: Optional[TimingMetrics] = None

        # Send-time limit clamping (rebuilt whenever limits change)
        self._clamp_table: list = []
        self._clamp_log_state: Dict[str, Tuple[float, int]] = {}
        self._build_clamp_table()

    # ------------------------------------------------------------------ setup

    def run(self) -> None:
        """
        Start the network loop.

        Waits for start signal, then initializes socket and begins
        communication loop. Ensures cleanup on exit.
        """
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
        # 1s matches the watchdog threshold; also bounds worst-case iteration
        # length so queued commands and E-stop state are never stalled for long.
        self.udp_socket.settimeout(1.0)
        self.udp_socket.bind(self.client_address)
        logging.info("Network process bound on %s", self.client_address)

    # ------------------------------------------------------------------ loop

    def _run_loop(self) -> None:
        """
        Main communication loop.

        Uses local dict snapshots to avoid per-key IPC overhead on
        multiprocessing.Manager dicts within the 4ms cycle. Per-cycle
        pipeline: snapshot -> one-shot substitution -> limit clamp ->
        rate limit -> E-stop substitution -> serialize -> send -> ack.
        """
        sync_counter = 0
        metrics_counter = 0
        cmd_counter = 0
        first_packet = True
        was_timed_out = False
        consecutive_errors = 0
        estop_was_active = False

        # Variable naming follows KUKA convention (robot's perspective):
        #   send_variables = what the robot SENDS to us (RIst, RSol, IPOC, etc.)
        #   receive_variables = what the robot RECEIVES from us (RKorr, DiO, EStr, etc.)
        local_robot_out = dict(self.send_variables)
        local_robot_in = dict(self.receive_variables)
        network_settings = self.config_parser.network_settings
        # ONLYSEND=TRUE (RSI config): the robot streams data and expects NO
        # reply from the sensor — the entire reply path is skipped.
        onlysend = bool(network_settings.get("onlysend"))
        if onlysend:
            logging.info("ONLYSEND mode: receiving robot data only, no replies will be sent")

        # Correction state: zeros in relative mode; last-transmitted offset in
        # absolute mode. Doubles as the substitution source during E-stop.
        prev_corrections: Dict[str, Dict[str, float]] = {}
        for key in self._CORRECTION_KEYS:
            if key in local_robot_in and isinstance(local_robot_in[key], dict):
                prev_corrections[key] = {k: 0.0 for k in local_robot_in[key]}

        # A fresh process must never ack a waypoint published to a previous
        # process (reconnect); start from whatever seq the parent is at.
        last_acked_seq = self.corr_seq.value

        while not self.stop_event.is_set():
            # Drain pending commands every ~10 cycles (~40ms at 4ms cycles)
            cmd_counter += 1
            if cmd_counter >= 10:
                self._process_commands()
                cmd_counter = 0

            try:
                # 64KB = UDP maximum; the Full config's telegrams exceed 1KB
                # and Windows raises WinError 10040 on undersized buffers.
                data_received, self.controller_ip_and_port = self.udp_socket.recvfrom(65535)
                message = data_received.decode()

                # Parse robot's outgoing data (ElementTree — handles any attribute order)
                try:
                    self._parse_received_data(message, local_robot_out)
                except RSIPacketError:
                    logging.warning("Parse failed, sending last known good response")

                if first_packet:
                    first_packet = False
                    if self.connected_event:
                        self.connected_event.set()

                ipoc = local_robot_out.get("IPOC", 0)

                if not onlysend:
                    # Snapshot (seq, receive_variables) as an atomic pair — a
                    # publish bumps the seq only after its payload is written,
                    # both under corr_lock, so seeing seq N here guarantees the
                    # snapshot contains waypoint N's corrections.
                    with self.corr_lock:
                        seq = self.corr_seq.value
                        local_robot_in = dict(self.receive_variables)

                    # IPOC sync: the robot owns the clock and advances it each
                    # cycle; the reply must echo the received IPOC UNCHANGED
                    # (packets with a mismatched timestamp are rejected).
                    local_robot_in["IPOC"] = ipoc

                    estop_now = self._handle_estop_transition(prev_corrections, estop_was_active)
                    if estop_now:
                        # Substitute safe corrections: zeros (relative) or the
                        # frozen last-transmitted offset (absolute). Zeroing in
                        # absolute mode would command a return-to-path motion.
                        for key, held in prev_corrections.items():
                            if key in local_robot_in:
                                local_robot_in[key] = dict(held)
                    else:
                        # One-shot latch: a published relative delta is transmitted
                        # on exactly one cycle; until a new seq arrives, send zeros.
                        if (self.oneshot_active.value and self.rsi_mode == 'relative'
                                and seq == last_acked_seq):
                            for key in self._CORRECTION_KEYS:
                                if isinstance(local_robot_in.get(key), dict):
                                    local_robot_in[key] = {k: 0.0 for k in local_robot_in[key]}

                        self._enforce_limits(local_robot_in)
                        self._apply_rate_limit(local_robot_in, prev_corrections)

                        if self.rsi_mode != 'absolute':
                            # In absolute mode _apply_rate_limit maintains
                            # prev_corrections; in relative mode track what we
                            # transmit so E-stop can freeze/zero coherently.
                            for key in self._CORRECTION_KEYS:
                                val = local_robot_in.get(key)
                                if isinstance(val, dict):
                                    prev_corrections[key] = {k: 0.0 for k in val}

                    send_xml = XMLGenerator.generate_send_xml(local_robot_in, network_settings)
                    self.udp_socket.sendto(send_xml.encode(), self.controller_ip_and_port)

                    # Ack after a successful transmit — but never during E-stop,
                    # so trajectory executors time out and abort cleanly.
                    if not estop_now and seq != last_acked_seq:
                        last_acked_seq = seq
                        self.corr_ack.value = seq

                    estop_was_active = estop_now

                self.ipoc_value.value = ipoc
                consecutive_errors = 0

                # Sync robot's outgoing data -> Manager dict periodically (every 10 cycles)
                sync_counter += 1
                if sync_counter >= 10:
                    self.send_variables.update(local_robot_out)
                    sync_counter = 0

                if self.timing_metrics is not None:
                    self.timing_metrics.record_cycle(ipoc)

                    metrics_counter += 1
                    if metrics_counter >= 100 or was_timed_out:
                        self._update_metrics_dict()
                        metrics_counter = 0
                was_timed_out = False

                if self.logging_active.value and self.log_queue:
                    self._queue_log_entry(local_robot_out, local_robot_in)

            except socket.timeout:
                logging.warning("No message received within timeout period")
                was_timed_out = True
                # Commands (E-stop reset, logging, limits) must apply even
                # while the robot is silent.
                self._process_commands()
                cmd_counter = 0
                if self.timing_metrics:
                    if self.timing_metrics.check_watchdog():
                        logging.error("Watchdog timeout - communication lost!")
                    # Publish so the parent (auto-reconnect, diagnostics)
                    # can actually observe the loss.
                    self._update_metrics_dict()
            except Exception as e:
                logging.error("Network process error: %s", e)
                consecutive_errors += 1
                if isinstance(e, (BrokenPipeError, EOFError, ConnectionResetError)):
                    logging.error("Manager unreachable (parent gone?) - network process exiting")
                    break
                if consecutive_errors > self._CONSECUTIVE_ERROR_LIMIT:
                    logging.error("Too many consecutive errors - network process exiting")
                    break

    # ------------------------------------------------------------------ stages

    def _handle_estop_transition(
        self,
        prev_corrections: Dict[str, Dict[str, float]],
        estop_was_active: bool,
    ) -> bool:
        """
        Handle E-stop state transitions; returns current E-stop state.

        On the rising edge the user's pending corrections are cleared at the
        source (receive_variables) so motion cannot resume from stale values
        after reset:
        - relative mode: corrections and ramp state are zeroed;
        - absolute mode: the last-transmitted offset is FROZEN (held) — both
          on the wire and in the ramp state — so the rate limiter cannot
          ramp toward a stale target during the stop, and reset resumes from
          the held offset with a zero-magnitude step.
        """
        active = bool(self.estop_active.value)
        if active and not estop_was_active:
            try:
                if self.rsi_mode == 'relative':
                    for key in prev_corrections:
                        prev_corrections[key] = {k: 0.0 for k in prev_corrections[key]}
                # In absolute mode prev_corrections already holds the
                # last-transmitted (rate-limited) offset — freeze it as-is.
                for key, held in prev_corrections.items():
                    if key in self.receive_variables:
                        self.receive_variables[key] = dict(held)
                logging.critical(
                    "E-stop engaged: corrections %s at source",
                    "zeroed" if self.rsi_mode == 'relative' else "frozen"
                )
            except Exception as e:
                logging.error("E-stop source clear failed: %s", e)
        return active

    def _build_clamp_table(self) -> None:
        """Precompute (parent, key, lo, hi) clamp entries from safety limits."""
        table = []
        for path, (lo, hi) in self.safety_manager.limits.items():
            if "." in path:
                parent, key = path.split(".", 1)
                table.append((parent, key, float(lo), float(hi)))
            else:
                table.append((path, None, float(lo), float(hi)))
        self._clamp_table = table

    def _enforce_limits(self, robot_in: dict) -> None:
        """
        Clamp outgoing values to safety limits at send time.

        Clamps (never raises — the loop must answer every cycle). This closes
        the bypass where direct writes to receive_variables, or values written
        before a limit was tightened, would stream to the robot unchecked.
        Logging is throttled to at most one line per second per path.
        """
        if not self._clamp_table or self.safety_manager.override:
            return

        for parent, key, lo, hi in self._clamp_table:
            if key is None:
                val = robot_in.get(parent)
                if isinstance(val, (int, float)) and not isinstance(val, bool):
                    if val < lo or val > hi:
                        robot_in[parent] = min(max(val, lo), hi)
                        self._log_clamp(parent, val)
            else:
                sub = robot_in.get(parent)
                if isinstance(sub, dict) and key in sub:
                    val = sub[key]
                    if isinstance(val, (int, float)) and not isinstance(val, bool):
                        if val < lo or val > hi:
                            sub[key] = min(max(val, lo), hi)
                            self._log_clamp(f"{parent}.{key}", val)

    def _log_clamp(self, path: str, value: float) -> None:
        now = time.monotonic()
        last, suppressed = self._clamp_log_state.get(path, (0.0, 0))
        if now - last >= 1.0:
            msg = "Safety clamp: %s=%s exceeded limits and was clamped"
            if suppressed:
                msg += f" ({suppressed} similar clamps suppressed)"
            logging.warning(msg, path, value)
            self._clamp_log_state[path] = (now, 0)
        else:
            self._clamp_log_state[path] = (last, suppressed + 1)

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
                    # Back-compat: E-stop is normally delivered via the shared
                    # Value directly (<=1 cycle); the command is redundant.
                    self.estop_active.value = True
                elif action == 'estop_reset':
                    self.estop_active.value = False
                elif action == 'set_limit':
                    self.safety_manager.set_limit(cmd['path'], cmd['min'], cmd['max'])
                    self._build_clamp_table()
                elif action == 'set_override':
                    self.safety_manager.override = bool(cmd.get('enable'))

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
            if corr_key not in robot_in or not isinstance(robot_in[corr_key], dict):
                continue

            corr = robot_in[corr_key]
            prev_corr = prev.get(corr_key, {})

            if self.rsi_mode == 'relative':
                if max_rate <= 0:
                    continue
                # Each value is a per-cycle delta — clamp directly
                for axis in corr:
                    if axis in axis_set:
                        val = corr[axis]
                        corr[axis] = max(-max_rate, min(max_rate, val))
            else:
                # Absolute mode — track transmitted offsets (needed for
                # E-stop freeze) and, when enabled, ramp toward the target.
                for axis in corr:
                    if axis in axis_set:
                        target = corr[axis]
                        if max_rate > 0:
                            previous = prev_corr.get(axis, 0.0)
                            delta = target - previous
                            clamped_delta = max(-max_rate, min(max_rate, delta))
                            corr[axis] = previous + clamped_delta
                        prev_corr[axis] = corr[axis]

            robot_in[corr_key] = corr
            prev[corr_key] = prev_corr

    # ------------------------------------------------------------------ metrics / logging

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
            except Exception:
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
            except Exception:
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

    # ------------------------------------------------------------------ helpers

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
