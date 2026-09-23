import multiprocessing
import select
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
from .shared_variables import SharedVariables
from .exceptions import RSINetworkError, RSITimeoutError, RSIPacketError, RSILoggingError
from .timing_metrics import TimingMetrics


def _coerce_like(current: Any, text: Optional[str]) -> Any:
    """Parse *text* into whatever type *current* already is.

    The config's TYPE decides a variable's Python type (BOOL -> bool,
    LONG -> int, DOUBLE -> float, STRING -> str), so the existing value is a
    reliable template for what arrives from the robot.
    """
    text = "" if text is None else text.strip()
    if isinstance(current, bool):
        # Judge by VALUE. A controller may well send a Bool as "0.00" or
        # "1.000000" (Precision applies to everything it sends), and a
        # string comparison would read "0.00" as True.
        try:
            return float(text) != 0.0
        except ValueError:
            return text.lower() in ("true", "1")
    if isinstance(current, int):
        try:
            return int(float(text))
        except ValueError:
            return current
    if isinstance(current, float):
        try:
            return float(text)
        except ValueError:
            return current
    return text


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
    # EKorr is a correction too. Leaving it out meant an E-stop froze RKorr
    # and AKorr but kept transmitting a held external-axis correction.
    _CORRECTION_KEYS = ('RKorr', 'AKorr', 'EKorr')
    _CONSECUTIVE_ERROR_LIMIT = 250  # ~1s of continuous hot-loop failures

    def __init__(
        self,
        ip: str,
        port: int,
        send_variables: Any,  # SharedVariables (a dict or Manager dict also works, slower)
        receive_variables: Any,  # SharedVariables (likewise)
        stop_event: multiprocessing.Event,
        config_parser: Any,  # ConfigParser type
        start_event: multiprocessing.Event,
        command_queue: multiprocessing.Queue,
        metrics_dict: Optional[Any] = None,  # SharedVariables (or any dict)
        connected_event: Optional[multiprocessing.Event] = None,
        rsi_mode: str = 'relative',
        max_cartesian_rate: float = 0.0,
        max_joint_rate: float = 0.0,
        cycle_time: float = 0.004,
        estop_active: Optional[Any] = None,   # shared Value('b'), parent-owned
        corr_seq: Optional[Any] = None,       # only without SharedVariables, which carry the seq
        corr_ack: Optional[Any] = None,       # shared Value('q'), parent-owned
        oneshot_active: Optional[Any] = None,  # shared Value('b'), parent-owned
        ipoc_value: Optional[Any] = None,     # shared Value('q'), parent-owned
        corr_lock: Optional[Any] = None,      # only without SharedVariables
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

        # E-stop rising edge whose write to receive_variables has not landed yet
        self._estop_clear_pending: bool = False

        # Stale robot packets passed over after a stall (see _newest_packet)
        self._skipped_packets: int = 0
        self._skipped_unlogged: int = 0
        self._last_skip_log: float = 0.0

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
        # Deliberately NOT SO_REUSEADDR: on Windows it lets a second process
        # bind the same UDP port, after which packet delivery between the two
        # sockets is arbitrary - a leftover KUKA TestServer silently swallows
        # the robot's packets while RSIPI reports a healthy bind. Claim the
        # port exclusively and fail loudly instead.
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            try:
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            except OSError:
                pass
        # 1s matches the watchdog threshold; also bounds worst-case iteration
        # length so queued commands and E-stop state are never stalled for long.
        self.udp_socket.settimeout(1.0)
        try:
            self.udp_socket.bind(self.client_address)
        except OSError as e:
            raise RSINetworkError(
                f"Cannot bind UDP {self.client_address[0]}:{self.client_address[1]} - {e}. "
                "Another program is using this port (KUKA TestServer, a second RSIPI "
                "instance, or the echo server). Close it and retry."
            ) from e
        logging.info("Network process bound on %s", self.client_address)

    # ------------------------------------------------------------------ loop

    def _run_loop(self) -> None:
        """
        Main communication loop.

        Nothing in the loop waits on another process. receive_variables are
        snapshotted from shared memory; the robot's state (every cycle) and
        the metrics (every 100) are published to shared memory after the
        reply has gone, without blocking. Per-cycle pipeline: snapshot ->
        one-shot substitution -> limit clamp -> rate limit -> E-stop
        substitution -> serialize -> send -> ack -> publish robot state.
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
        local_robot_out = self.send_variables.copy()
        parent = multiprocessing.parent_process()
        if isinstance(self.receive_variables, SharedVariables):
            # The E-stop edge writes to the source from inside the loop. It
            # retries next cycle rather than hold up a reply.
            self.receive_variables.write_timeout = self.cycle_time / 2
        # A fresh process must never ack a waypoint published to a previous
        # process (reconnect); start from whatever seq the parent is at.
        last_acked_seq, local_robot_in = self._snapshot_corrections()
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

        while not self.stop_event.is_set():
            # Drain pending commands every ~10 cycles (~40ms at 4ms cycles)
            cmd_counter += 1
            if cmd_counter >= 10:
                self._process_commands()
                cmd_counter = 0
                if self._parent_gone(parent):
                    break

            try:
                # 64KB = UDP maximum; the Full config's telegrams exceed 1KB
                # and Windows raises WinError 10040 on undersized buffers.
                data_received, self.controller_ip_and_port = self.udp_socket.recvfrom(65535)
                data_received = self._newest_packet(data_received)
                message = data_received.decode()

                # Parse robot's outgoing data (ElementTree — handles any attribute order)
                try:
                    self._parse_received_data(message, local_robot_out)
                except RSIPacketError:
                    logging.warning("Parse failed, sending last known good response")

                if first_packet:
                    first_packet = False
                    # Publish the robot's state BEFORE announcing the
                    # connection: otherwise wait_for_connection() can return
                    # while get_current_pose() still reports config defaults
                    # (zeros).
                    sync_counter = self._publish_robot_state(
                        local_robot_out, sync_counter, block=True)
                    if self.connected_event:
                        self.connected_event.set()

                ipoc = local_robot_out.get("IPOC", 0)

                if not onlysend:
                    # Snapshot (seq, receive_variables) as an atomic pair:
                    # seeing seq N here guarantees the snapshot contains
                    # waypoint N's corrections.
                    snapshot_started = time.perf_counter()
                    seq, local_robot_in = self._snapshot_corrections()
                    if self.timing_metrics is not None:
                        self.timing_metrics.record_snapshot(
                            time.perf_counter() - snapshot_started)

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

                sync_counter = self._publish_robot_state(local_robot_out, sync_counter)

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
                if self._parent_gone(parent):
                    break
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

    @staticmethod
    def _parent_gone(parent: Any) -> bool:
        """True when the process that started this one has died.

        A parent killed outright leaves this process running, and it would go
        on sending the last corrections for as long as the robot asked, and
        hold the UDP port against the next run. The loop stops replying
        instead: the controller sees a comms loss and stops. A broken Manager
        pipe used to give the parent's death away, some of the time. Nothing
        here talks to a Manager now, so it is checked directly.
        """
        if parent is None or parent.is_alive():
            return False
        logging.critical("Parent process is gone - network process stops "
                         "replying to the robot")
        return True

    def _publish_robot_state(self, robot_out: dict, sync_counter: int,
                             block: bool = False) -> int:
        """Hand the robot's state to the client; returns the new sync_counter.

        Runs after the reply has gone. Shared memory takes the whole state
        every cycle without waiting: if another writer holds the lock, the
        next cycle's state goes out instead. A plain or Manager dict (tests,
        direct users) is updated every tenth cycle, as the Manager's cost
        used to require for everyone.
        """
        if isinstance(self.send_variables, SharedVariables):
            self.send_variables.replace(robot_out, block=block)
            return 0
        if block or sync_counter + 1 >= 10:
            self.send_variables.update(robot_out)
            return 0
        return sync_counter + 1

    def _snapshot_corrections(self) -> Tuple[int, Dict[str, Any]]:
        """(seq, receive_variables) as an atomic pair.

        This runs inside every reply window, so it must not talk to another
        process. SharedVariables hand the pair over from shared memory in
        microseconds. dict(manager_proxy) did it in one pipe round trip per
        key, 13 for the Drill context: 1.8 ms median and 4.3 ms worst case
        of a 4 ms window, and each publish_corrections() queued its own
        Manager calls ahead of them. On the KR C4 that was 10 % of replies
        late during a plunge, each late reply a feed step not applied.

        A plain dict or a Manager dict still works, for tests and direct
        users: copy() is one round trip where dict() was one per key.
        """
        variables = self.receive_variables
        if isinstance(variables, SharedVariables):
            return variables.snapshot()
        with self.corr_lock:
            return self.corr_seq.value, variables.copy()

    def _newest_packet(self, data: bytes) -> bytes:
        """Swap *data* for the newest robot packet already queued behind it.

        The robot sends a packet every cycle whether or not the last one was
        answered, so after a stall several are queued. All but the newest are
        already past their reply window. Answering them in turn sends replies
        the controller is certain to reject, and the first of those carries
        (and acks) any correction published meanwhile, which is then lost
        without an error. Only the newest is answered. When nothing is queued,
        the normal case, this costs one zero-timeout select().
        """
        skipped = 0
        while select.select([self.udp_socket], [], [], 0)[0]:
            try:
                data, self.controller_ip_and_port = self.udp_socket.recvfrom(65535)
            except ConnectionResetError:
                break  # a stale ICMP error on Windows; keep the packet in hand
            skipped += 1
        if skipped:
            self._log_skipped(skipped)
        return data

    def _log_skipped(self, count: int) -> None:
        """Record skipped packets; log at most one line per second."""
        self._skipped_packets += count
        self._skipped_unlogged += count
        now = time.monotonic()
        if now - self._last_skip_log >= 1.0:
            logging.warning(
                "PC fell behind the robot: answered only the newest packet, "
                "skipped %d stale one(s) (%d in total)",
                self._skipped_unlogged, self._skipped_packets)
            self._last_skip_log = now
            self._skipped_unlogged = 0

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
        if not active:
            self._estop_clear_pending = False
        elif not estop_was_active or self._estop_clear_pending:
            try:
                if self.rsi_mode == 'relative':
                    for key in prev_corrections:
                        prev_corrections[key] = {k: 0.0 for k in prev_corrections[key]}
                # In absolute mode prev_corrections already holds the
                # last-transmitted (rate-limited) offset — freeze it as-is.
                declared = self.receive_variables.keys()
                self.receive_variables.update(
                    {key: dict(held) for key, held in prev_corrections.items()
                     if key in declared})
                self._estop_clear_pending = False
                logging.critical(
                    "E-stop engaged: corrections %s at source",
                    "zeroed" if self.rsi_mode == 'relative' else "frozen"
                )
            except TimeoutError:
                # A parent thread holds the writer lock. Try again next
                # cycle; the wire carries the safe values either way.
                self._estop_clear_pending = True
            except Exception as e:
                self._estop_clear_pending = False
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
            health = self.timing_metrics.get_health_status()
            published = dict(health['stats'])
            published['is_healthy'] = health['is_healthy']
            published['warnings'] = health['warnings']
            published['watchdog_timeout'] = health['watchdog_timeout']
            published['skipped_packets'] = self._skipped_packets
            published['stale_snapshots'] = getattr(
                self.receive_variables, 'stale_snapshots', 0)
            # One write. 13 Manager writes, after computing the statistics
            # twice, used to hold the loop for a whole cycle every 100 cycles.
            if isinstance(self.metrics_dict, SharedVariables):
                self.metrics_dict.replace(published, block=False)
            else:
                self.metrics_dict.update(published)

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
                            # Dotted TYPE=BOOL tags (Digout.o1-3) are seeded
                            # False by ConfigParser and were arriving as
                            # 1.0/0.0 floats - judge those by value. INTERNAL
                            # groups (RIst, AIPos, MACur...) are seeded with
                            # int 0 placeholders, NOT a declared type, so they
                            # must stay float: coercing to the placeholder
                            # truncated 12.5 A to 12 and would do the same to
                            # every position.
                            for k, v in element.attrib.items():
                                current = existing.get(k)
                                if isinstance(current, bool):
                                    existing[k] = _coerce_like(current, v)
                                else:
                                    existing[k] = float(v)
                        else:
                            target[element.tag] = {k: float(v) for k, v in element.attrib.items()}
                    else:
                        # Keep the declared type. ConfigParser seeds each
                        # variable with a default matching its config TYPE, so
                        # coercing to what is already there turns "100" back
                        # into an int and "3.25" into a float. Storing the raw
                        # string made every scalar from the robot a str, which
                        # silently breaks comparisons and arithmetic on them.
                        target[element.tag] = _coerce_like(
                            target.get(element.tag), element.text)
                if element.tag == "IPOC":
                    target["IPOC"] = int(element.text)
        except ET.ParseError as e:
            logging.error("XML parse error in received message: %s", e)
            raise RSIPacketError(f"Failed to parse received XML: {e}") from e
        except Exception as e:
            logging.error("Error processing received message: %s", e)
            raise RSIPacketError(f"Unexpected error parsing packet: {e}") from e
