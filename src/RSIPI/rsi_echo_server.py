import copy
import socket
import time
import xml.etree.ElementTree as ET
import logging
import threading
from pathlib import Path
from .config_parser import ConfigParser
from .rsi_limit_parser import parse_ethernet_timeout

# Toggle logging for debugging purposes
LOGGING_ENABLED = True

# Run artefacts belong in logs/, not the repo root - the same place
# logging_api writes its CSVs. basicConfig will not create the directory
# itself and raises if it is missing, so make it first.
LOG_DIR = Path("logs")

if LOGGING_ENABLED:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        filename=str(LOG_DIR / "echo_server.log"),
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

# Maps correction tags from client <Sen> XML to robot state tags in <Rob> XML.
# EIPos (external axis actual positions) is the real KUKA keyword; the
# AXISCORREXT object applies EKorr corrections to external axes.
CORRECTION_TO_STATE = {
    "RKorr": "RIst",
    "AKorr": "AIPos",
    "EKorr": "EIPos",
}

# A real controller advances the setpoint alongside the actual position, so
# a corrected axis shows up in both. Mirroring keeps offline tests honest:
# without it, anything reading the setpoint (get_current_joints -> ASPos)
# sees zeros while the correction is plainly being applied.
STATE_TO_SETPOINT = {
    "RIst": "RSol",
    "AIPos": "ASPos",
    "EIPos": "ESPos",
}

# Digital outputs the sensor writes come straight back on the robot's own
# read-back channel (MAP2DIGOUT sets $OUT[...]; a DIGOUT over the same range
# reports it). Mirroring makes the offline I/O check meaningful.
WRITE_TO_READBACK = {
    "DiO": "DoutW",
}

# Faulty-packet budget used when neither an explicit value nor a .rsi file is given.
# Matches the ETHERNET object's default Timeout parameter in the shipped configs.
DEFAULT_TIMEOUT_PACKETS = 100

# Log the first N invalid packets in full, then throttle to one summary line per second.
FAULTY_LOG_DETAIL_LIMIT = 5
FAULTY_LOG_SUMMARY_INTERVAL = 1.0


class EchoServer:
    """
    Emulates a KUKA RSI controller for offline testing.

    Behaves like the robot side of the link:
      - Owns the IPOC clock: it increments IPOC itself every cycle and never
        adopts a value supplied by the client.
      - Validates every <Sen> reply (IPOC echo + SENTYPE) and drops the
        corrections carried by invalid packets.
      - Counts late/invalid packets, reports them via DEF_Delay, and "breaks
        off" RSI once the ETHERNET Timeout budget is exceeded.
    """

    def __init__(self, config_file, delay_ms=4, mode="relative", rsi_file=None,
                 timeout_packets=None, onlysend=None):
        """
        Initialise the echo server.

        Args:
            config_file (str): Path to RSI EthernetConfig.xml.
            delay_ms (int): Interpolation cycle in milliseconds (4 or 12 on a real controller).
                            Doubles as the IPOC increment per cycle.
            mode (str): Correction mode ("relative" or "absolute").
            rsi_file (str): Optional path to a .rsi / .rsi.xml signal-flow file. Its ETHERNET
                            object's Timeout parameter is used as the faulty-packet budget.
            timeout_packets (int): Explicit faulty-packet budget; overrides rsi_file.
            onlysend (bool): ONLYSEND mode - stream <Rob> data, expect no replies.
                             Default: taken from the config's ONLYSEND setting.
        """
        self.config = ConfigParser(config_file)
        network_settings = self.config.get_network_settings()

        # ONLYSEND: robot streams data, sensor sends nothing back
        if onlysend is None:
            onlysend = bool(network_settings.get("onlysend"))
        self.onlysend = bool(onlysend)

        # Late-packet behavior per RECEIVE element (dotted tag -> 0/1, default hold)
        self.holdon_map = dict(getattr(self.config, "holdon_map", {}))
        # Last valid correction values, for HOLDON emulation on late cycles
        self.last_corrections = {}
        self._consecutive_late = 0

        self.server_address = ("0.0.0.0", 50000)  # Local bind
        # A real controller transmits to the sensor's configured IP_NUMBER, and
        # the client binds that IP whenever it exists on this machine (e.g. the
        # RSI adapter is plugged in). Send there in that case - local delivery
        # works for any local interface - and fall back to loopback otherwise
        # so offline testing works without the RSI network configured.
        self.client_address = (self._local_target_ip(network_settings.get("ip")),
                               network_settings["port"])
        self.sentype = network_settings.get("sentype")
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(self.server_address)

        self.last_received = None

        # --- Clock ownership -------------------------------------------------
        # The controller owns IPOC: it increments it by one interpolation cycle
        # every pass and the sensor must echo the received value unchanged.
        self.ipoc_value = 123456
        self.ipoc_increment = int(delay_ms)  # ms per cycle, captured before the /1000 conversion
        self.last_sent_ipoc = None  # IPOC carried by the packet we most recently transmitted

        self.delay_ms = delay_ms / 1000  # Convert to seconds
        self.mode = mode.lower()

        # --- Faulty packet accounting ---------------------------------------
        self.faulty_packets = 0
        self._faulty_logged = 0
        self._last_faulty_summary = 0.0
        self._faulty_since_summary = 0
        self.timeout_packets = self._resolve_timeout_budget(rsi_file, timeout_packets)

        # Build internal state from config send_variables (what the robot sends out).
        # Deep copy so mutations to self.state don't affect the parser's data.
        self.state = copy.deepcopy(self.config.send_variables)

        # Ensure IPOC is managed separately (we increment it ourselves)
        self.state.pop("IPOC", None)

        # Reference pose captured at start-up. In absolute mode a correction is
        # applied relative to this pose (RKorr is a correction, never a world pose).
        self.base_pose = {key: copy.deepcopy(value)
                          for key, value in self.state.items()
                          if isinstance(value, dict)}

        self.running = True
        self.thread = threading.Thread(target=self.send_message, daemon=True)

        logging.info(f"Echo Server started on {self.server_address} "
                     f"(SENTYPE={self.sentype}, IPOC step={self.ipoc_increment}, "
                     f"faulty packet budget={self.timeout_packets})")
        print(f"Echo Server started in {self.mode.upper()} mode.")

    # ------------------------------------------------------------------ setup

    @staticmethod
    def _local_target_ip(ip):
        """The configured sensor IP if it is bindable on this host, else loopback."""
        if not ip or ip in ("0.0.0.0", "127.0.0.1"):
            return "127.0.0.1"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind((ip, 0))
            return ip
        except OSError:
            return "127.0.0.1"

    def _resolve_timeout_budget(self, rsi_file, timeout_packets):
        """
        Resolve the faulty-packet budget.

        Precedence: explicit timeout_packets > ETHERNET Timeout in rsi_file > default.
        """
        if timeout_packets is not None:
            return int(timeout_packets)

        if rsi_file:
            parsed = parse_ethernet_timeout(rsi_file)
            if parsed is not None:
                logging.info(f"Using ETHERNET Timeout={parsed} from {rsi_file}")
                return int(parsed)
            logging.warning(f"No ETHERNET Timeout found in {rsi_file}; "
                            f"falling back to {DEFAULT_TIMEOUT_PACKETS}")

        return DEFAULT_TIMEOUT_PACKETS

    # -------------------------------------------------------------- reception

    def receive_and_process(self):
        """
        Reads one incoming UDP datagram (if any) and hands it to process_reply().
        """
        try:
            self.udp_socket.settimeout(self.delay_ms)
            # 64KB = UDP maximum; Full-config telegrams exceed 1KB and Windows
            # raises WinError 10040 on undersized buffers.
            data, addr = self.udp_socket.recvfrom(65535)
        except socket.timeout:
            # DELIBERATE DIVERGENCE FROM A REAL CONTROLLER:
            # a real robot counts a missing reply as a late packet. This server
            # starts transmitting before any client attaches, so every cycle
            # before connection would otherwise be scored as faulty and break
            # RSI off immediately. Receive timeouts therefore do NOT count.
            return None
        except ConnectionResetError:
            print("Connection was reset by client. Waiting before retry...")
            time.sleep(0.5)
            return None
        except Exception as e:
            print(f"[ERROR] Failed to read input: {e}")
            return None

        if self.onlysend:
            # ONLYSEND: the controller expects no replies at all - anything
            # received is spurious and neither applied nor counted as faulty.
            return None

        try:
            xml_string = data.decode(errors="replace")
        except Exception as e:
            print(f"[ERROR] Failed to decode input: {e}")
            return None

        return self.process_reply(xml_string)

    def process_reply(self, xml_string):
        """
        Parse, validate and (only if valid) apply one <Sen> reply.

        A reply is rejected when:
          - it cannot be parsed as XML,
          - the <Sen> Type attribute does not match the configured SENTYPE,
          - its IPOC is not the exact value we last transmitted.

        Rejected packets increment the faulty counter and their corrections and
        state updates are discarded in full.

        Args:
            xml_string (str): Raw <Sen> XML from the client.

        Returns:
            bool: True if the packet was accepted and applied, False otherwise.
        """
        self.last_received = xml_string

        # --- Parse -----------------------------------------------------------
        try:
            root = ET.fromstring(xml_string)
        except ET.ParseError as e:
            self._register_faulty_packet(f"malformed XML ({e})")
            return False

        sen_type = root.attrib.get("Type")

        reply_ipoc = None
        for elem in root:
            if elem.tag == "IPOC":
                text = (elem.text or "").strip()
                try:
                    reply_ipoc = int(text)
                except ValueError:
                    reply_ipoc = None
                break

        # --- Validate --------------------------------------------------------
        if self.sentype is not None and sen_type != self.sentype:
            self._register_faulty_packet(
                f"SENTYPE mismatch (expected {self.sentype!r}, got {sen_type!r})")
            return False

        # last_sent_ipoc is None only before the first transmission; nothing to
        # echo yet, so any timestamp is tolerated.
        if self.last_sent_ipoc is not None and reply_ipoc != self.last_sent_ipoc:
            self._register_faulty_packet(
                f"IPOC mismatch (expected {self.last_sent_ipoc}, got {reply_ipoc})")
            return False

        # --- Apply -----------------------------------------------------------
        for elem in root:
            tag = elem.tag

            if tag == "IPOC":
                # The controller owns the clock - never adopt the client's value.
                continue

            if tag in CORRECTION_TO_STATE:
                # Apply correction (RKorr/AKorr/EKorr) to corresponding state variable
                state_key = CORRECTION_TO_STATE[tag]
                if state_key in self.state and isinstance(self.state[state_key], dict):
                    base = self.base_pose.get(state_key, {})
                    last = self.last_corrections.setdefault(tag, {})
                    for axis, value in elem.attrib.items():
                        if axis in self.state[state_key]:
                            try:
                                value = float(value)
                            except (TypeError, ValueError):
                                continue
                            last[axis] = value  # remembered for HOLDON emulation
                            if self.mode == "relative":
                                self.state[state_key][axis] += value
                            else:
                                # Absolute mode: correction is applied against the
                                # start pose, not treated as a world coordinate.
                                self.state[state_key][axis] = base.get(axis, 0.0) + value
                            self._mirror_to_setpoint(state_key, axis)

            elif tag in self.state:
                # Update scalar state values (DiO, DiL, etc.)
                if isinstance(self.state[tag], dict):
                    # Structured variable sent as attributes
                    for attr, value in elem.attrib.items():
                        if attr in self.state[tag]:
                            try:
                                self.state[tag][attr] = float(value)
                            except (TypeError, ValueError):
                                continue
                elif isinstance(self.state[tag], (int, float)):
                    text = (elem.text or "").strip()
                    try:
                        self.state[tag] = int(text) if isinstance(self.state[tag], int) else float(text)
                    except ValueError:
                        pass

            if tag in WRITE_TO_READBACK:
                # Not necessarily in self.state (it is a RECEIVE variable),
                # so read the value straight off the element.
                twin = WRITE_TO_READBACK[tag]
                if twin in self.state:
                    try:
                        self.state[twin] = int(float((elem.text or "0").strip()))
                    except ValueError:
                        pass

        logging.debug(f"Processed input: {ET.tostring(root).decode()}")
        self._consecutive_late = 0
        return True

    def _register_faulty_packet(self, reason):
        """
        Count an invalid packet, log it (throttled) and break off RSI when the
        controller's Timeout budget is exceeded.
        """
        self.faulty_packets += 1
        self._faulty_since_summary += 1

        if self._faulty_logged < FAULTY_LOG_DETAIL_LIMIT:
            self._faulty_logged += 1
            self._faulty_since_summary = 0
            message = (f"Rejected packet #{self.faulty_packets}: {reason}")
            logging.warning(message)
            print(f"[WARN] {message}")
        else:
            now = time.monotonic()
            if now - self._last_faulty_summary >= FAULTY_LOG_SUMMARY_INTERVAL:
                message = (f"{self._faulty_since_summary} further packet(s) rejected "
                           f"(total {self.faulty_packets}/{self.timeout_packets}); "
                           f"latest: {reason}")
                logging.warning(message)
                print(f"[WARN] {message}")
                self._last_faulty_summary = now
                self._faulty_since_summary = 0

        if self.faulty_packets > self.timeout_packets:
            message = (f"RSI BROKE OFF: {self.faulty_packets} faulty packets exceeded "
                       f"Timeout={self.timeout_packets}")
            logging.error(message)
            print(f"[ERROR] {message}")
            self.running = False

    def _mirror_to_setpoint(self, state_key, axis):
        """Keep the setpoint twin (RSol/ASPos/ESPos) in step with the actual."""
        twin = STATE_TO_SETPOINT.get(state_key)
        if twin and isinstance(self.state.get(twin), dict) and axis in self.state[twin]:
            self.state[twin][axis] = self.state[state_key][axis]

    def _apply_holdon_late_cycle(self):
        """
        Emulate controller HOLDON semantics for a cycle with no valid reply.

        Per the RSI manual (late packets are rejected; each RECEIVE element's
        HOLDON attribute governs its ETHERNET object output):
          - HOLDON=1: the last valid value REMAINS at the output. In relative
            mode that means the old delta keeps integrating; in absolute mode
            the held offset simply stays applied.
          - HOLDON=0: the output is reset to 0. In relative mode nothing is
            applied this cycle; in absolute mode the axis returns to its
            base (zero-correction) pose.

        Held values stop being re-applied once the run of consecutive late
        cycles exceeds the Timeout budget - a real controller would have
        broken off RSI by then (this server tolerates silence by design, so
        the cap bounds the drift at what real hardware would allow).
        """
        self._consecutive_late += 1
        if self._consecutive_late > self.timeout_packets:
            return
        for tag, last in self.last_corrections.items():
            state_key = CORRECTION_TO_STATE.get(tag)
            if state_key not in self.state or not isinstance(self.state[state_key], dict):
                continue
            base = self.base_pose.get(state_key, {})
            for axis, value in last.items():
                if axis not in self.state[state_key]:
                    continue
                hold = self.holdon_map.get(f"{tag}.{axis}", 1)
                if self.mode == "relative":
                    if hold:
                        self.state[state_key][axis] += value
                else:
                    if not hold:
                        self.state[state_key][axis] = base.get(axis, 0.0)

    # ------------------------------------------------------------ transmission

    def _update_delay_report(self):
        """
        Publish the late/invalid packet count in DEF_Delay, exactly as a real
        controller streams it back to the sensor as <Delay D="n"/>.
        """
        delay = self.state.get("Delay")
        if isinstance(delay, dict):
            delay["D"] = self.faulty_packets

    def generate_message(self):
        """
        Creates a reply XML message based on current state.
        Format matches KUKA RSI's expected response structure.
        Iterates over all state variables from the config's send_variables.
        """
        # Called once per cycle, immediately before the message is built.
        self._update_delay_report()

        root = ET.Element("Rob", Type="KUKA")

        for key, value in self.state.items():
            if key == "Delay" and isinstance(value, dict):
                # DEF_Delay is a packet counter - emitted as a plain integer.
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, str(int(sub_value)))
            elif isinstance(value, dict):
                # Structured variable (RIst, AIPos, etc.) -> XML attributes
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, f"{float(sub_value):.2f}")
            elif isinstance(value, bool):
                ET.SubElement(root, key).text = "1" if value else "0"
            elif isinstance(value, (int, float)):
                ET.SubElement(root, key).text = str(value)
            elif isinstance(value, str):
                ET.SubElement(root, key).text = value

        ET.SubElement(root, "IPOC").text = str(self.ipoc_value)
        return ET.tostring(root, encoding="utf-8").decode()

    def send_once(self):
        """
        Transmits one <Rob> packet and advances the IPOC clock by one cycle.

        Returns:
            str: The XML that was transmitted.
        """
        response = self.generate_message()
        self.last_sent_ipoc = self.ipoc_value
        self.udp_socket.sendto(response.encode(), self.client_address)
        self.ipoc_value += self.ipoc_increment
        return response

    def send_message(self):
        """
        Main loop to receive input, update state, and send reply.
        Runs in a background thread until stopped.
        """
        while self.running:
            try:
                accepted = self.receive_and_process()
                if not self.running:
                    break  # RSI broke off while processing the last reply
                if not self.onlysend and accepted is not True:
                    # No valid reply this cycle - HOLDON semantics apply
                    self._apply_holdon_late_cycle()
                self.send_once()
                time.sleep(self.delay_ms)
            except Exception as e:
                print(f"[ERROR] EchoServer error: {e}")
                time.sleep(1)

    # -------------------------------------------------------------- lifecycle

    def start(self):
        """Starts the echo server loop in a background thread."""
        self.running = True
        self.thread.start()

    def stop(self):
        """Stops the echo server and cleans up the socket."""
        print("Stopping Echo Server...")
        self.running = False
        if self.thread.ident is not None:
            self.thread.join()
        self.udp_socket.close()
        print(f"Faulty packets: {self.faulty_packets} (Timeout budget {self.timeout_packets})")
        print("Echo Server Stopped.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Run Echo Server for RSI Simulation")
    parser.add_argument("--config", type=str, default="RSI_EthernetConfig.xml", help="Path to RSI config file")
    parser.add_argument("--mode", type=str, choices=["relative", "absolute"], default="relative", help="Correction mode")
    parser.add_argument("--delay", type=int, default=4, help="Interpolation cycle / IPOC step in ms")
    parser.add_argument("--rsi-file", type=str, default=None,
                        help="Path to a .rsi/.rsi.xml signal flow file; its ETHERNET Timeout "
                             "parameter sets the faulty packet budget")
    parser.add_argument("--timeout-packets", type=int, default=None,
                        help="Faulty packet budget before RSI breaks off (overrides --rsi-file)")
    parser.add_argument("--onlysend", action="store_true", default=None,
                        help="ONLYSEND mode: stream robot data, expect no replies "
                             "(default: taken from the config's ONLYSEND setting)")

    args = parser.parse_args()
    server = EchoServer(
        config_file=args.config,
        delay_ms=args.delay,
        mode=args.mode,
        rsi_file=args.rsi_file,
        timeout_packets=args.timeout_packets,
        onlysend=args.onlysend,
    )

    try:
        server.start()
        while server.running:
            time.sleep(1)
        server.stop()
    except KeyboardInterrupt:
        server.stop()
