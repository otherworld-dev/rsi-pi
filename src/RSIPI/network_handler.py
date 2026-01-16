import multiprocessing
import socket
import logging
import xml.etree.ElementTree as ET
import os
import datetime
from queue import Empty
from .xml_handler import XMLGenerator
from .safety_manager import SafetyManager


class CSVLogger(multiprocessing.Process):
    """Separate process for writing CSV logs without blocking the network loop."""

    def __init__(self, log_queue, stop_event, filename):
        super().__init__()
        self.log_queue = log_queue
        self.stop_event = stop_event
        self.filename = filename
        self.daemon = True

    def run(self):
        """Write log entries from queue to CSV file."""
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
    """Handles UDP communication and optional CSV logging in a separate process."""

    def __init__(self, ip, port, send_variables, receive_variables, stop_event, config_parser, start_event, command_queue):
        super().__init__()
        self.send_variables = send_variables
        self.receive_variables = receive_variables
        self.stop_event = stop_event
        self.start_event = start_event
        self.config_parser = config_parser
        self.command_queue = command_queue
        self.safety_manager = SafetyManager(config_parser.safety_limits)

        self.client_address = (ip, port)
        self.logging_active = multiprocessing.Value('b', False)

        self.controller_ip_and_port = None
        self.udp_socket = None

        # Logging infrastructure (created when logging starts)
        self.log_queue = None
        self.log_stop_event = None
        self.csv_logger = None

    def run(self):
        """Start the network loop."""
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

    def _setup_socket(self):
        """Create and bind the UDP socket."""
        if not self.is_valid_ip(self.client_address[0]):
            logging.warning(f"Invalid IP address '{self.client_address[0]}'. Falling back to '0.0.0.0'.")
            self.client_address = ('0.0.0.0', self.client_address[1])

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(self.client_address)
        logging.info(f"Network process bound on {self.client_address}")

    def _run_loop(self):
        """Main communication loop."""
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

    def _process_commands(self):
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

    def _queue_log_entry(self):
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

    def _start_logging(self, filename):
        """Start CSV logging to the specified file."""
        if self.logging_active.value:
            logging.warning("Logging already active")
            return

        self.log_queue = multiprocessing.Queue(maxsize=1000)
        self.log_stop_event = multiprocessing.Event()

        self.csv_logger = CSVLogger(self.log_queue, self.log_stop_event, filename)
        self.csv_logger.start()

        self.logging_active.value = True
        logging.info(f"CSV logging started: {filename}")

    def _stop_logging(self):
        """Stop CSV logging."""
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

    def _cleanup(self):
        """Clean up resources."""
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
    def is_valid_ip(ip):
        try:
            socket.inet_aton(ip)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.bind((ip, 0))
            return True
        except (socket.error, OSError):
            return False

    def process_received_data(self, xml_string):
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
        except Exception as e:
            logging.error(f"Error parsing received message: {e}")
