import multiprocessing
import socket
import logging
import xml.etree.ElementTree as ET
from .xml_handler import XMLGenerator
from .safety_manager import SafetyManager

class NetworkProcess(multiprocessing.Process):
    """Handles UDP communication and optional CSV logging in a separate process."""

    def __init__(self, ip, port, send_variables, receive_variables, stop_event, config_parser, start_event):
        super().__init__()
        self.send_variables = send_variables
        self.receive_variables = receive_variables
        self.stop_event = stop_event
        self.start_event = start_event  # ✅ NEW
        self.config_parser = config_parser
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.safety_manager = SafetyManager(config_parser.safety_limits)

        self.client_address = (ip, port)
        self.logging_active = multiprocessing.Value('b', False)
        self.log_filename = multiprocessing.Array('c', 256)
        self.csv_process = None

        self.controller_ip_and_port = None

    def run(self):
        """Start the network loop."""
        self.start_event.wait()  # ✅ Wait until RSIClient sends start signal

        try:
            if not self.is_valid_ip(self.client_address[0]):
                logging.warning(f"Invalid IP address '{self.client_address[0]}'. Falling back to '0.0.0.0'.")
                self.client_address = ('0.0.0.0', self.client_address[1])

            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind(self.client_address)
            logging.info(f"✅ Network process bound on {self.client_address}")

        except OSError as e:
            logging.error(f"❌ Failed to bind to {self.client_address}: {e}")
            raise

        while not self.stop_event.is_set():
            try:
                self.udp_socket.settimeout(5)
                data_received, self.controller_ip_and_port = self.udp_socket.recvfrom(1024)
                message = data_received.decode()
                self.process_received_data(message)
                send_xml = XMLGenerator.generate_send_xml(self.send_variables, self.config_parser.network_settings)
                self.udp_socket.sendto(send_xml.encode(), self.controller_ip_and_port)

                if self.logging_active.value:
                    self.log_to_csv()

            except socket.timeout:
                logging.error("[WARNING] No message received within timeout period.")
            except Exception as e:
                logging.error(f"[ERROR] Network process error: {e}")

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
            logging.error(f"[ERROR] Error parsing received message: {e}")
