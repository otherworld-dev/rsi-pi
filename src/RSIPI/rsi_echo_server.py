import copy
import socket
import time
import xml.etree.ElementTree as ET
import logging
import threading
from .config_parser import ConfigParser

# Toggle logging for debugging purposes
LOGGING_ENABLED = True

if LOGGING_ENABLED:
    logging.basicConfig(
        filename="echo_server.log",
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

# Maps correction tags from client <Sen> XML to robot state tags in <Rob> XML
CORRECTION_TO_STATE = {
    "RKorr": "RIst",
    "AKorr": "AIPos",
    "EKorr": "ELPos",
}


class EchoServer:
    """
    Simulates a KUKA RSI UDP server for testing.

    - Responds to incoming RSI correction commands.
    - Updates internal position state (absolute/relative).
    - Returns structured XML messages (like a real robot).
    """

    def __init__(self, config_file, delay_ms=4, mode="relative"):
        """
        Initialise the echo server.

        Args:
            config_file (str): Path to RSI EthernetConfig.xml.
            delay_ms (int): Delay between messages in milliseconds.
            mode (str): Correction mode ("relative" or "absolute").
        """
        self.config = ConfigParser(config_file)
        network_settings = self.config.get_network_settings()

        self.server_address = ("0.0.0.0", 50000)  # Local bind
        self.client_address = ("127.0.0.1", network_settings["port"])  # Client to echo back to
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(self.server_address)

        self.last_received = None
        self.ipoc_value = 123456
        self.delay_ms = delay_ms / 1000  # Convert to seconds
        self.mode = mode.lower()

        # Build internal state from config send_variables (what the robot sends out).
        # Deep copy so mutations to self.state don't affect the parser's data.
        self.state = copy.deepcopy(self.config.send_variables)

        # Ensure IPOC is managed separately (we increment it ourselves)
        self.state.pop("IPOC", None)

        self.running = True
        self.thread = threading.Thread(target=self.send_message, daemon=True)

        logging.info(f"Echo Server started on {self.server_address}")
        print(f"Echo Server started in {self.mode.upper()} mode.")

    def receive_and_process(self):
        """
        Handles one incoming UDP message and updates the internal state accordingly.
        Supports correction tags (RKorr->RIst, AKorr->AIPos, EKorr->ELPos),
        scalar state updates (DiO, DiL, etc.), and IPOC synchronisation.
        """
        try:
            self.udp_socket.settimeout(self.delay_ms)
            data, addr = self.udp_socket.recvfrom(1024)
            xml_string = data.decode()
            root = ET.fromstring(xml_string)
            self.last_received = xml_string

            for elem in root:
                tag = elem.tag

                if tag in CORRECTION_TO_STATE:
                    # Apply correction (RKorr/AKorr/EKorr) to corresponding state variable
                    state_key = CORRECTION_TO_STATE[tag]
                    if state_key in self.state and isinstance(self.state[state_key], dict):
                        for axis, value in elem.attrib.items():
                            if axis in self.state[state_key]:
                                value = float(value)
                                if self.mode == "relative":
                                    self.state[state_key][axis] += value
                                else:
                                    self.state[state_key][axis] = value

                elif tag == "IPOC":
                    self.ipoc_value = int(elem.text.strip())

                elif tag in self.state:
                    # Update scalar state values (DiO, DiL, etc.)
                    if isinstance(self.state[tag], dict):
                        # Structured variable sent as attributes
                        for attr, value in elem.attrib.items():
                            if attr in self.state[tag]:
                                self.state[tag][attr] = float(value)
                    elif isinstance(self.state[tag], (int, float)):
                        self.state[tag] = int(elem.text.strip()) if isinstance(self.state[tag], int) else float(elem.text.strip())

            logging.debug(f"Processed input: {ET.tostring(root).decode()}")
        except socket.timeout:
            pass  # No data within delay window
        except ConnectionResetError:
            print("Connection was reset by client. Waiting before retry...")
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] Failed to process input: {e}")

    def generate_message(self):
        """
        Creates a reply XML message based on current state.
        Format matches KUKA RSI's expected response structure.
        Iterates over all state variables from the config's send_variables.
        """
        root = ET.Element("Rob", Type="KUKA")

        for key, value in self.state.items():
            if isinstance(value, dict):
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

    def send_message(self):
        """
        Main loop to receive input, update state, and send reply.
        Runs in a background thread until stopped.
        """
        while self.running:
            try:
                self.receive_and_process()
                response = self.generate_message()
                self.udp_socket.sendto(response.encode(), self.client_address)
                self.ipoc_value += 4
                time.sleep(self.delay_ms)
            except Exception as e:
                print(f"[ERROR] EchoServer error: {e}")
                time.sleep(1)

    def start(self):
        """Starts the echo server loop in a background thread."""
        self.running = True
        self.thread.start()

    def stop(self):
        """Stops the echo server and cleans up the socket."""
        print("Stopping Echo Server...")
        self.running = False
        self.thread.join()
        self.udp_socket.close()
        print("✅ Echo Server Stopped.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Run Echo Server for RSI Simulation")
    parser.add_argument("--config", type=str, default="RSI_EthernetConfig.xml", help="Path to RSI config file")
    parser.add_argument("--mode", type=str, choices=["relative", "absolute"], default="relative", help="Correction mode")
    parser.add_argument("--delay", type=int, default=4, help="Delay between messages in ms")

    args = parser.parse_args()
    server = EchoServer(config_file=args.config, delay_ms=args.delay, mode=args.mode)

    try:
        server.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
