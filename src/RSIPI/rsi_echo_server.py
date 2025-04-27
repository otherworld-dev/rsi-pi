import socket
import time
import xml.etree.ElementTree as ET
import logging
import threading
from src.RSIPI.rsi_config import RSIConfig

# ✅ Toggle logging for debugging purposes
LOGGING_ENABLED = True

if LOGGING_ENABLED:
    logging.basicConfig(
        filename="echo_server.log",
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )


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
        self.config = RSIConfig(config_file)
        network_settings = self.config.get_network_settings()

        self.server_address = ("0.0.0.0", 50000)  # Local bind
        self.client_address = ("127.0.0.1", network_settings["port"])  # Client to echo back to
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(self.server_address)

        self.last_received = None
        self.ipoc_value = 123456
        self.delay_ms = delay_ms / 1000  # Convert to seconds
        self.mode = mode.lower()

        # Internal state to simulate robot values
        self.state = {
            "RIst": {k: 0.0 for k in ["X", "Y", "Z", "A", "B", "C"]},
            "AIPos": {f"A{i}": 0.0 for i in range(1, 7)},
            "ELPos": {f"E{i}": 0.0 for i in range(1, 7)},
            "DiO": 0,
            "DiL": 0
        }

        self.running = True
        self.thread = threading.Thread(target=self.send_message, daemon=True)

        logging.info(f"Echo Server started on {self.server_address}")
        print(f"Echo Server started in {self.mode.upper()} mode.")

    def receive_and_process(self):
        """
        Handles one incoming UDP message and updates the internal state accordingly.
        Supports RKorr, AKorr, DiO, DiL, and IPOC updates.
        """
        try:
            self.udp_socket.settimeout(self.delay_ms)
            data, addr = self.udp_socket.recvfrom(1024)
            xml_string = data.decode()
            root = ET.fromstring(xml_string)
            self.last_received = xml_string

            for elem in root:
                tag = elem.tag
                if tag in ["RKorr", "AKorr"]:
                    for axis, value in elem.attrib.items():
                        value = float(value)
                        if tag == "RKorr" and axis in self.state["RIst"]:
                            # Apply Cartesian correction
                            if self.mode == "relative":
                                self.state["RIst"][axis] += value
                            else:
                                self.state["RIst"][axis] = value
                        elif tag == "AKorr" and axis in self.state["AIPos"]:
                            # Apply joint correction
                            if self.mode == "relative":
                                self.state["AIPos"][axis] += value
                            else:
                                self.state["AIPos"][axis] = value
                elif tag in ["DiO", "DiL"]:
                    if tag in self.state:
                        self.state[tag] = int(elem.text.strip())
                elif tag == "IPOC":
                    self.ipoc_value = int(elem.text.strip())

            logging.debug(f"Processed input: {ET.tostring(root).decode()}")
        except socket.timeout:
            pass  # No data within delay window
        except ConnectionResetError:
            print("⚠️ Connection was reset by client. Waiting before retry...")
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] Failed to process input: {e}")

    def generate_message(self):
        """
        Creates a reply XML message based on current state.
        Format matches KUKA RSI's expected response structure.
        """
        root = ET.Element("Rob", Type="KUKA")

        for key in ["RIst", "AIPos", "ELPos"]:
            element = ET.SubElement(root, key)
            for sub_key, value in self.state[key].items():
                element.set(sub_key, f"{value:.2f}")

        for key in ["DiO", "DiL"]:
            ET.SubElement(root, key).text = str(self.state[key])

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
