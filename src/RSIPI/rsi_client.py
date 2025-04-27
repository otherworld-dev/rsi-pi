import logging
import multiprocessing
import time
from .config_parser import ConfigParser
from .network_handler import NetworkProcess
from .safety_manager import SafetyManager
import threading

class RSIClient:
    """Main RSI API class that integrates network, config handling, and message processing."""

    def __init__(self, config_file, rsi_limits_file=None):
        logging.info(f"Loading RSI configuration from {config_file}...")

        self.config_parser = ConfigParser(config_file, rsi_limits_file)
        network_settings = self.config_parser.get_network_settings()

        self.manager = multiprocessing.Manager()
        self.send_variables = self.manager.dict(self.config_parser.send_variables)
        self.receive_variables = self.manager.dict(self.config_parser.receive_variables)
        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()  # ✅ NEW

        self.safety_manager = SafetyManager(self.config_parser.safety_limits)

        # ✅ Create NetworkProcess but don't start communication yet
        self.network_process = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event
        )
        self.network_process.start()
        self.logger = None

    def start(self):
        """Send start signal to NetworkProcess and run control loop."""
        logging.info("RSIClient sending start signal to NetworkProcess...")
        self.start_event.set()
        self.running = True

        logging.info("RSI Client Started")

        try:
            while self.running and not self.stop_event.is_set():
                time.sleep(2)
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logging.error(f"RSI Client encountered an error: {e}")

    def stop(self):
        """Stop the network process and the client thread safely."""
        logging.info("🛑 Stopping RSI Client...")

        self.running = False
        self.stop_event.set()  # ✅ Tell network process to exit nicely

        if self.network_process and self.network_process.is_alive():
            self.network_process.join(timeout=3)  # ✅ Give it time to shutdown
            if self.network_process.is_alive():
                logging.warning("⚠️ Forcing network process termination...")
                self.network_process.terminate()
                self.network_process.join()

        if hasattr(self, "thread") and self.thread and self.thread.is_alive():
            self.thread.join()
            self.thread = None

        logging.info("✅ RSI Client Stopped")

    def reconnect(self):
        """Reconnects the network process safely."""
        logging.info("Reconnecting RSI Client network...")

        if self.network_process and self.network_process.is_alive():
            self.stop_event.set()
            self.network_process.terminate()
            self.network_process.join()

        # Fresh new events
        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()

        # Create new network process
        network_settings = self.config_parser.get_network_settings()
        self.network_process = NetworkProcess(
            network_settings["ip"],
            network_settings["port"],
            self.send_variables,
            self.receive_variables,
            self.stop_event,
            self.config_parser,
            self.start_event
        )
        self.network_process.start()

        # Fresh control thread
        self.thread = threading.Thread(target=self.start, daemon=True)
        self.thread.start()
