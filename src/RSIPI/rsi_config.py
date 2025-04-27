import xml.etree.ElementTree as ET
import logging
from src.RSIPI.rsi_limit_parser import parse_rsi_limits

# ✅ Configure Logging (toggleable)
LOGGING_ENABLED = False  # Change too False to silence logging output

if LOGGING_ENABLED:
    logging.basicConfig(
        filename="rsi_config.log",
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )


class RSIConfig:
    """
    Loads and parses the RSI EthernetConfig.xml file, extracting:
    - Network communication settings
    - Variables to send/receive (with correct structure)
    - Optional safety limit data from .rsi.xml file
    """

    # Known internal RSI variables and their structure
    internal = {
        "ComStatus": "String",
        "RIst": ["X", "Y", "Z", "A", "B", "C"],
        "RSol": ["X", "Y", "Z", "A", "B", "C"],
        "AIPos": ["A1", "A2", "A3", "A4", "A5", "A6"],
        "ASPos": ["A1", "A2", "A3", "A4", "A5", "A6"],
        "ELPos": ["E1", "E2", "E3", "E4", "E5", "E6"],
        "ESPos": ["E1", "E2", "E3", "E4", "E5", "E6"],
        "MaCur": ["A1", "A2", "A3", "A4", "A5", "A6"],
        "MECur": ["E1", "E2", "E3", "E4", "E5", "E6"],
        "IPOC": 0,
        "BMode": "Status",
        "IPOSTAT": "",
        "Delay": ["D"],
        "EStr": "EStr Test",
        "Tech.C1": ["C11", "C12", "C13", "C14", "C15", "C16", "C17", "C18", "C19", "C110"],
        "Tech.C2": ["C21", "C22", "C23", "C24", "C25", "C26", "C27", "C28", "C29", "C210"],
        "Tech.T2": ["T21", "T22", "T23", "T24", "T25", "T26", "T27", "T28", "T29", "T210"],
    }

    def __init__(self, config_file, rsi_limits_file=None):
        """
        Initialise config loader.

        Args:
            config_file (str): Path to the RSI EthernetConfig.xml file.
            rsi_limits_file (str): Optional path to .rsi.xml safety limits.
        """
        self.config_file = config_file
        self.rsi_limits_file = rsi_limits_file
        self.safety_limits = {}

        self.network_settings = {}
        self.send_variables = {}
        self.receive_variables = {}

        self.load_config()
        self.load_safety_limits()  # Optional safety overlay

    def load_safety_limits(self):
        """Loads safety bands from an optional .rsi.xml file, if provided."""
        if self.rsi_limits_file:
            try:
                self.safety_limits = parse_rsi_limits(self.rsi_limits_file)
                logging.info(f"Loaded safety limits from {self.rsi_limits_file}")
            except Exception as e:
                logging.warning(f"Failed to load RSI safety limits: {e}")
                self.safety_limits = {}

    @staticmethod
    def strip_def_prefix(tag):
        """Removes DEF_ prefix from variable names."""
        return tag.replace("DEF_", "")

    def process_internal_variable(self, tag):
        """Initialises structured internal variables based on known RSI types."""
        if tag in self.internal:
            if isinstance(self.internal[tag], list):
                return {key: 0.0 for key in self.internal[tag]}
            return self.internal[tag]
        return None

    def process_variable_structure(self, var_dict, tag, var_type):
        """
        Parses and groups structured variables, e.g., Tech.T2 → {'Tech': {'T2': 0.0}}.

        Args:
            var_dict (dict): Either send_variables or receive_variables.
            tag (str): The variable tag from XML.
            var_type (str): The TYPE attribute from XML.
        """
        if tag in self.internal:
            var_dict[tag] = self.process_internal_variable(tag)
        elif "." in tag:
            base, subkey = tag.split(".", 1)
            if base not in var_dict:
                var_dict[base] = {}
            var_dict[base][subkey] = self.get_default_value(var_type)
        else:
            var_dict[tag] = self.get_default_value(var_type)

    @staticmethod
    def get_default_value(var_type):
        """Returns a suitable default value for a given variable type."""
        if var_type == "BOOL":
            return False
        elif var_type == "STRING":
            return ""
        elif var_type == "LONG":
            return 0
        elif var_type == "DOUBLE":
            return 0.0
        return None  # Fallback for unknown types

    def load_config(self):
        """
        Parses the RSI config.xml, extracting:
        - IP/port and communication mode
        - Structured send and receive variable templates
        """
        try:
            logging.info(f"Loading config file: {self.config_file}")
            tree = ET.parse(self.config_file)
            root = tree.getroot()

            # Extract <CONFIG> network settings
            config = root.find("CONFIG")
            self.network_settings = {
                "ip": config.find("IP_NUMBER").text.strip(),
                "port": int(config.find("PORT").text.strip()),
                "sentype": config.find("SENTYPE").text.strip(),
                "onlysend": config.find("ONLYSEND").text.strip().upper() == "TRUE",
            }
            logging.info(f"Network settings loaded: {self.network_settings}")

            # Extract <SEND> section
            send_section = root.find("SEND/ELEMENTS")
            for element in send_section.findall("ELEMENT"):
                tag = self.strip_def_prefix(element.get("TAG"))
                var_type = element.get("TYPE")
                if tag != "FREE":  # Ignore placeholder entries
                    self.process_variable_structure(self.send_variables, tag, var_type)

            # Extract <RECEIVE> section
            receive_section = root.find("RECEIVE/ELEMENTS")
            for element in receive_section.findall("ELEMENT"):
                tag = self.strip_def_prefix(element.get("TAG"))
                var_type = element.get("TYPE")
                if tag != "FREE":
                    self.process_variable_structure(self.receive_variables, tag, var_type)

            logging.info("Configuration successfully loaded.")
            logging.debug(f"Send Variables: {self.send_variables}")
            logging.debug(f"Receive Variables: {self.receive_variables}")

        except Exception as e:
            logging.error(f"Error loading {self.config_file}: {e}")

    def get_network_settings(self):
        """Returns network configuration (IP, port, SENTYPE, ONLYSEND)."""
        return self.network_settings

    def get_send_variables(self):
        """Returns structured send variable dictionary."""
        return self.send_variables

    def get_receive_variables(self):
        """Returns structured receive variable dictionary."""
        return self.receive_variables
