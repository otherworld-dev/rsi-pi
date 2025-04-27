import logging
import xml.etree.ElementTree as ET

class ConfigParser:
    """
    Parses an RSI XML configuration file to extract structured variable definitions and
    network settings for both sending and receiving messages. Also integrates optional
    safety limits from an RSI limits XML file.
    """

    def __init__(self, config_file, rsi_limits_file=None):
        """
        Constructor method that loads the config file, parses variable definitions, and optionally
        loads safety limits.

        Args:
            config_file (str): Path to the RSI_EthernetConfig.xml file.
            rsi_limits_file (str, optional): Path to .rsi.xml file containing safety limits.
        """
        from src.RSIPI.rsi_limit_parser import parse_rsi_limits

        self.config_file = config_file
        self.rsi_limits_file = rsi_limits_file
        self.safety_limits = {}

        # Defines known internal variable structures used in RSI messaging
        self.internal_structure = {
            "ComStatus": "String",
        "RIst": {"X":0, "Y":0, "Z":0, "A":0, "B":0, "C":0},
        "RSol": {"X":0, "Y":0, "Z":0, "A":0, "B":0, "C":0},
        "ASPos": {"A1":0, "A2":0, "A3":0, "A4":0, "A5":0, "A6":0},
        "ELPos": {"E1":0, "E2":0, "E3":0, "E4":0, "E5":0, "E6":0},
        "ESPos": {"E1":0, "E2":0, "E3":0, "E4":0, "E5":0, "E6":0},
        "MaCur": {"A1":0, "A2":0, "A3":0, "A4":0, "A5":0, "A6":0},
        "MECur": {"E1":0, "E2":0, "E3":0, "E4":0, "E5":0, "E6":0},
        "IPOC": 000000,
        "BMode": "Status",
        "IPOSTAT": "",
        "Delay": ["D"],
        "EStr": "RSIPI: Client started",
        "Tech.C1": {"C11":0, "C12":0, "C13":0, "C14":0, "C15":0, "C16":0, "C17":0, "C18":0, "C19":0, "C110":0},
        "Tech.C2": {"C21":0, "C22":0, "C23":0, "C24":0, "C25":0, "C26":0, "C27":0, "C28":0, "C29":0, "C210":0},
        "Tech.C3": {"C31":0, "C32":0, "C33":0, "C34":0, "C35":0, "C36":0, "C37":0, "C38":0, "C39":0, "C310":0},
        "Tech.C4": {"C41":0, "C42":0, "C43":0, "C44":0, "C45":0, "C46":0, "C47":0, "C48":0, "C49":0, "C410":0},
        "Tech.C5": {"C51":0, "C52":0, "C53":0, "C54":0, "C55":0, "C56":0, "C57":0, "C58":0, "C59":0, "C510":0},
        "Tech.C6": {"C61":0, "C62":0, "C63":0, "C64":0, "C65":0, "C66":0, "C67":0, "C68":0, "C69":0, "C610":0},
        "Tech.T1": {"T11":0, "T12":0, "T13":0, "T14":0, "T15":0, "T16":0, "T17":0, "T18":0, "T19":0, "T110":0},
        "Tech.T2": {"T21":0, "T22":0, "T23":0, "T24":0, "T25":0, "T26":0, "T27":0, "T28":0, "T29":0, "T210":0},
        "Tech.T3": {"T31":0, "T32":0, "T33":0, "T34":0, "T35":0, "T36":0, "T37":0, "T38":0, "T39":0, "T310":0},
        "Tech.T4": {"T41":0, "T42":0, "T43":0, "T44":0, "T45":0, "T46":0, "T47":0, "T48":0, "T49":0, "T410":0},
        "Tech.T5": {"T51":0, "T52":0, "T53":0, "T54":0, "T55":0, "T56":0, "T57":0, "T58":0, "T59":0, "T510":0},
        "Tech.T6": {"T61":0, "T62":0, "T63":0, "T64":0, "T65":0, "T66":0, "T67":0, "T68":0, "T69":0, "T610":0},
        }

        self.network_settings = {}
        self.receive_variables, self.send_variables = self.process_config()

        # Flatten Tech.CX and Tech.TX keys into a single 'Tech' dictionary
        self.rename_tech_keys(self.send_variables)
        self.rename_tech_keys(self.receive_variables)

        # Ensure IPOC is always included in send variables
        if "IPOC" not in self.send_variables:
            self.send_variables["IPOC"] = 0

        # Optionally load safety limits from .rsi.xml file
        if self.rsi_limits_file:
            try:
                self.safety_limits = parse_rsi_limits(self.rsi_limits_file)
            except Exception as e:
                print(f"[WARNING] Failed to load .rsi.xml safety limits: {e}")
                self.safety_limits = {}

    def process_config(self):
        """
        Parses the RSI config file and builds the send/receive variable dictionaries.

        Returns:
            tuple: (send_vars, receive_vars) structured dictionaries.
        """
        send_vars = {}
        receive_vars = {}

        try:
            tree = ET.parse(self.config_file)
            root = tree.getroot()

            # Extract <CONFIG> section for IP/port/etc.
            config = root.find("CONFIG")
            if config is None:
                raise ValueError("Missing <CONFIG> section in RSI_EthernetConfig.xml")

            self.network_settings = {
                "ip": config.find("IP_NUMBER").text.strip() if config.find("IP_NUMBER") is not None else None,
                "port": int(config.find("PORT").text.strip()) if config.find("PORT") is not None else None,
                "sentype": config.find("SENTYPE").text.strip() if config.find("SENTYPE") is not None else None,
                "onlysend": config.find("ONLYSEND").text.strip().upper() == "TRUE" if config.find("ONLYSEND") is not None else False,
            }

            print(f"✅ Loaded network settings: {self.network_settings}")

            if None in self.network_settings.values():
                raise ValueError("Missing one or more required network settings (ip, port, sentype, onlysend)")

            # Parse SEND section
            send_section = root.find("SEND/ELEMENTS")
            if send_section is not None:
                for element in send_section.findall("ELEMENT"):
                    tag = element.get("TAG").replace("DEF_", "")
                    var_type = element.get("TYPE", "")
                    self.process_variable_structure(send_vars, tag, var_type)

            # Parse RECEIVE section
            receive_section = root.find("RECEIVE/ELEMENTS")
            if receive_section is not None:
                for element in receive_section.findall("ELEMENT"):
                    tag = element.get("TAG").replace("DEF_", "")
                    var_type = element.get("TYPE", "")
                    self.process_variable_structure(receive_vars, tag, var_type)

            return send_vars, receive_vars

        except Exception as e:
            logging.error(f"Error processing config file: {e}")
            return {}, {}

    def process_variable_structure(self, var_dict, tag, var_type, indx=""):
        """
        Processes and assigns a variable to the dictionary based on its tag and type.

        Args:
            var_dict (dict): Dictionary to add variable to.
            tag (str): Variable tag (can be nested like Tech.T1).
            var_type (str): Variable type (e.g. BOOL, DOUBLE, STRING).
            indx (str): Optional index (unused).
        """
        tag = tag.replace("DEF_", "")  # Remove DEF_ prefix if present

        if tag in self.internal_structure:
            # If pre-defined internally, copy structure
            internal_value = self.internal_structure[tag]
            var_dict[tag] = internal_value.copy() if isinstance(internal_value, dict) else internal_value
        elif "." in tag:
            # Handle nested dictionary e.g. Tech.T21 -> { 'Tech': { 'T21': 0.0 } }
            parent, subkey = tag.split(".", 1)
            if parent not in var_dict:
                var_dict[parent] = {}
            var_dict[parent][subkey] = self.get_default_value(var_type)
        else:
            # Standard single-value variable
            var_dict[tag] = self.get_default_value(var_type)

    @staticmethod
    def rename_tech_keys(var_dict):
        """
        Combines all Tech.XX keys into a single 'Tech' dictionary.

        Args:
            var_dict (dict): The variable dictionary to modify.
        """
        tech_data = {}
        for key in list(var_dict.keys()):
            if key.startswith("Tech."):
                tech_data.update(var_dict.pop(key))
        if tech_data:
            var_dict["Tech"] = tech_data

    @staticmethod
    def get_default_value(var_type):
        """
        Returns a default Python value based on RSI TYPE.

        Args:
            var_type (str): RSI type attribute.

        Returns:
            Default Python value.
        """
        if var_type == "BOOL":
            return False
        elif var_type == "STRING":
            return ""
        elif var_type == "LONG":
            return 0
        elif var_type == "DOUBLE":
            return 0.0
        return None

    def get_network_settings(self):
        """
        Returns extracted IP, port, and message mode settings.

        Returns:
            dict: Network settings extracted from the config file.
        """
        return self.network_settings
