import re
import xml.etree.ElementTree as ET
from typing import Dict, Any, List, Tuple, Optional


class XMLGenerator:
    """
    Converts structured dictionaries of RSI send/receive variables into
    valid XML strings for UDP transmission to/from the robot controller.
    """

    @staticmethod
    def generate_send_xml(send_variables, network_settings):
        """
        Build an outgoing XML message based on the current send variables.

        Args:
            send_variables (dict): Structured dictionary of values to send.
            network_settings (dict): Contains 'sentype' used for the root element.

        Returns:
            str: XML-formatted string ready for UDP transmission.
        """
        root = ET.Element("Sen", Type=network_settings["sentype"])

        for key, value in send_variables.items():
            if key == "FREE":
                continue

            if isinstance(value, dict):
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, f"{float(sub_value):.2f}")
            else:
                ET.SubElement(root, key).text = str(value)

        return ET.tostring(root, encoding="utf-8").decode()

    @staticmethod
    def generate_receive_xml(receive_variables):
        """
        Build an incoming XML message for emulation/testing purposes.

        Args:
            receive_variables (dict): Structured dictionary of values to simulate reception.

        Returns:
            str: XML-formatted string mimicking a KUKA robot's reply.
        """
        root = ET.Element("Rob", Type="KUKA")

        for key, value in receive_variables.items():
            if isinstance(value, dict) or hasattr(value, "items"):
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, f"{float(sub_value):.2f}")
            else:
                ET.SubElement(root, key).text = str(value)

        return ET.tostring(root, encoding="utf-8").decode()


class FastXMLGenerator:
    """
    Pre-compiled string template XML generator for the 4ms hot path.

    Compiles format strings at init time from the variable structure,
    then generates XML via string formatting (no DOM construction).
    ~5-10x faster than ElementTree for fixed-schema RSI messages.
    """

    def __init__(self, variables: dict, root_tag: str = "Sen", type_attr: str = "ImFree") -> None:
        """
        Compile a format template from the variable structure.

        Args:
            variables: Dict of variable names → values/dicts (defines the XML schema)
            root_tag: Root XML element name ('Sen' for outgoing, 'Rob' for incoming)
            type_attr: Value for the Type attribute on root element
        """
        self._keys: List[Tuple[str, Optional[List[str]]]] = []
        parts = [f'<{root_tag} Type="{type_attr}">']

        for key, value in variables.items():
            if key == "FREE":
                continue

            if isinstance(value, dict):
                subkeys = list(value.keys())
                self._keys.append((key, subkeys))
                # Use __ separator to avoid Python format_map treating . as attribute access
                attr_template = " ".join(f'{sk}="{{{key}__{sk}:.2f}}"' for sk in subkeys)
                parts.append(f"<{key} {attr_template} />")
            else:
                self._keys.append((key, None))
                parts.append(f"<{key}>{{{key}}}</{key}>")

        # Ensure IPOC is always in template (required by robot)
        if "IPOC" not in variables:
            self._keys.append(("IPOC", None))
            parts.append("<IPOC>{IPOC}</IPOC>")

        parts.append(f"</{root_tag}>")
        self._template = "".join(parts)

    def generate(self, variables: dict) -> str:
        """
        Generate XML string from current variable values using pre-compiled template.

        Args:
            variables: Current variable values dict

        Returns:
            XML string ready for UDP transmission
        """
        fmt_args = {}
        for key, subkeys in self._keys:
            if subkeys is not None:
                val = variables.get(key, {})
                if isinstance(val, dict):
                    for sk in subkeys:
                        fmt_args[f"{key}__{sk}"] = float(val.get(sk, 0.0))
                else:
                    for sk in subkeys:
                        fmt_args[f"{key}__{sk}"] = 0.0
            else:
                fmt_args[key] = variables.get(key, "")
        return self._template.format_map(fmt_args)


class FastXMLParser:
    """
    Pre-compiled regex XML parser for the 4ms hot path.

    Compiles regex patterns at init time from the known variable structure,
    then parses incoming XML via targeted regex extraction (no DOM construction).
    ~3-5x faster than ElementTree for fixed-schema RSI messages.
    """

    def __init__(self, variables: dict) -> None:
        """
        Compile regex patterns from the variable structure.

        Args:
            variables: Dict of expected variable names → values/dicts
        """
        self._dict_patterns: List[Tuple[str, re.Pattern, List[str]]] = []
        self._scalar_patterns: List[Tuple[str, re.Pattern]] = []

        for key, value in variables.items():
            if isinstance(value, dict):
                subkeys = list(value.keys())
                attr_pattern = r"\s+".join(
                    rf'{sk}="([^"]*)"' for sk in subkeys
                )
                # Also match any order with a looser pattern as fallback
                pattern = re.compile(rf"<{re.escape(key)}\s+{attr_pattern}")
                self._dict_patterns.append((key, pattern, subkeys))
            else:
                pattern = re.compile(rf"<{re.escape(key)}>([^<]*)</{re.escape(key)}>")
                self._scalar_patterns.append((key, pattern))

        # Always parse IPOC specifically
        self._ipoc_pattern = re.compile(r"<IPOC>(\d+)</IPOC>")

    def parse(self, xml_string: str, target: dict) -> None:
        """
        Parse XML string into target dict using pre-compiled patterns.

        Args:
            xml_string: Raw XML string from robot
            target: Dict to update with parsed values (modified in-place)
        """
        # Parse dict-type elements (attributes)
        for key, pattern, subkeys in self._dict_patterns:
            if key not in target:
                continue
            match = pattern.search(xml_string)
            if match:
                existing = target.get(key)
                if isinstance(existing, dict):
                    for i, sk in enumerate(subkeys):
                        existing[sk] = float(match.group(i + 1))
                else:
                    target[key] = {sk: float(match.group(i + 1)) for i, sk in enumerate(subkeys)}

        # Parse scalar elements (text content)
        for key, pattern in self._scalar_patterns:
            if key not in target:
                continue
            match = pattern.search(xml_string)
            if match:
                target[key] = match.group(1)

        # IPOC always parsed as int
        ipoc_match = self._ipoc_pattern.search(xml_string)
        if ipoc_match:
            target["IPOC"] = int(ipoc_match.group(1))
