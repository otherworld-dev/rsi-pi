import re
import xml.etree.ElementTree as ET
from typing import Dict, Any, List, Tuple, Optional


def format_value(value: Any) -> str:
    """Render one value for the telegram, keeping its declared type.

    ConfigParser gives each variable a default matching its config TYPE -
    BOOL becomes a Python bool, LONG an int, DOUBLE a float - and
    update_variable() coerces writes to that same type. So the Python type
    is the config's type, and formatting from it keeps the wire faithful to
    the declaration: a BOOL goes out as 1, not 1.000000.

    Grouped values (dotted tags, sent as XML attributes) used to be forced
    through float() regardless, which made every BOOL and LONG in a group a
    six-decimal float. Whether a controller accepts that for a Bool-typed
    signal is untested, and a silently mistyped signal is the hardest kind of
    fault to find on a robot.
    """
    if isinstance(value, bool):          # before int: bool IS an int
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6f}"
    if value is None:
        return "0"
    return str(value)


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
                    element.set(sub_key, format_value(sub_value))
            else:
                ET.SubElement(root, key).text = format_value(value)

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
                    element.set(sub_key, format_value(sub_value))
            else:
                ET.SubElement(root, key).text = format_value(value)

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
        self._subkinds: Dict[str, Dict[str, str]] = {}
        parts = [f'<{root_tag} Type="{type_attr}">']

        for key, value in variables.items():
            if key == "FREE":
                continue

            if isinstance(value, dict):
                subkeys = list(value.keys())
                self._keys.append((key, subkeys))
                # Pick each attribute's format from its declared type, once,
                # at compile time: ints and bools as integers, everything else
                # to six decimals. Formatting a BOOL as 1.000000 would be a
                # silently mistyped signal on the wire.
                self._subkinds[key] = {
                    sk: ("i" if isinstance(value[sk], (bool, int)) else "f")
                    for sk in subkeys
                }
                # Use __ separator to avoid Python format_map treating . as attribute access
                attr_template = " ".join(
                    f'{sk}="{{{key}__{sk}:{"d" if self._subkinds[key][sk] == "i" else ".6f"}}}"'
                    for sk in subkeys)
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
                kinds = self._subkinds.get(key, {})
                if not isinstance(val, dict):
                    val = {}
                for sk in subkeys:
                    raw = val.get(sk, 0)
                    # Match the coercion to the template's format spec: a "d"
                    # slot must receive an int or format_map raises.
                    fmt_args[f"{key}__{sk}"] = (
                        int(raw or 0) if kinds.get(sk) == "i" else float(raw or 0.0))
            else:
                val = variables.get(key, "")
                if isinstance(val, bool):
                    val = int(val)
                fmt_args[key] = val
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
