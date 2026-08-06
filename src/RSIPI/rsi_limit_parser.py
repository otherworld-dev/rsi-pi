import logging
import xml.etree.ElementTree as ET
from typing import Optional


def _local_name(tag):
    """Strips any XML namespace from a tag name ('{ns}rSIParameter' -> 'rSIParameter')."""
    return tag.rsplit("}", 1)[-1] if isinstance(tag, str) else tag


def parse_ethernet_timeout(xml_path) -> Optional[int]:
    """
    Reads the ETHERNET object's Timeout parameter from an RSI signal-flow file.

    Timeout is the number of late/invalid sensor packets the controller tolerates
    before it breaks off RSI.

    Supports both shipped formats:
      - .rsi.xml : <RSIObject ObjType="ETHERNET"><Parameters>
                       <Parameter Name="Timeout" ParamValue="100" />
      - .rsi     : <rSIParameter name="Timeout" value="100" /> (namespaced RSIVisual model)

    Args:
        xml_path: Path to a .rsi.xml or .rsi file.

    Returns:
        int: The Timeout value, or None if absent or the file cannot be parsed.
    """
    try:
        tree = ET.parse(xml_path)
    except (ET.ParseError, OSError) as e:
        logging.warning(f"Failed to parse RSI file for ETHERNET Timeout: {xml_path} ({e})")
        return None

    root = tree.getroot()

    # .rsi.xml (RSIObject) format
    for rsi_object in root.iter():
        if _local_name(rsi_object.tag) != "RSIObject":
            continue
        if rsi_object.attrib.get("ObjType", "") != "ETHERNET":
            continue

        for params in rsi_object:
            if _local_name(params.tag) != "Parameters":
                continue
            for param in params:
                if _local_name(param.tag) != "Parameter":
                    continue
                if param.attrib.get("Name") == "Timeout":
                    try:
                        return int(param.attrib["ParamValue"])
                    except (KeyError, ValueError):
                        logging.warning(f"Unreadable ETHERNET Timeout value in {xml_path}")
                        return None

    # .rsi (RSIVisual model) fallback
    for element in root.iter():
        if _local_name(element.tag) != "rSIParameter":
            continue
        if element.attrib.get("name") == "Timeout":
            try:
                return int(element.attrib["value"])
            except (KeyError, ValueError):
                logging.warning(f"Unreadable ETHERNET Timeout value in {xml_path}")
                return None

    logging.warning(f"No ETHERNET Timeout parameter found in {xml_path}")
    return None


def parse_rsi_limits(xml_path):
    """
    Parses a .rsi.xml file (RSIObject format) and returns structured safety limits.

    Returns:
        dict: Structured limits in the form { "RKorr.X": (min, max), "AKorr.A1": (min, max), ... }
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    raw_limits = {}

    for rsi_object in root.findall("RSIObject"):
        obj_type = rsi_object.attrib.get("ObjType", "")
        params = rsi_object.find("Parameters")

        if params is None:
            continue  # Skip malformed entries

        if obj_type == "POSCORR":
            # Cartesian position correction limits
            for param in params.findall("Parameter"):
                name = param.attrib["Name"]
                value = float(param.attrib["ParamValue"])
                if name == "LowerLimX":
                    raw_limits["RKorr.X_min"] = value
                elif name == "UpperLimX":
                    raw_limits["RKorr.X_max"] = value
                elif name == "LowerLimY":
                    raw_limits["RKorr.Y_min"] = value
                elif name == "UpperLimY":
                    raw_limits["RKorr.Y_max"] = value
                elif name == "LowerLimZ":
                    raw_limits["RKorr.Z_min"] = value
                elif name == "UpperLimZ":
                    raw_limits["RKorr.Z_max"] = value
                elif name == "MaxRotAngle":
                    # Apply symmetric bounds to A/B/C
                    for axis in ["A", "B", "C"]:
                        raw_limits[f"RKorr.{axis}_min"] = -value
                        raw_limits[f"RKorr.{axis}_max"] = value

        elif obj_type == "AXISCORR":
            # Joint axis correction limits
            for param in params.findall("Parameter"):
                name = param.attrib["Name"]
                value = float(param.attrib["ParamValue"])
                if name.startswith("LowerLimA") or name.startswith("UpperLimA"):
                    axis = name[-1]
                    key = f"AKorr.A{axis}_{'min' if 'Lower' in name else 'max'}"
                    raw_limits[key] = value

        elif obj_type == "AXISCORREXT":
            # External axis correction limits -> EKorr, the variable the
            # write path (move_external_axis) actually validates against.
            # Accepts both LowerLimE1 and LowerLim1 parameter spellings.
            for param in params.findall("Parameter"):
                name = param.attrib["Name"]
                value = float(param.attrib["ParamValue"])
                if name.startswith(("LowerLim", "UpperLim")):
                    axis = name[-1]
                    if not axis.isdigit():
                        continue
                    key = f"EKorr.E{axis}_{'min' if 'Lower' in name else 'max'}"
                    raw_limits[key] = value

    # Combine _min and _max entries into structured tuples
    structured_limits = {}
    for key in list(raw_limits.keys()):
        if key.endswith("_min"):
            base = key[:-4]
            min_val = raw_limits.get(f"{base}_min")
            max_val = raw_limits.get(f"{base}_max")
            if min_val is not None and max_val is not None:
                structured_limits[base] = (min_val, max_val)

    return structured_limits
