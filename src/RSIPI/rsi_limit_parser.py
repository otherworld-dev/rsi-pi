import xml.etree.ElementTree as ET

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
            # External axis correction limits
            for param in params.findall("Parameter"):
                name = param.attrib["Name"]
                value = float(param.attrib["ParamValue"])
                if name.startswith("LowerLimE") or name.startswith("UpperLimE"):
                    axis = name[-1]
                    key = f"AKorr.E{axis}_{'min' if 'Lower' in name else 'max'}"
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
