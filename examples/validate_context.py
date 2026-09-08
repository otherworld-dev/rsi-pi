"""Check that an RSI context and its Ethernet config agree.

Catches the mismatches that only show up on the controller as cryptic
errors - most importantly:

    RSI_CREATE: Invalid index - signal output

which means an object references an ETHERNET output index that the config's
RECEIVE section never declares (typically a .rsi and a config file copied
from different builds).

Usage:
    python examples/validate_context.py controller/SensorInterface/RSIPI_Basic.rsi.xml
    python examples/validate_context.py            # checks every context found
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SENSOR_DIR = Path(__file__).resolve().parent.parent / "controller" / "SensorInterface"


def channels(config_path):
    """(send_indices, receive_indices) declared with a numeric INDX."""
    root = ET.parse(config_path).getroot()
    out = []
    for section in ("SEND", "RECEIVE"):
        idx = set()
        elements = root.find(f"{section}/ELEMENTS")
        for el in (elements if elements is not None else []):
            raw = el.get("INDX", "")
            if raw.isdigit():
                idx.add(int(raw))
        out.append(idx)
    return out


def check(rsi_xml_path):
    rsi_xml = Path(rsi_xml_path)
    root = ET.parse(rsi_xml).getroot()
    problems = []

    ethernet = next((o for o in root.findall("RSIObject")
                     if o.get("ObjType") == "ETHERNET"), None)
    if ethernet is None:
        return [f"{rsi_xml.name}: no ETHERNET object"]

    cfg_name = next((p.get("ParamValue") for p in ethernet.findall("Parameters/Parameter")
                     if p.get("Name") == "ConfigFile"), None)
    cfg_path = rsi_xml.parent / cfg_name if cfg_name else None
    print(f"\n{rsi_xml.name}  ->  ConfigFile = {cfg_name}")

    if cfg_path is None or not cfg_path.exists():
        return [f"{rsi_xml.name}: ConfigFile '{cfg_name}' not found beside the context"]

    send_idx, recv_idx = channels(cfg_path)
    eth_id = ethernet.get("ObjID")

    # Objects consume ETHERNET outputs; each must be a declared RECEIVE channel.
    used_out = {}
    for obj in root.findall("RSIObject"):
        for inp in obj.findall("Inputs/Input"):
            if inp.get("OutObjID") == eth_id:
                used_out.setdefault(int(inp.get("OutIdx")), []).append(obj.get("ObjID"))

    # ETHERNET consumes other objects' outputs; each is a SEND channel.
    used_in = {int(i.get("InIdx")) for i in ethernet.findall("Inputs/Input")}

    for idx in sorted(used_out):
        if idx not in recv_idx:
            problems.append(
                f"  {', '.join(used_out[idx])} reads {eth_id} output {idx}, but "
                f"{cfg_name} declares no RECEIVE channel {idx} "
                f"(declared: {sorted(recv_idx) or 'none'})")
    for idx in sorted(used_in):
        if idx not in send_idx:
            problems.append(
                f"  {eth_id} input {idx} is wired, but {cfg_name} declares no "
                f"SEND channel {idx} (declared: {sorted(send_idx) or 'none'})")

    unused = sorted(recv_idx - set(used_out))
    if unused:
        print(f"  note: RECEIVE channels {unused} are declared but unused by any "
              f"object (fine for spares like FREE)")

    print(f"  objects: {[o.get('ObjType') for o in root.findall('RSIObject')]}")
    print(f"  SEND channels {sorted(send_idx)} | RECEIVE channels {sorted(recv_idx)}")
    return problems


if __name__ == "__main__":
    targets = ([Path(a) for a in sys.argv[1:]] or
               sorted(SENSOR_DIR.glob("*.rsi.xml")))
    if not targets:
        print(f"No .rsi.xml files found in {SENSOR_DIR}")
        sys.exit(2)

    all_problems = []
    for t in targets:
        all_problems += check(t)

    print()
    if all_problems:
        print("MISMATCHES FOUND:")
        for p in all_problems:
            print(p)
        print("\nCopy the .rsi, .rsi.xml, .rsi.diagram AND the config together -")
        print("they are one unit; a stale pair is what produces")
        print("'RSI_CREATE: Invalid index - signal output' on the controller.")
        sys.exit(1)
    print("OK - every context matches its config.")
