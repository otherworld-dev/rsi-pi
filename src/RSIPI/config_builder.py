"""Generate a matching RSI_EthernetConfig XML from an RSI context.

Writing the Ethernet config by hand after building a context in RSIVisual is
fiddly and easy to get wrong, and getting it wrong is what produces

    RSI_CREATE: Invalid index - signal output

on the controller - the context references an ETHERNET channel the config
never declares. None of it needs to be typed out, though: the context already
records which object feeds every ETHERNET input and which object consumes
every ETHERNET output. This module reads that and writes the config.

    python -m RSIPI.config_builder MyContext.rsi.xml --ip 10.10.10.10 --port 64000

The generated config is correct by construction, and
examples/validate_context.py will confirm it.

Tag names follow the conventions RSIPI's API expects (RKorr, AKorr, DiO,
SenP1-3 and so on), because those are what motion.update_cartesian(),
io.set_output() and the rest look for. Rename them and the telegram still
works, but the named API methods stop finding their variables.
"""
import argparse
import logging
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .exceptions import RSIConfigError

# DataSize ordinals as they appear in a context: 0 = Bit, 1 = Byte, 2 = Word.
# Anything wider than a single bit arrives as an integer word, not a flag.
_BIT = "0"

# Objects that FEED the ETHERNET object (robot -> PC, the SEND section).
# type -> (tag template, RSI type). {n} is the object's instance number,
# {port} its output port name, {idx} the output index.
_SEND_MAP: Dict[str, Tuple[str, str]] = {
    "DIGIN": ("DiL", "LONG"),
    # DIGOUT's tag and type depend on its DataSize, refined below.
    "DIGOUT": ("Digout.o{n}", "BOOL"),
    "SOURCE": ("Source{n}", "DOUBLE"),
    # A correction object can feed ETHERNET too - its Stat output says
    # whether the controller is clamping, and which limit it hit.
    "POSCORR": ("PosCorr{port}", "LONG"),
    "AXISCORR": ("AxisCorr{port}", "LONG"),
    "POSCORRMON": ("PosCorrMon.{port}", "DOUBLE"),
    "AXISCORRMON": ("AxisCorrMon.{port}", "DOUBLE"),
    "MOTORCURRENT": ("MotorCurrent.A{idx}", "DOUBLE"),
    "ANIN": ("AnIn{n}", "DOUBLE"),
    "ANOUT": ("AnOutRead{n}", "DOUBLE"),
    "SEN_PINT": ("SenPInt{n}", "LONG"),
    "SEN_PREA": ("SenPRead{n}", "DOUBLE"),
    "OV_PRO": ("OvPro", "LONG"),
    # One STATUS object = one status value, so several are normal. Naming them
    # Status1, Status2... keeps them distinct; monitoring.get_robot_status()
    # takes the same index.
    "STATUS": ("Status{n}", "LONG"),
    "GEARTORQUE": ("GearTorque.A{idx}", "DOUBLE"),
    "TIMER": ("Timer{n}", "DOUBLE"),
}

# Objects that CONSUME ETHERNET outputs (PC -> robot, the RECEIVE section).
# type -> (list of tags indexed by the object's input index, RSI type, HOLDON)
_AXES6 = ["A1", "A2", "A3", "A4", "A5", "A6"]
_RECEIVE_MAP: Dict[str, Tuple[List[str], str, str]] = {
    "POSCORR": ([f"RKorr.{a}" for a in "XYZABC"], "DOUBLE", "1"),
    "AXISCORR": ([f"AKorr.{a}" for a in _AXES6], "DOUBLE", "1"),
    "AXISCORREXT": ([f"EKorr.E{i}" for i in range(1, 7)], "DOUBLE", "1"),
    "MAP2DIGOUT": (["DiO"], "LONG", "1"),
    "MAP2SEN_PREA": (["SenP{n}"], "DOUBLE", "1"),
    "MAP2SEN_PINT": (["SenPIntW{n}"], "LONG", "0"),
    "MAP2ANOUT": (["AnOut{n}"], "DOUBLE", "0"),
    "MAP2OV_PRO": (["OvProW"], "LONG", "0"),
    "STOP": (["MoveStop"], "BOOL", "0"),
}

# Tags the API looks for unnumbered when there is only one of that object.
# io.set_analog() defaults to AnOut1 but krl.write_sen_pint() to SenPIntW, so
# the generator has to match those exactly or the methods stop finding them.
_UNNUMBERED_WHEN_FIRST = {"SenPIntW1": "SenPIntW"}

# INTERNAL declarations. They cost no channel - RSI reads them straight from
# the system variables - so the sensible default is the set KUKA's own example
# uses, plus joint feedback when the context corrects joints.
_INTERNAL_SEND_BASE = ["DEF_RIst", "DEF_RSol", "DEF_Delay", "DEF_Tech.C1"]
_INTERNAL_SEND_JOINTS = ["DEF_AIPos", "DEF_ASPos"]
_INTERNAL_RECEIVE = [("DEF_EStr", "STRING", None), ("DEF_Tech.T2", "DOUBLE", "0")]


def _instance_number(obj_id: str) -> str:
    """Trailing digits of an ObjID: 'MAP2SEN_PREA3' -> '3'."""
    digits = ""
    for ch in reversed(obj_id):
        if ch.isdigit():
            digits = ch + digits
        else:
            break
    return digits or "1"


def _params(obj: ET.Element) -> Dict[str, str]:
    return {p.get("Name"): p.get("ParamValue")
            for p in obj.findall("Parameters/Parameter")}


def _out_port_names(rsi_path: Path, obj_id: str) -> List[str]:
    """Output port names of an object, read from the sibling .rsi if present.

    POSCORRMON's ports are X/Y/Z/A/B/C and AXISCORRMON's are A1-A6/E1-E6, and
    only the .rsi records those names. Without it we fall back to indices.
    """
    if not rsi_path.is_file():
        return []
    import re
    text = rsi_path.read_text(encoding="utf-8-sig", errors="replace")
    match = re.search(r'<rSIElement name="%s".*?</rSIElement>' % re.escape(obj_id),
                      text, re.S)
    if not match:
        return []
    return re.findall(r'<rSIOutPort name="(\w+)"', match.group(0))


def build_config(rsi_xml: str, ip: str, port: int, sentype: str = "ImFree",
                 onlysend: bool = False,
                 internals: Optional[List[str]] = None) -> str:
    """Build the Ethernet config XML text for a context.

    Args:
        rsi_xml: path to the context's ``.rsi.xml``
        ip: sensor (PC) IP the controller transmits to
        port: UDP port
        sentype: the name in ``<Sen Type="">``
        onlysend: True for one-way data logging (no replies expected)
        internals: extra INTERNAL SEND tags, e.g. ``["DEF_MACur"]``

    Returns:
        The config file's XML as text.

    Raises:
        RSIConfigError: if the context has no ETHERNET object, or wires a
            channel this builder has no naming convention for.
    """
    path = Path(rsi_xml)
    root = ET.parse(path).getroot()
    objs = {o.get("ObjID"): o for o in root.findall("RSIObject")}

    ethernet = next((o for o in root.findall("RSIObject")
                     if o.get("ObjType") == "ETHERNET"), None)
    if ethernet is None:
        raise RSIConfigError(
            f"{path.name} has no ETHERNET object, so it does not talk to a PC "
            "and needs no Ethernet config.")
    eth_id = ethernet.get("ObjID")
    rsi_file = path.with_suffix("").with_suffix(".rsi")  # foo.rsi.xml -> foo.rsi

    # ---------------------------------------------------------------- SEND
    send: List[Tuple[int, str, str]] = []
    unknown: List[str] = []
    for inp in ethernet.findall("Inputs/Input"):
        channel = int(inp.get("InIdx"))
        source = objs.get(inp.get("OutObjID"))
        if source is None:
            unknown.append(f"channel {channel}: unknown object "
                           f"{inp.get('OutObjID')!r}")
            continue
        otype = source.get("ObjType")
        if otype not in _SEND_MAP:
            unknown.append(f"SEND channel {channel}: no tag convention for "
                           f"{otype}")
            continue
        template, rtype = _SEND_MAP[otype]
        out_idx = int(inp.get("OutIdx", "1"))
        ports = _out_port_names(rsi_file, source.get("ObjID"))
        port_name = ports[out_idx - 1] if len(ports) >= out_idx else str(out_idx)
        tag = template.format(n=_instance_number(source.get("ObjID")),
                              port=port_name, idx=out_idx)
        # A DIGIN/DIGOUT wider than one bit is an integer word, not a flag.
        params = _params(source)
        if otype == "DIGOUT":
            if params.get("DataSize", _BIT) == _BIT:
                tag, rtype = f"Digout.o{_instance_number(source.get('ObjID'))}", "BOOL"
            else:
                tag, rtype = "DoutW", "LONG"
        elif otype == "DIGIN" and params.get("DataSize", _BIT) == _BIT:
            rtype = "BOOL"
        send.append((channel, tag, rtype))

    # ------------------------------------------------------------- RECEIVE
    # Corrections win a shared channel. KUKA's own example feeds MAP2SEN_PREA
    # from the same channels as POSCORR, so one channel can have two consumers;
    # naming it after the correction is what makes RKorr.X mean RKorr.X.
    _PRIORITY = {"POSCORR": 0, "AXISCORR": 0, "AXISCORREXT": 0}
    by_channel: Dict[int, Tuple[int, str, str, str]] = {}
    for obj in root.findall("RSIObject"):
        otype = obj.get("ObjType")
        wired = [(int(i.get("InIdx", "1")), int(i.get("OutIdx")))
                 for i in obj.findall("Inputs/Input")
                 if i.get("OutObjID") == eth_id]
        if not wired:
            continue
        if otype not in _RECEIVE_MAP:
            unknown += [f"RECEIVE channel {ch}: no tag convention for {otype}"
                        for _in, ch in wired]
            continue
        tags, rtype, holdon = _RECEIVE_MAP[otype]
        if otype == "MAP2DIGOUT":
            params = _params(obj)
            if params.get("DataSize", _BIT) == _BIT:
                # A bit-addressed MAP2DIGOUT drives ONE output, so name it after
                # that output: Dout.o5 for $OUT[5]. That is the per-bit group
                # notation io.set_output() auto-detects, and it keeps several
                # such objects from all being called "DiO".
                tags, rtype = [f"Dout.o{params.get('Index', '1')}"], "BOOL"
        n = _instance_number(obj.get("ObjID"))
        # Index tags by POSITION among this object's wired inputs, not by the
        # raw InIdx: AXISCORREXT's correction inputs start at 7, so InIdx-1
        # would run off the end of the tag list and silently reuse the first.
        for position, (_in_idx, channel) in enumerate(sorted(wired)):
            tag = tags[position] if position < len(tags) else tags[-1]
            tag = _UNNUMBERED_WHEN_FIRST.get(tag.format(n=n), tag.format(n=n))
            priority = _PRIORITY.get(otype, 1)
            existing = by_channel.get(channel)
            if existing is None or priority < existing[0]:
                by_channel[channel] = (priority, tag, rtype, holdon)
    receive = [(ch, tag, rtype, hold)
               for ch, (_p, tag, rtype, hold) in by_channel.items()]

    # Spare channels: a gap in the RECEIVE numbering is a declared-but-unused
    # channel (KUKA's example calls channel 7 "FREE"). Nothing consumes it, so
    # it cannot be derived from the objects - but leaving a hole shifts nothing
    # and declaring it keeps the numbering aligned with the context.
    if receive:
        used = {ch for ch, *_ in receive}
        for channel in range(1, max(used) + 1):
            if channel not in used:
                receive.append((channel, "FREE", "LONG", "1"))

    if unknown:
        raise RSIConfigError(
            "Cannot name every channel in this context:\n  " +
            "\n  ".join(unknown) +
            "\nAdd the object to _SEND_MAP/_RECEIVE_MAP in config_builder.py, "
            "or write the config by hand.")

    # The controller caps the OVERALL correction at 6 mm / 6 deg unless the
    # context carries a monitor object to raise it, and exceeding that stops
    # RSI with KSS29000. Found on the robot (2026-09-10): a context with
    # +/-50 mm POSCORR limits and no POSCORRMON died at ~5 mm on every move.
    types = {o.get("ObjType") for o in objs.values()}
    if "POSCORR" in types and "POSCORRMON" not in types:
        logging.warning(
            "%s has POSCORR but no POSCORRMON: the controller will stop RSI once "
            "the overall Cartesian correction exceeds its 6 mm / 6 deg default. "
            "Add a POSCORRMON (MaxTrans / MaxRotAngle) in RSIVisual.", Path(rsi_xml).name)
    if "AXISCORR" in types and "AXISCORRMON" not in types:
        logging.warning(
            "%s has AXISCORR but no AXISCORRMON: the overall axis correction is "
            "capped at 6 deg by default. Add an AXISCORRMON in RSIVisual.", Path(rsi_xml).name)

    # ------------------------------------------------------------- assemble
    internal_send = list(_INTERNAL_SEND_BASE)
    if any(o.get("ObjType") in ("AXISCORR", "AXISCORRMON") for o in objs.values()):
        internal_send += _INTERNAL_SEND_JOINTS
    for extra in internals or []:
        if extra not in internal_send:
            internal_send.append(extra)

    lines = [
        "<ROOT>",
        "   <CONFIG>",
        f"      <IP_NUMBER>{ip}</IP_NUMBER>   <!-- IP-number of the external socket -->",
        f"      <PORT>{port}</PORT>                   <!-- Port-number of the external socket -->",
        f"      <SENTYPE>{sentype}</SENTYPE>        <!-- The name of your system send in <Sen Type=\"\" > -->",
        f"      <ONLYSEND>{'TRUE' if onlysend else 'FALSE'}</ONLYSEND>       "
        "<!-- TRUE means the client don't expect answers. Do not send anything to robot -->",
        "   </CONFIG>",
        f"   <!-- Generated by RSIPI from {path.name}. Channel numbers come from",
        "        that context: the two files must stay together. -->",
        "   <SEND>",
        "      <ELEMENTS>",
    ]
    for tag in internal_send:
        rtype = "LONG" if tag == "DEF_Delay" else "DOUBLE"
        lines.append(f'         <ELEMENT TAG="{tag}" TYPE="{rtype}" INDX="INTERNAL" />')
    for channel, tag, rtype in sorted(send):
        lines.append(f'         <ELEMENT TAG="{tag}" TYPE="{rtype}" INDX="{channel}" />')
    lines += ["      </ELEMENTS>", "   </SEND>", "   <RECEIVE>", "      <ELEMENTS>"]
    for tag, rtype, holdon in _INTERNAL_RECEIVE:
        hold = f' HOLDON="{holdon}"' if holdon is not None else ""
        lines.append(f'         <ELEMENT TAG="{tag}" TYPE="{rtype}" INDX="INTERNAL"{hold} />')
    for channel, tag, rtype, holdon in sorted(receive):
        lines.append(f'         <ELEMENT TAG="{tag}" TYPE="{rtype}" INDX="{channel}" '
                     f'HOLDON="{holdon}" />')
    lines += ["      </ELEMENTS>", "   </RECEIVE>", "</ROOT>", ""]
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m RSIPI.config_builder",
        description="Generate the RSI_EthernetConfig XML that matches a context.")
    parser.add_argument("context", help="path to the context's .rsi.xml")
    parser.add_argument("--ip", required=True, help="sensor (PC) IP address")
    parser.add_argument("--port", type=int, required=True, help="UDP port")
    parser.add_argument("--sentype", default="ImFree", help="SENTYPE (default: ImFree)")
    parser.add_argument("--onlysend", action="store_true",
                        help="one-way data logging: the PC never replies")
    parser.add_argument("--internal", action="append", default=[],
                        metavar="DEF_TAG",
                        help="extra INTERNAL SEND tag, e.g. --internal DEF_MACur")
    parser.add_argument("--out", help="output path (default: print to stdout)")
    args = parser.parse_args(argv)

    try:
        xml = build_config(args.context, args.ip, args.port, args.sentype,
                           args.onlysend, args.internal)
    except (RSIConfigError, ET.ParseError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    if args.out:
        Path(args.out).write_text(xml, encoding="utf-8")
        print(f"wrote {args.out}")
        print("Check it against the context before deploying:")
        print(f"  python examples/validate_context.py {args.context}")
    else:
        print(xml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
