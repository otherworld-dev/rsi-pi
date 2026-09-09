"""From an RSIVisual context to files ready for the controller - no robot,
no network.

Walks the full "context to deploy" path using the shipped Max context as
the worked example:

  1. List the contexts RSIPI ships (available_contexts/describe_contexts).
  2. Locate Max's .rsi.xml (context_files), regenerate its Ethernet config
     with config_builder.build_config() using the same ip/port/sentype the
     shipped RSI_EthernetConfig_Max.xml itself declares, and check the
     result against that shipped file channel-for-channel - the same
     comparison tests/test_config_builder.py makes (SEND/RECEIVE INDX ->
     (TAG, TYPE) for every numbered channel), not a byte diff: comments,
     attribute order and INTERNAL declarations are allowed to differ.
  3. Copy Max's four files with deploy.deploy() and print where they go on
     a KRC4, from docs/controller-setup.md.

    python examples/example_17_context_to_deploy.py
    python examples/dry_run.py examples/example_17_context_to_deploy.py
"""
import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from RSIPI import available_contexts, describe_contexts, context_files
from RSIPI.config_builder import build_config
from RSIPI.deploy import deploy


def channels(xml_source, is_text=False):
    """(section, index) -> (tag, type) for every numbered channel."""
    root = ET.fromstring(xml_source) if is_text else ET.parse(xml_source).getroot()
    out = {}
    for section in ("SEND", "RECEIVE"):
        for element in root.find(f"{section}/ELEMENTS"):
            index = element.get("INDX")
            if index.isdigit():
                out[(section, int(index))] = (element.get("TAG"), element.get("TYPE"))
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    # dry_run.py passes a config path positionally for every script it runs;
    # this one needs no config, so accept and ignore it.
    parser.add_argument("config", nargs="?", help=argparse.SUPPRESS)
    parser.add_argument("--out", default=None,
                        help="write to this directory instead of a temp dir")
    args = parser.parse_args()

    print(f"Available contexts: {available_contexts()}")
    print(describe_contexts())

    print("\nMax context files:")
    files = context_files("max")
    for f in files:
        print(f"  {f}")
    rsi_xml = next(f for f in files if f.name.endswith(".rsi.xml"))
    shipped_config = next(f for f in files if f.name.startswith("RSI_EthernetConfig_"))

    # Read the settings back from the shipped config itself rather than
    # hardcoding them, so this keeps working if the shipped IP/port changes.
    cfg = ET.parse(shipped_config).getroot().find("CONFIG")
    ip = cfg.find("IP_NUMBER").text.strip()
    port = int(cfg.find("PORT").text.strip())
    sentype = cfg.find("SENTYPE").text.strip()
    onlysend = cfg.find("ONLYSEND").text.strip().upper() == "TRUE"
    print(f"\nShipped config uses ip={ip} port={port} sentype={sentype} "
          f"onlysend={onlysend}")

    out_root = Path(args.out) if args.out else Path(tempfile.mkdtemp(prefix="rsipi_deploy_"))
    out_root.mkdir(parents=True, exist_ok=True)

    # Max is the one shipped context needing an INTERNAL tag beyond the base
    # set - DEF_MACur for motor currents - which build_config() cannot infer
    # from the .rsi.xml alone (see tests/test_config_builder.py, which passes
    # the same internals=["DEF_MACur"] for this context).
    generated_xml = build_config(str(rsi_xml), ip, port, sentype=sentype,
                                 onlysend=onlysend, internals=["DEF_MACur"])
    generated_path = out_root / "RSI_EthernetConfig_Max.generated.xml"
    generated_path.write_text(generated_xml, encoding="utf-8")
    print(f"\nGenerated config written to {generated_path}")

    matches = channels(generated_xml, is_text=True) == channels(shipped_config)
    print(f"Matches shipped {shipped_config.name} channel-for-channel: {matches}")

    deploy_dir = out_root / "deploy"
    written = deploy("max", out_dir=str(deploy_dir))
    print(f"\nDeployed context 'max' to {deploy_dir}:")
    for path in written:
        print(f"  {path}")

    print("\nPut these on the KRC4 controller (docs/controller-setup.md):")
    print("  context files (.rsi, .rsi.xml, .rsi.diagram, RSI_EthernetConfig_*.xml)")
    print(r"    -> C:\KRC\ROBOTER\Config\User\Common\SensorInterface")
    print("  KRL programs (controller/Program/*.src)")
    print(r"    -> C:\KRC\ROBOTER\KRC\R1\Program")
    print("  Copy all four context files together, as the Expert user group -")
    print("  a partial set produces RSI_CREATE: Invalid index - signal output.")
