"""Client lifecycle without motion: state transitions, safety limits, restart.

Shows how ClientState changes across start()/stop() (the enum lives in
rsi_client.py), how safety limits stay empty until they are loaded from a
context's .rsi.xml - the file the KUKA controller actually reads, parsed by
RSIPI.rsi_limit_parser.parse_rsi_limits() (see safety_api.py and
example_08_safety_limits.py for the write-time enforcement side) - and what
actually works to bring the client back up after stop().

Restart uses api.reconnect(), NOT a second api.start() on the same instance.
RSIClient._VALID_TRANSITIONS only allows STOPPED -> INITIALIZED
(rsi_client.py ~line 42), so api.start() after api.stop() raises
RSIClientNotReady. RSIClient.reconnect() (rsi_client.py ~line 327) resets
state to INITIALIZED, rebuilds the Manager/dicts/events, spawns a fresh
NetworkProcess, and (restart=True, the default) starts a new control-loop
thread - all on the SAME RSIClient underneath this SAME RSIAPI object.
tests/test_lifecycle_reconnect.py exercises the same path (there, called
while RUNNING, to prove a latched E-stop survives it).

No motion commands here, so no confirm() gate is needed.

    python examples/example_18_rsi_lifecycle.py
    python examples/dry_run.py examples/example_18_rsi_lifecycle.py
"""
import sys

from RSIPI import RSIAPI, context, context_files
from RSIPI.rsi_limit_parser import parse_rsi_limits


def print_limits(label, limits):
    print(f"{label}: {len(limits)} entries")
    for key in ("RKorr.X", "AKorr.A1"):
        if key in limits:
            lower, upper = limits[key]
            print(f"  {key}: [{lower}, {upper}]")


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else context("joints")
    api = RSIAPI(CONFIG)
    print(f"State before start(): {api.state}")

    api.start()
    print(f"State after start(): {api.state}")

    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s - is RSI_ON running?")
        api.stop()
        sys.exit(1)
    # Unchanged from the line above: ClientState does not track the
    # connection itself, only connected_event does.
    print(f"State after first packet: {api.state}")

    print_limits("Safety limits as loaded", api.safety.get_limits())

    # RSIAPI has no post-construction "load limits from file" call - only its
    # constructor accepts rsi_limits_file, and that already ran without one.
    # context_files() returns the four files a context ships as
    # [.rsi, .rsi.xml, .rsi.diagram, RSI_EthernetConfig]; index 1 is the
    # .rsi.xml, the RSIObject-format file parse_rsi_limits() reads (POSCORR
    # for RKorr.*, AXISCORR for AKorr.*). Apply each bound through
    # set_limit() so it reaches both enforcement layers.
    rsi_xml = context_files("joints")[1]
    loaded = parse_rsi_limits(str(rsi_xml))
    for variable, (lower, upper) in loaded.items():
        api.safety.set_limit(variable, lower, upper)
    print_limits(f"Safety limits after loading {rsi_xml.name}", api.safety.get_limits())

    api.stop()
    print(f"State after stop(): {api.state}")

    # A second api.start() on this instance would attempt STOPPED -> STARTING,
    # which _VALID_TRANSITIONS forbids (only STOPPED -> INITIALIZED is legal),
    # so it raises RSIClientNotReady. reconnect() is what works: it resets
    # state to INITIALIZED, builds a fresh NetworkProcess, and restarts the
    # control loop in a new background thread, on this same RSIAPI/RSIClient.
    print("Restarting with api.reconnect() - a second start() would be refused")
    api.reconnect()

    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s after reconnect()")
        api.stop()
        sys.exit(1)
    print(f"State after reconnect() + first packet: {api.state}")

    api.stop()
    print(f"State after final stop(): {api.state}")
