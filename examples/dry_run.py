"""Run a hardware script against the emulated controller before the lab.

The bring-up scripts are only ever exercised in front of a robot, which is
the worst place to discover a typo in a branch that only runs when someone
answers "yes". This starts the echo server, points the script at it, and
answers every prompt affirmatively.

    python examples/dry_run.py examples/max_test.py --context max
    python examples/dry_run.py examples/first_contact.py --context joints

WHAT THIS PROVES: the script runs end to end, every branch is reachable, and
no call raises.

WHAT IT DOES NOT PROVE: any result. The emulator applies no POSCORR limits, so
clamping is never reported; it has no RSI_MOVECORR, so a STOP can never end
one; and it binds no objects, so it cannot tell you whether a context will
load on the controller. Failures for those reasons are expected here and are
not bugs - read them as "the script asked the question correctly".
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from RSIPI import context, available_contexts
from RSIPI.rsi_echo_server import EchoServer


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python examples/dry_run.py",
        description="Run a hardware script against the emulated controller.")
    parser.add_argument("script", help="the script to run, e.g. examples/max_test.py")
    parser.add_argument("--context", default="max", choices=sorted(available_contexts()),
                        help="which context to emulate (default: max)")
    parser.add_argument("--port", type=int, default=59490,
                        help="loopback UDP port (default: 59490)")
    parser.add_argument("--answers", type=int, default=8,
                        help="how many prompts to answer yes (default: 8)")
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args(argv)

    # Point a copy of the context's config at loopback so nothing touches a
    # real network, and so a robot on the bench cannot be driven by accident.
    tree = ET.parse(context(args.context))
    root = tree.getroot()
    root.find("CONFIG/IP_NUMBER").text = "127.0.0.1"
    root.find("CONFIG/PORT").text = str(args.port)
    config = os.path.join(tempfile.mkdtemp(), "dry_run_config.xml")
    tree.write(config)

    print(f"Emulating context {args.context!r} on 127.0.0.1:{args.port}")
    try:
        server = EchoServer(config, mode="relative")
    except OSError as e:
        # The echo server binds a fixed port (50000). A previous run that was
        # killed rather than stopped still holds it, and the raw OSError says
        # nothing useful.
        print(f"Could not start the emulated controller: {e}")
        print()
        print("The echo server binds UDP 50000. Something already has it -")
        print("usually an earlier dry run that did not shut down. Find it with:")
        print("  Get-NetUDPEndpoint -LocalPort 50000 | "
              "Select-Object OwningProcess")
        return 2
    # Plausible non-zero readings, so a check that reads a sensor sees data.
    if "AnIn1" in server.state:
        server.state["AnIn1"] = 3.25
    if isinstance(server.state.get("MACur"), dict):
        server.state["MACur"]["A1"] = 12.5
    server.start()
    time.sleep(0.5)

    try:
        # _confirm.confirm() auto-answers on this, so prompt counts do not
        # have to match. Never set it with a robot attached.
        proc = subprocess.run(
            [sys.executable, args.script, config],
            input="yes\n" * args.answers,     # for scripts with their own prompt
            env=dict(os.environ, RSIPI_ASSUME_YES="1"),  # for _confirm
            capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        print(f"TIMED OUT after {args.timeout}s - the script hung.")
        return 2
    finally:
        server.stop()

    print(proc.stdout)
    if proc.stderr.strip():
        print("--- stderr (tail) ---")
        print(proc.stderr[-1500:])

    if "Traceback" in proc.stderr:
        print("\nThe script RAISED. Fix that before taking it to the robot.")
        return 1
    print("\nScript ran to completion. Remember: PASS/FAIL above reflects the")
    print("emulator, not the robot - see this file's docstring.")
    return 0


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    raise SystemExit(main())
