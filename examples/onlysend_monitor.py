"""ONLYSEND verification - the robot streams, the PC answers nothing.

ONLYSEND=TRUE in the RSI config puts the link in one-way data-logging mode:
the controller sends a <Rob> telegram every cycle and expects no <Sen> reply
at all. RSIPI skips its entire reply path in this mode - no serialise, no
sendto, no ack.

WHAT MAKES THIS A REAL TEST, not just "it printed some numbers":

  With ONLYSEND=FALSE, a PC that never replies kills the link in 0.4 s - the
  ETHERNET object breaks off with RSIBad after Timeout unanswered cycles.
  This script sends nothing for a full 30 s - 75x that break-off window. If
  the robot is still streaming at the end, ONLYSEND is genuinely active on
  the controller. If the config there were still FALSE, the robot would have
  stopped within the first second.

So the pass condition is the robot's survival, not the PC's output.

Usage:
    python examples/onlysend_monitor.py [config.xml]

Order does not matter here (there is no break-off to race), but starting
this first captures the telegrams from the very first cycle.
  1. PC:      run this script
  2. Pendant: T1, select RSIPI_OnlySend, press Start - it streams for 60 s

The KRL side must be RSIPI_OnlySend.src, which deliberately has no
RSI_MOVECORR(): corrections cannot be sent in this mode, so there would be
nothing to drive a sensor-guided motion. The robot stands still throughout.
"""
import sys
import time

from RSIPI import RSIAPI
from RSIPI.config_parser import ConfigParser
from RSIPI.exceptions import RSIStateError

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
CONFIG = (_args[0] if _args
          else "controller/SensorInterface/RSI_EthernetConfig_OnlySend.xml")
MONITOR_SECONDS = 30   # + ~7 s of logging, inside RSIPI_OnlySend.src's 60 s
LOG_SECONDS = 5


def preflight(config):
    """Refuse to run against a config that is not actually ONLYSEND.

    Without this the script would happily "pass" against a normal config
    while RSIPI quietly replied to every packet - proving nothing.
    """
    settings = ConfigParser(config).get_network_settings()
    print(f"Config      : {config}")
    print(f"Listening on: {settings['ip']}:{settings['port']}  "
          f"(SENTYPE={settings['sentype']})")
    print(f"ONLYSEND    : {settings.get('onlysend')}")
    if not settings.get("onlysend"):
        print("\nThis config has ONLYSEND=FALSE, so RSIPI would reply normally "
              "and the test would prove nothing.\nUse "
              "controller/SensorInterface/RSI_EthernetConfig_OnlySend.xml "
              "(and RSIPI_OnlySend.src on the pendant).")
        return False
    print("Replies     : NONE - the reply path is skipped entirely\n")
    return True


def test_streaming(api, seconds):
    """Robot data must keep arriving, with IPOC advancing ~1000/s."""
    print(f"== Test 1: one-way streaming for {seconds} s ==")
    print("The PC sends nothing at all. With ONLYSEND=FALSE the robot would")
    print("break off within 0.4 s, so surviving this is the actual result.\n")
    first_ipoc = last_ipoc = None
    pose_seen = False
    start = time.time()
    for i in range(seconds):
        time.sleep(1)
        send = api.client.send_variables
        ipoc = send.get("IPOC")
        pose = send.get("RIst") or {}
        if first_ipoc is None:
            first_ipoc = ipoc
        rate = "" if last_ipoc is None else f"  +{ipoc - last_ipoc} IPOC/s"
        last_ipoc = ipoc
        if any(abs(float(v or 0)) > 1e-9 for v in pose.values()):
            pose_seen = True
        if i % 5 == 0 or i == seconds - 1:
            xyz = " ".join(f"{k}={float(pose.get(k, 0)):.2f}" for k in ("X", "Y", "Z"))
            print(f"  [{i + 1:2d}s] {xyz}  IPOC={ipoc}{rate}")

    elapsed = time.time() - start
    advanced = (last_ipoc or 0) - (first_ipoc or 0)
    still_live = advanced > 0
    print(f"\n  IPOC advanced {advanced} over {elapsed:.1f} s "
          f"({advanced / elapsed:.0f}/s, expect ~1000/s)")
    print(f"  Robot still streaming after {elapsed:.0f} s of PC silence: "
          f"{'YES' if still_live else 'NO'}")
    print(f"  Real pose data (non-zero RIst): {'YES' if pose_seen else 'NO'}")
    return still_live and pose_seen


def test_write_guard(api):
    """Every correction path must refuse, rather than silently do nothing."""
    print("\n== Test 2: correction writes are refused ==")
    print("In this mode a correction can never reach the robot, so the API")
    print("must say so instead of accepting the value and dropping it.\n")
    results = []

    try:
        api.motion.update_cartesian(X=1.0)
        print("  update_cartesian(X=1.0)      ACCEPTED - should have raised!")
        results.append(False)
    except RSIStateError as e:
        print(f"  update_cartesian(X=1.0)      refused: {e}")
        results.append(True)

    try:
        api.client.publish_corrections({"RKorr": {"X": 1.0}})
        print("  publish_corrections(...)     ACCEPTED - should have raised!")
        results.append(False)
    except RSIStateError as e:
        print(f"  publish_corrections(...)     refused: {e}")
        results.append(True)

    return all(results)


def test_logging(api, seconds):
    """The point of ONLYSEND: capture the stream to CSV."""
    print(f"\n== Test 3: CSV capture ({seconds} s) ==")
    path = api.logging.start(f"logs/onlysend_{int(time.time())}.csv")
    time.sleep(seconds)
    api.logging.stop()
    time.sleep(2)  # logging process flushes and closes

    try:
        with open(path, encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except OSError as e:
        print(f"  could not read {path}: {e}")
        return False

    rows = max(len(lines) - 1, 0)
    print(f"  {path}: {rows} rows in {seconds} s ({rows / seconds:.0f}/s)")
    if rows:
        print(f"  header: {lines[0][:100]}")
        print(f"  last:   {lines[-1][:100]}")
    else:
        print("  no data rows captured")
    return rows > 0


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    if not preflight(CONFIG):
        sys.exit(2)

    api = RSIAPI(CONFIG)
    api.start()
    print("Waiting for the robot - START RSIPI_OnlySend ON THE PENDANT "
          "(up to 120 s)...")
    if not api.wait_for_connection(120):
        print("\nNO PACKETS. Checks:")
        print("  - did the KRL program reach RSI_ON?")
        print("  - is RSIPI_OnlySend.rsi on the controller, with its")
        print("    RSI_EthernetConfig_OnlySend.xml alongside it?")
        print("  - does the controller's config use this IP/port?")
        api.stop()
        sys.exit(1)

    print("\nCONNECTED - robot telegrams arriving.\n")
    results = {}
    try:
        results["streaming"] = test_streaming(api, MONITOR_SECONDS)
        results["write guard"] = test_write_guard(api)
        results["CSV capture"] = test_logging(api, LOG_SECONDS)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # No zero_corrections() here: there is nothing to zero, and every
        # write path correctly refuses in this mode.
        api.stop()

    print("\n" + "=" * 60)
    for name, ok in results.items():
        print(f"  {name:<12} {'PASS' if ok else 'FAIL'}")
    passed = results and all(results.values())
    print("=" * 60)
    if passed:
        print("ONLYSEND verified: the robot streamed for the full run while the")
        print("PC sent nothing, and every correction path refused cleanly.")
    else:
        print("ONLYSEND NOT verified - see the failures above.")
    print("\nThe pendant program ends by itself after 60 s.")
    sys.exit(0 if passed else 1)
