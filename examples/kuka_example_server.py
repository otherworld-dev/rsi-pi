"""RSIPI as a drop-in replacement for KUKA's TestServer.exe.

Works with KUKA's own example program `RSI_Ethernet.src` and its context
`RSI_Ethernet.rsi`, using the matching `RSI_EthernetConfig.xml` (this repo's
root copy is KUKA's file with PORT set to 64000).

It answers every robot packet with a valid <Sen> reply (IPOC echoed
unchanged, corrections zero), and prints what the robot sends - the same
fields TestServer shows - so you can compare the two directly.

ORDER MATTERS: start this FIRST, then start the KRL program on the pendant.
KUKA's example has no HALT before RSI_ON, and the ETHERNET object breaks off
(RSIBad) after Timeout unanswered cycles.

Usage:
    python examples/kuka_example_server.py [config.xml] [--jog]

    --jog     after 10 s of clean monitoring, offer a small +2 mm X move
              (paced one delta per robot cycle, rate-limited to 0.1 mm/cycle)
    --joints  also offer a small A6 joint correction - only works on a
              context that wires AXISCORR (RSIPI_Basic/RSIPI_Full, not
              KUKA's RSI_Ethernet example)
"""
import sys
import time

from RSIPI import RSIAPI

CONFIG = "RSI_EthernetConfig.xml"
RATE_LIMIT = 0.1      # mm per robot cycle (~25 mm/s at 4 ms)
JOG_MM = 2.0


def fmt(d):
    if isinstance(d, dict):
        return " ".join(f"{k}={float(v):.1f}" for k, v in d.items())
    return str(d)


def monitor(api, seconds):
    """Print the robot's state once a second, like TestServer's packet view."""
    last_ipoc = None
    for _ in range(seconds):
        time.sleep(1)
        send = api.client.send_variables
        ipoc = send.get("IPOC")
        rate = "" if last_ipoc is None else f"  (+{ipoc - last_ipoc} IPOC/s)"
        last_ipoc = ipoc
        print(f"  RIst  {fmt(send.get('RIst'))}")
        print(f"  Delay {fmt(send.get('Delay'))}   DiL={send.get('DiL')}   "
              f"IPOC={ipoc}{rate}")


def jog(api):
    pose = api.motion.get_current_pose()
    print(f"\nCurrent X = {pose['X']:.2f}")
    if input(f"Move +{JOG_MM} mm in X at ~25 mm/s? [yes/NO]: ").strip().lower() != "yes":
        print("Skipped.")
        return
    api.motion.move_cartesian_trajectory({"X": pose["X"] + JOG_MM}, steps=20)
    time.sleep(0.5)
    moved = api.motion.get_current_pose()["X"] - pose["X"]
    print(f"  moved {moved:+.2f} mm (commanded {JOG_MM:+.1f})")
    if input("Move back? [yes/NO]: ").strip().lower() == "yes":
        api.motion.move_cartesian_trajectory({"X": pose["X"]}, steps=20)
        time.sleep(0.5)
        print(f"  back at X = {api.motion.get_current_pose()['X']:.2f}")


def io_readback(api):
    """Write the DiO word and read the robot's own view of it back.

    Answers "did the robot actually set the outputs?" without the pendant.
    Needs a context with the DoutW read-back channel (RSIPI_Joints).
    """
    if "DoutW" not in api.client.send_variables:
        print("\nNo DoutW read-back channel in this config - skipping I/O check.")
        return
    print("\nI/O read-back (DiO word -> robot outputs -> DoutW):")
    ok = True
    for word in (3, 5, 0):
        api.tools.update_variable("DiO", word)
        time.sleep(1.0)
        got = api.client.send_variables.get("DoutW")
        try:
            match = int(float(got)) == word
        except (TypeError, ValueError):
            match = False
        ok &= match
        print(f"  wrote DiO={word:<3} robot reports DoutW={got!r}  "
              f"{'MATCH' if match else 'MISMATCH'}")
    api.tools.update_variable("DiO", 0)
    print("  => digital outputs " + ("WORK" if ok else "are NOT being set"))


def jog_joint(api, axis="A6", degrees=2.0):
    """Diagnose the joint path: feedback first, then whether AKorr applies."""
    if "AKorr" not in api.client.receive_variables:
        print("\nAKorr is not in this config's RECEIVE section - joint "
              "corrections need RSIPI_Joints, skipping.")
        return

    send = api.client.send_variables
    aipos = dict(send.get("AIPos") or {})
    aspos = dict(send.get("ASPos") or {})
    print("\nJoint feedback from the robot:")
    print(f"  AIPos (actual)   {aipos}")
    print(f"  ASPos (setpoint) {aspos}")
    live = {k: v for d in (aspos, aipos) for k, v in d.items() if abs(float(v or 0)) > 1e-9}
    if not live:
        print("  => both are all-zero: the robot is not populating joint feedback,")
        print("     so any measurement below is blind. Corrections may still apply.")

    # Raw correction hold: publish AKorr directly and watch the axis. This
    # bypasses trajectory logic entirely, so it answers only one question -
    # does an axis correction reach the robot at all?
    if input(f"\nHold a small {axis} correction for 2 s (raw AKorr)? [yes/NO]: "
             ).strip().lower() != "yes":
        print("Skipped.")
        return
    before_i, before_s = aipos.get(axis, 0.0), aspos.get(axis, 0.0)
    api.motion.update_joints(**{axis: 0.02})       # deg per cycle, relative mode
    for _ in range(4):
        time.sleep(0.5)
        s = api.client.send_variables
        print(f"    AIPos.{axis}={dict(s.get('AIPos') or {}).get(axis)}  "
              f"ASPos.{axis}={dict(s.get('ASPos') or {}).get(axis)}")
    api.motion.update_joints(**{axis: 0.0})
    time.sleep(0.5)
    s = api.client.send_variables
    d_i = float(dict(s.get("AIPos") or {}).get(axis, 0.0)) - float(before_i)
    d_s = float(dict(s.get("ASPos") or {}).get(axis, 0.0)) - float(before_s)
    print(f"  net change: AIPos {d_i:+.3f} deg, ASPos {d_s:+.3f} deg")
    if abs(d_i) > 0.05 or abs(d_s) > 0.05:
        print("  => AXISCORR works: joint corrections reach the robot.")
    else:
        print("  => no joint movement detected. Either the robot ignores axis")
        print("     corrections while POSCORR is also active, or joint feedback")
        print("     is not being reported (check the pendant's axis display).")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    config = args[0] if args else CONFIG
    want_jog = "--jog" in sys.argv
    want_joints = "--joints" in sys.argv

    api = RSIAPI(config, max_cartesian_rate=RATE_LIMIT)
    net = api.client.config_parser.get_network_settings()
    print(f"Config      : {config}")
    print(f"Listening on: {net['ip']}:{net['port']}  (SENTYPE={net['sentype']})")
    print("Replies     : IPOC echoed unchanged, corrections zero\n")

    api.start()
    print("Waiting for the robot - START THE KRL PROGRAM NOW (up to 120 s)...")
    if not api.wait_for_connection(120):
        print("\nNO PACKETS. Checks:")
        print("  - is another program holding the port (KUKA TestServer)?")
        print("  - did the KRL program reach RSI_ON?")
        print("  - does the controller's config use this IP/port?")
        api.stop()
        sys.exit(1)

    print("\nCONNECTED - robot packets arriving:\n")
    try:
        monitor(api, 10)
        delay = api.client.send_variables.get("Delay")
        d = delay.get("D") if isinstance(delay, dict) else delay
        print(f"\nLate-packet counter (DEF_Delay): {d}")
        io_readback(api)
        if want_jog:
            jog(api)
        if want_joints:
            jog_joint(api)
        if not (want_jog or want_joints):
            print("Monitoring only. Re-run with --jog to test a small move.")
            print("Ctrl+C to stop.")
            while True:
                monitor(api, 5)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # An ONLYSEND config refuses every correction write, including this
        # one - that mode has examples/onlysend_monitor.py instead.
        try:
            api.motion.update_cartesian(X=0.0, Y=0.0, Z=0.0)
        except Exception as e:
            print(f"cleanup warning: {type(e).__name__}: {e}")
        api.stop()
        print("Stopped. Cancel the KRL program on the pendant.")
