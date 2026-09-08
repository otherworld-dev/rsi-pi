"""Test whether RSI_MOVECORR() can be ended from the PC (STOP object).

Pair with controller/Program/RSIPI_Stop.src and the RSIPI_Stop context.

WHY THIS SCRIPT MEASURES MOTION FIRST
-------------------------------------
The previous STOP object did not fail loudly - it made the robot silently
stop applying *every* correction while RSI carried on running perfectly:
263 packets/s, Delay 0, no error, no log entry, and a pose that never
changed. It was caught only by noticing the reported position was
byte-identical between two runs.

So test 1 commands a 5 mm move and measures it. A STOP object that breaks
corrections fails here, loudly, before anything else is attempted. Only if
motion survives does test 2 try to end MOVECORR.

The fix under test: KUKA's own STOP objects (RSI Examples/CircleCorr and
DistanceCtrl) carry exactly ONE parameter, Mode=ExitMoveCorr. Ours had
invented a second, `Channel`, which does not exist on the object. Mode=4
itself was correct. RSIPI_Stop.rsi now matches KUKA's shape exactly.

Usage:
    python examples/stop_test.py [config.xml]

Order matters: start this FIRST, then press Start past the HALT on the
pendant. Hand on the enabling switch and the hardware E-stop throughout.
"""
import sys
import time

from RSIPI import RSIAPI, context

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
CONFIG = (_args[0] if _args
          else context("stop"))
RATE_LIMIT = 0.1     # mm per robot cycle (~25 mm/s at 4 ms)
MOVE_MM = 5.0
LIMIT = 6.0


def ask(prompt):
    return input(f"\n{prompt} [yes/NO]: ").strip().lower() == "yes"


def test_corrections_still_work(api):
    """The regression check: does the STOP object silently kill motion?"""
    print("\n== Test 1: are corrections still applied? ==")
    print("This is the failure mode that cost the most last time: RSI runs")
    print("perfectly while nothing moves. Measuring, not assuming.\n")

    start = api.motion.get_current_pose()
    print(f"  start X={start['X']:.2f}")
    if not ask(f"Move +{MOVE_MM:.0f} mm in X at ~25 mm/s? Robot WILL move."):
        print("Skipped - cannot judge the STOP object without this.")
        return None

    api.motion.move_cartesian_trajectory({"X": start["X"] + MOVE_MM}, steps=50)
    time.sleep(0.5)
    moved = api.motion.get_current_pose()["X"] - start["X"]
    print(f"  moved {moved:+.2f} mm (commanded {MOVE_MM:+.1f})")

    ok = abs(moved) > 0.3
    if ok:
        print("  => corrections WORK with the STOP object present.")
    else:
        print("  => THE ROBOT DID NOT MOVE. The STOP object has disabled")
        print("     corrections again, exactly as before. Do not proceed -")
        print("     remove STOP1 from RSIPI_Stop.rsi/.rsi.xml and re-test.")

    # Put it back regardless, so the robot ends where it started.
    api.motion.move_cartesian_trajectory({"X": start["X"]}, steps=50)
    time.sleep(0.5)
    print(f"  returned to X={api.motion.get_current_pose()['X']:.2f}")
    return ok


def test_exit_movecorr(api):
    """Does MoveStop actually end RSI_MOVECORR on the controller?

    Detection: when MOVECORR ends, the KRL program runs on to RSI_OFF, so
    the robot stops transmitting. A frozen IPOC is the signal.
    """
    print("\n== Test 2: end RSI_MOVECORR from the PC ==")
    if "MoveStop" not in api.client.receive_variables:
        print("  MoveStop is not declared in this config - wrong context?")
        return False
    if not ask("Send MoveStop to end RSI_MOVECORR?"):
        print("Skipped.")
        return False

    before = api.client.send_variables.get("IPOC")
    api.motion.exit_movecorr()            # sets MoveStop, holds, clears
    print("  MoveStop pulsed; watching whether the robot stops sending...")

    for i in range(6):
        time.sleep(0.5)
        now = api.client.send_variables.get("IPOC")
        if now == before:
            print(f"  IPOC frozen at {now} after {(i + 1) * 0.5:.1f}s")
            print("  => MOVECORR ENDED: the program ran on to RSI_OFF.")
            print("     Confirm on the pendant: 'MOVECORR ENDED - STOP object works'")
            return True
        before = now

    print(f"  IPOC still advancing ({before}) - MOVECORR did not end.")
    print("  => The STOP object loaded but did not trigger. Next suspects:")
    print("     the input wiring (KUKA drives STOP from a condition object,")
    print("     not an ETHERNET channel), or the BOOL encoding on the wire.")
    return False


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    print(f"Config: {CONFIG}")
    api = RSIAPI(CONFIG, rsi_mode="relative", max_cartesian_rate=RATE_LIMIT)
    for axis in ("X", "Y", "Z"):
        api.safety.set_limit(f"RKorr.{axis}", -LIMIT, LIMIT)

    api.start()
    print("Waiting for the robot - press Start on the pendant now (up to 120 s)...")
    if not api.wait_for_connection(120):
        print("\nNO PACKETS. Did the KRL program reach RSI_ON? Is RSIPI_Stop.rsi")
        print("on the controller with RSI_EthernetConfig_Stop.xml beside it?")
        api.stop()
        sys.exit(1)

    print("\nCONNECTED.\n")
    results = {}
    try:
        moved = test_corrections_still_work(api)
        results["corrections still work"] = moved
        if moved:
            results["MoveStop ends MOVECORR"] = test_exit_movecorr(api)
        else:
            print("\nSkipping test 2 - corrections are broken, so ending")
            print("MOVECORR would prove nothing.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            api.client.zero_corrections()
        except Exception as e:
            print(f"cleanup warning: {type(e).__name__}: {e}")
        api.stop()

    print("\n" + "=" * 60)
    for name, ok in results.items():
        print(f"  {name:<26} {'PASS' if ok else 'SKIPPED' if ok is None else 'FAIL'}")
    print("=" * 60)
    if results.get("MoveStop ends MOVECORR"):
        print("STOP object works. exit_movecorr() can now end a sensor-guided")
        print("motion, and STOP can be added to RSIPI_Basic and RSIPI_Joints.")
    elif results.get("corrections still work"):
        print("Corrections survive the STOP object (the old bug is fixed), but")
        print("MoveStop did not end MOVECORR - see the suggestions above.")
    else:
        print("The STOP object still disables corrections. Do not ship it.")
