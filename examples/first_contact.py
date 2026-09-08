"""Guided first contact with a real robot - three stages, each gated by a prompt.

Stage 1: connect only (no motion) - proves IPOC echo, packet flow, Delay counter.
Stage 2: one 5 mm X move, paced at 0.1 mm/cycle (25 mm/s), then back.
Stage 3: software E-stop drill during a slow move - verifies zeroing and
         that reset does NOT resume motion.

Usage:
    python examples/first_contact.py [config.xml] [--absolute]

The mode MUST match the KRL program: RSI_ON(#RELATIVE) is the default here,
RSI_ON(#ABSOLUTE) needs --absolute. A mismatch usually looks like "the robot
does not move at all" rather than an error.

Order matters: the ETHERNET object breaks off (RSIBad) 0.4 s after RSI_ON
if nothing answers, so the PC must be listening FIRST.
  1. Pendant: T1 mode, override ~10%, select RSIPI_Minimal, press Start -
     it BCO-runs, then HALTs with "Start the Python sender, then press Start".
  2. PC: run this script (repo root, venv python) - it binds and waits up to
     30 s for the robot's first packet.
  3. Pendant: press Start again - RSI_ON runs and Stage 1 connects.
Hand on the enabling switch and the hardware E-stop throughout.
"""
import sys
import time

from RSIPI import RSIAPI, context

# Must be the SAME file the controller's ETHERNET object loads.
# Override from the command line, e.g. to test against a known-good context:
#   python examples/first_contact.py "RSI Configuration files/Current/MScEthernetConfig.xml"
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
CONFIG = (_args[0] if _args
          else context("basic"))
# MUST match the KRL side: RSI_ON(#RELATIVE) or RSI_ON(#ABSOLUTE). Get this
# wrong and the robot appears not to move: relative deltas read as absolute
# offsets are tiny and never accumulate.
RSI_MODE = "absolute" if "--absolute" in sys.argv else "relative"
RATE_LIMIT = 0.1                         # mm per cycle clamp (25 mm/s at 4 ms)
LIMIT = 6.0                              # +/- mm per-cycle safety limit on RKorr


def ask(prompt):
    answer = input(f"\n{prompt} [yes/NO]: ").strip().lower()
    return answer == "yes"


def delay_count(api):
    delay = api.client.send_variables.get("Delay")
    return delay.get("D") if isinstance(delay, dict) else delay


def stage1(api):
    print("\n== Stage 1: connection only ==")
    print("Listening. NOW press Start on the pendant (the program is HALTed before RSI_ON).")
    print("Waiting up to 120 s for the robot's first packet...")
    if not api.wait_for_connection(120):
        print("NO PACKETS in 30 s. Check: did you press Start on the pendant after "
              "the HALT (so RSI_ON ran)? PC IP matches IP_NUMBER? Firewall allows "
              "python inbound UDP?")
        return False
    print("Connected. Monitoring for 10 s (robot holds position)...")
    faults = 0
    for _ in range(10):
        time.sleep(1)
        pose = api.motion.get_current_pose()
        d = delay_count(api)
        print(f"  RIst X={pose['X']:.2f} Y={pose['Y']:.2f} Z={pose['Z']:.2f}  Delay={d}")
        if d and float(d) > 0:
            faults += 1
    healthy = api.diagnostics.is_healthy() if hasattr(api, "diagnostics") else True
    print(f"Stage 1 result: connected, late-packet counter {'clean' if not faults else 'NON-ZERO'}, "
          f"healthy={healthy}")
    return True


def stage2(api):
    print("\n== Stage 2: 5 mm move in X and back ==")
    if not ask("Robot will move 5 mm in +X at ~25 mm/s, then return. Proceed?"):
        print("Skipped.")
        return
    start = api.motion.get_current_pose()
    print(f"  start X={start['X']:.2f}")
    api.motion.move_cartesian_trajectory({"X": start["X"] + 5.0}, steps=50)
    time.sleep(0.5)
    mid = api.motion.get_current_pose()
    moved = mid["X"] - start["X"]
    print(f"  after +5: X={mid['X']:.2f} (moved {moved:+.2f} mm)")
    if abs(moved) < 0.3:
        print("  !! The robot did not move. The usual cause is a mode mismatch:")
        print(f"     this script is running rsi_mode={RSI_MODE!r}, so the KRL")
        print(f"     program must use RSI_ON(#{RSI_MODE.upper()}). Check the")
        print("     pendant, and re-run with --absolute if KRL says #ABSOLUTE.")
    api.motion.move_cartesian_trajectory({"X": start["X"]}, steps=50)
    time.sleep(0.5)
    end = api.motion.get_current_pose()
    print(f"  back:     X={end['X']:.2f} (net {end['X'] - start['X']:+.2f} mm)")


def stage3(api):
    print("\n== Stage 3: software E-stop drill ==")
    # In absolute mode the correction IS the total offset, so the move must
    # stay inside the safety limit or it aborts on RKorr.X before the E-stop
    # can prove anything. In relative mode each delta is tiny, so a longer
    # move is fine and gives the E-stop more room to land mid-motion.
    distance = (LIMIT - 1.0) if RSI_MODE == "absolute" else 20.0
    if not ask(f"Robot will start a slow {distance:.0f} mm +X move; "
               f"E-stop fires after 1 s. Proceed?"):
        print("Skipped.")
        return
    start = api.motion.get_current_pose()

    import threading
    fired = threading.Event()

    def fire():
        time.sleep(1.0)
        api.safety.stop()
        fired.set()
        print("  E-STOP sent")

    thread = threading.Thread(target=fire, daemon=True)
    thread.start()
    try:
        api.motion.move_cartesian_trajectory({"X": start["X"] + distance}, steps=400)
    except Exception as e:
        print(f"  trajectory aborted: {type(e).__name__}")
    # The move can finish or abort before the E-stop lands; wait for it, or
    # the reset below would run first and leave the E-stop latched.
    thread.join(timeout=5)
    if not fired.is_set():
        print("  WARNING: E-stop never fired")
    time.sleep(0.5)
    stopped = api.motion.get_current_pose()
    print(f"  stopped at X={stopped['X']:.2f} "
          f"(moved {stopped['X'] - start['X']:+.2f} of {distance:.0f} mm)")

    api.safety.reset()
    print("  reset sent - robot must NOT move now; watching 3 s...")
    time.sleep(3)
    after = api.motion.get_current_pose()
    print(f"  after reset X={after['X']:.2f} (drift {after['X'] - stopped['X']:+.2f} mm)")
    if ask("Return to the start position (slow)?"):
        api.motion.move_cartesian_trajectory({"X": start["X"]}, steps=400)


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    print(f"Config: {CONFIG}")
    print(f"Mode:   rsi_mode={RSI_MODE!r} - the KRL side must use "
          f"RSI_ON(#{RSI_MODE.upper()})")
    api = RSIAPI(CONFIG, rsi_mode=RSI_MODE, max_cartesian_rate=RATE_LIMIT)
    api.safety.set_limit("RKorr.X", -LIMIT, LIMIT)
    api.safety.set_limit("RKorr.Y", -LIMIT, LIMIT)
    api.safety.set_limit("RKorr.Z", -LIMIT, LIMIT)
    api.start()
    try:
        if stage1(api):
            stage2(api)
            stage3(api)
    finally:
        # zero_corrections() writes straight to the shared state, so it works
        # even with an E-stop latched - update_cartesian() would be rejected
        # by validation and crash the cleanup.
        try:
            api.client.zero_corrections()
            if api.safety.is_stopped():
                api.safety.reset()
        except Exception as e:
            print(f"cleanup warning: {type(e).__name__}: {e}")
        api.stop()
        print("\nStopped. Cancel RSIPI_Minimal on the pendant.")
