"""Move a chosen distance in one axis, report what the robot did, move back.

A diagnostic for the lab: when a move stops or the controller complains,
this isolates the distance from everything else (no E-stop, no trajectory
tricks - one paced straight line, then the same line back).

    python examples/move_probe.py 20            # +20 mm in X, Basic context
    python examples/move_probe.py 20 --axis Z
    python examples/move_probe.py 8 --axis A    # degrees
    python examples/move_probe.py 20 --config src/RSIPI/contexts/RSI_EthernetConfig_Joints.xml

Speed is fixed at 0.05 mm (or deg) per cycle = 12.5 per second; the
per-cycle clamp in the network process is 0.1.
"""
import argparse
import sys
import time

from RSIPI import RSIAPI, context

from _confirm import confirm

STEP = 0.05          # mm or deg per robot cycle: 12.5 per second at 4 ms

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    p = argparse.ArgumentParser()
    p.add_argument("distance", type=float, help="mm (X/Y/Z) or degrees (A/B/C), signed")
    p.add_argument("--axis", default="X", choices=list("XYZABC"))
    p.add_argument("--config", default=None, help="Ethernet config (default: the Basic context)")
    args = p.parse_args()

    api = RSIAPI(args.config or context("basic"), rsi_mode="relative", max_cartesian_rate=0.1)
    api.start()
    print("Listening - run the program to RSI_ON now if it is not already.")
    if not api.wait_for_connection(120.0):
        print("No packets from the robot in 120 s.")
        api.stop()
        sys.exit(1)

    def report(label, start):
        pose = api.motion.get_current_pose()
        print(f"  {label}: {args.axis}={pose[args.axis]:.2f}  "
              f"(moved {pose[args.axis] - start[args.axis]:+.2f})   "
              f"X={pose['X']:.2f} Y={pose['Y']:.2f} Z={pose['Z']:.2f} "
              f"A={pose['A']:.2f} B={pose['B']:.2f} C={pose['C']:.2f}")

    steps = max(1, int(abs(args.distance) / STEP))
    start = api.motion.get_current_pose()
    report("start", start)
    unit = "deg" if args.axis in "ABC" else "mm"
    if confirm(f"Move {args.distance:+.1f} {unit} in {args.axis}",
               f"{steps} steps of {STEP} {unit}, one per cycle: about {STEP * 250:.1f} {unit}/s."):
        t0 = time.time()
        try:
            api.motion.move_cartesian_trajectory({args.axis: start[args.axis] + args.distance}, steps=steps)
            print(f"  move finished in {time.time() - t0:.2f} s")
        except Exception as e:
            print(f"  move ABORTED after {time.time() - t0:.2f} s: {type(e).__name__}: {e}")
        time.sleep(0.5)
        report("after", start)
        if api.wait_for_connection(0.0) and api.diagnostics.check_watchdog():
            print("  the robot has STOPPED SENDING - read the pendant message now")
        elif confirm(f"Move back {-args.distance:+.1f} {unit} in {args.axis}", "same speed"):
            api.motion.move_cartesian_trajectory({args.axis: start[args.axis]}, steps=steps)
            time.sleep(0.5)
            report("back", start)
    api.stop()
