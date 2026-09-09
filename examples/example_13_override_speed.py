"""Program override as a speed dial - does $OV_PRO touch RSI corrections?

Needs a context with an OV_PRO object (MAP2OV_PRO) - Max is the only shipped
context that has one, so this defaults to context("max") like example 11.

$OV_PRO scales the speed of PROGRAMMED KRL motion, that much is certain from
the object reference. Whether it also scales corrections sent over RSI is an
open question this script MEASURES rather than assumes: it times the same
10 mm move at 50% and at 100% override. The clock runs until the ROBOT's
applied correction (POSCORRMON) settles at the target, not until RSIPI's
executor returns - the executor is paced by IPOC and always takes the same
time. If override reaches RSI corrections too, the 50% move should settle
roughly twice as late; if RSI corrections bypass the program override, the
two durations should come out about equal.
Treat the result as unverified until it has been checked against the
hardware findings from an actual lab session.

The echo server always echoes OvProW straight back as OvPro (a read/write
round trip) and never changes how fast it applies RKorr, so under dry_run
the two durations will come out about equal regardless of what a real
controller does - that proves the read/write path works, not the physics.

    python examples/example_13_override_speed.py
    python examples/dry_run.py examples/example_13_override_speed.py
"""
import sys
import time

from RSIPI import RSIAPI, context

from _confirm import confirm

DISTANCE = 10.0        # mm, X axis
SPEED = 5.0             # mm/s, approx
STEPS = 100
CYCLES_PER_STEP = 5     # 100 steps x 5 cycles x 4 ms = 2 s -> 10 mm / 2 s = 5 mm/s


def timed_move(api, end_pose, delta_x):
    """Run a Cartesian move; return (robot seconds, applied correction).

    The executor is paced by the robot's IPOC clock, so it always returns
    after STEPS x CYCLES_PER_STEP cycles whatever the robot did with the
    corrections. What override could change is the ROBOT's response, which
    is the applied correction (POSCORRMON): keep watching it after the
    executor returns until it settles at the target, and time that.
    """
    target = api.monitoring.get_applied_correction()["X"] + delta_x
    start = time.time()
    api.motion.move_cartesian_trajectory(end_pose, steps=STEPS, cycles_per_step=CYCLES_PER_STEP)
    executor = time.time() - start

    deadline = start + 4 * STEPS * CYCLES_PER_STEP * 0.004 + 2.0
    while time.time() < deadline:
        applied = api.monitoring.get_applied_correction()
        if abs(applied["X"] - target) < 0.3:
            break
        time.sleep(0.02)
    robot = time.time() - start
    print(f"   executor {executor:.3f} s, robot settled {robot:.3f} s "
          f"(target X {target:+.1f} mm)")
    return robot, applied


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else context("max")
    api = RSIAPI(CONFIG)
    api.start()
    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s - is RSI_ON running?")
        api.stop()
        sys.exit(1)

    current = api.monitoring.get_override()
    print(f"Program override: {current}")
    if current is None:
        print("This context declares no OvPro - program override needs a "
              "MAP2OV_PRO object, e.g. context('max'). Exiting.")
        api.stop()
        sys.exit(1)

    print("Checking that out-of-range percentages are rejected:")
    for bad in (0, 101):
        try:
            api.monitoring.set_override(bad)
        except ValueError as e:
            print(f"   set_override({bad}) -> ValueError: {e}")

    duration_50 = None
    duration_100 = None

    try:
        print(api.monitoring.set_override(50))
        time.sleep(0.2)
        print(f"Read back: {api.monitoring.get_override()}%")

        start_pose = api.motion.get_current_pose()
        forward = {"X": start_pose["X"] + DISTANCE}
        detail = (f"{STEPS} steps x {CYCLES_PER_STEP} cycles x 4 ms = 2 s, "
                  f"about {SPEED:.0f} mm/s, at 50% override.")
        if confirm(f"Move X by +{DISTANCE:.0f} mm at 50% override", detail):
            duration_50, applied_50 = timed_move(api, forward, +DISTANCE)
            print(f"   took {duration_50:.3f} s; applied correction: {applied_50}")

        print(api.monitoring.set_override(100))
        time.sleep(0.2)
        print(f"Read back: {api.monitoring.get_override()}%")

        back_pose = api.motion.get_current_pose()
        backward = {"X": back_pose["X"] - DISTANCE}
        detail = (f"{STEPS} steps x {CYCLES_PER_STEP} cycles x 4 ms = 2 s, "
                  f"about {SPEED:.0f} mm/s, at 100% override, returning to start.")
        if confirm(f"Move X back by {DISTANCE:.0f} mm at 100% override", detail):
            duration_100, applied_100 = timed_move(api, backward, -DISTANCE)
            print(f"   took {duration_100:.3f} s; applied correction: {applied_100}")

        print()
        if duration_50 is not None and duration_100 is not None:
            print(f"50% override: {duration_50:.3f} s   100% override: {duration_100:.3f} s")
            print("A slower 50% move would mean $OV_PRO scales RSI corrections too; "
                  "equal durations are the expected result if RSI corrections bypass "
                  "the program override - unverified until checked on hardware.")
        else:
            print("One or both timed moves were skipped - no duration comparison to report.")
    finally:
        # Always leave the robot at full speed, even if a move above raised.
        try:
            print(api.monitoring.set_override(100))
        except Exception as e:
            print(f"cleanup warning: could not restore override: {type(e).__name__}: {e}")
        api.stop()
