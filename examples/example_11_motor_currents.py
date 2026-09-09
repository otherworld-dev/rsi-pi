"""Read the motor currents - at rest, then while one axis works.

MACur is an INTERNAL SEND tag: RSI reads the currents straight from the
controller and it costs no channel. Only the Max and Full contexts declare
it, so this example defaults to context("max"); on any other context
get_motor_currents() raises rather than reporting zeros.

The object reference gives no units, so until they have been compared
against the pendant treat the numbers as relative - what this shows is that
they change when an axis moves. The echo server does not model motor
current: dry_run.py seeds A1 = 12.5 so the read path can be checked, and it
stays at 12.5 through the move. Only the robot can show a rise.

    python examples/example_11_motor_currents.py
    python examples/dry_run.py examples/example_11_motor_currents.py
"""
import sys
import threading
import time

from RSIPI import RSIAPI, context
from RSIPI.exceptions import RSIVariableError

from _confirm import confirm

AXES = ("A1", "A2", "A3", "A4", "A5", "A6")


def sample(api, peaks, seconds=None, while_alive=None, interval=0.1):
    """Print the currents twice a second and keep the largest magnitude seen
    per axis in `peaks`. Runs for `seconds`, or while `while_alive` runs."""
    end = None if seconds is None else time.time() + seconds
    next_print = 0.0
    while True:
        if end is not None and time.time() >= end:
            break
        if while_alive is not None and not while_alive.is_alive():
            break
        currents = api.monitoring.get_motor_currents()
        for axis in AXES:
            peaks[axis] = max(peaks[axis], abs(currents[axis]))
        if time.time() >= next_print:
            print("   " + "  ".join(f"{a}={currents[a]:8.3f}" for a in AXES))
            next_print = time.time() + 0.5
        time.sleep(interval)


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

    try:
        api.monitoring.get_motor_currents()
    except RSIVariableError as e:
        print(f"Cannot read motor currents: {e}")
        api.stop()
        sys.exit(1)

    at_rest = {axis: 0.0 for axis in AXES}
    print("Motor currents at rest (2 s):")
    sample(api, at_rest, seconds=2.0)

    # 5 degrees over 25 steps x 25 cycles x 4 ms = 2.5 s, about 2 deg/s.
    # The move runs in a thread so the currents can be read while it happens.
    if confirm("Rotate joint A1 by 5 degrees while sampling currents",
               "About 2.5 s at roughly 2 deg/s; AKorr is zeroed when it finishes."):
        mover = threading.Thread(
            target=api.motion.move_joint_trajectory,
            args=({"A1": 5.0},), kwargs={"steps": 25, "cycles_per_step": 25},
            daemon=True)
        moving = {axis: 0.0 for axis in AXES}
        print("Motor currents during the move:")
        mover.start()
        sample(api, moving, while_alive=mover)
        mover.join(timeout=5.0)

        print()
        print("Peak |current| per axis:")
        print("   axis     at rest    moving")
        for axis in AXES:
            print(f"   {axis}   {at_rest[axis]:9.3f} {moving[axis]:9.3f}")
        busiest = max(AXES, key=lambda a: moving[a] - at_rest[a])
        print(f"Largest rise: {busiest} (A1 should top this list)")

    api.stop()
