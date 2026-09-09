"""Detect contact from motor current, the follow-on to example 11.

MACur is an INTERNAL SEND tag, so this needs a context that declares it -
Max or Full - and defaults to context("max") like example 11: sample
currents at rest for a baseline, then watch the idle axes for a deviation
past MARGIN while A1 rotates, and stop the instant one crosses it. A1
itself is excluded - a driven axis draws more current just by moving - and
is reported separately.

The object reference gives no units for MACur, so MARGIN below is a guess
and must be re-tuned once the scale has been compared against the pendant -
see example 11 for the same caveat on the raw readings.

The echo server does not model motor current: dry_run.py seeds MACur.A1 =
12.5 and it never changes, and every other axis stays at 0. Baseline and
live samples are therefore identical throughout, so under dry_run this
script always reports "no contact" - that is the expected, correct result,
not a bug. Only the robot can show a real deviation.

    python examples/example_12_contact_detection.py
    python examples/dry_run.py examples/example_12_contact_detection.py
"""
import sys
import threading
import time

from RSIPI import RSIAPI, context
from RSIPI.exceptions import RSIVariableError

from _confirm import confirm

AXES = ("A1", "A2", "A3", "A4", "A5", "A6")

# MACur's units are not documented anywhere in the KUKA object reference, so
# this margin is an arbitrary starting point in whatever units MACur reports
# and MUST be re-tuned once the scale is known against the pendant.
MARGIN = 2.0

# The axis being driven is excluded from the check: its current rises
# because it is moving, not because it hit anything. Contact shows on the
# axes that should be idle. Including the moving axis needs a baseline
# recorded during an identical unobstructed move, which is the next step
# once the units are known.
MOVING_AXIS = "A1"


def contact_axes(baseline, currents, margin, ignore=()):
    """Axes whose |current - baseline| exceeds margin.

    Pure function (no I/O), so it can be unit-tested directly.

    Args:
        baseline: {axis: float} mean current at rest
        currents: {axis: float} one live sample
        margin: threshold in MACur's own (currently unknown) units
        ignore: axes to leave out - the one being driven, whose current
            rises simply because it is moving

    Returns:
        List of axis names exceeding margin, in AXES order.
    """
    return [axis for axis in AXES if axis not in ignore
            and abs(currents.get(axis, 0.0) - baseline.get(axis, 0.0)) > margin]


def sample_at_rest(api, seconds=2.0, interval=0.02):
    """Sample currents while idle; return (baseline mean, spread) per axis."""
    samples = {axis: [] for axis in AXES}
    end = time.time() + seconds
    while time.time() < end:
        currents = api.monitoring.get_motor_currents()
        for axis in AXES:
            samples[axis].append(currents[axis])
        time.sleep(interval)
    baseline = {axis: sum(v) / len(v) for axis, v in samples.items()}
    spread = {axis: max(v) - min(v) for axis, v in samples.items()}
    return baseline, spread


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

    print("Sampling currents at rest (2 s) for a baseline...")
    baseline, spread = sample_at_rest(api, seconds=2.0)
    print("   axis     mean    spread")
    for axis in AXES:
        print(f"   {axis}   {baseline[axis]:9.3f} {spread[axis]:9.3f}")

    detail = ("50 steps x 25 cycles x 4 ms = 5 s, about 2 deg/s. Cancels and "
              f"holds the instant any axis crosses MARGIN ({MARGIN}) from "
              "its baseline.")
    if confirm("Rotate joint A1 by 10 deg while polling motor current for contact", detail):
        mover = threading.Thread(
            target=api.motion.move_joint_trajectory,
            args=({"A1": 10.0},), kwargs={"steps": 50, "cycles_per_step": 25},
            daemon=True)
        peak = {axis: 0.0 for axis in AXES}
        tripped = []
        start = time.time()
        mover.start()
        while mover.is_alive():
            currents = api.monitoring.get_motor_currents()
            for axis in AXES:
                peak[axis] = max(peak[axis], abs(currents[axis] - baseline[axis]))
            hit = contact_axes(baseline, currents, MARGIN, ignore=(MOVING_AXIS,))
            if hit:
                tripped = hit
                api.motion.cancel_trajectory()
                api.client.zero_corrections()   # hold position; direct write, safe under E-stop too
                break
            time.sleep(0.02)
        mover.join(timeout=5.0)
        elapsed = time.time() - start

        print()
        if tripped:
            print(f"CONTACT after {elapsed:.2f} s - tripped: {', '.join(tripped)}")
            for axis in tripped:
                print(f"   {axis}: {peak[axis]:.3f} over margin {MARGIN} "
                      f"(baseline {baseline[axis]:.3f})")
        else:
            print(f"No contact after {elapsed:.2f} s (move ran to completion).")
        print("Peak |deviation from baseline| per axis:")
        for axis in AXES:
            note = "  (driven axis, not checked)" if axis == MOVING_AXIS else ""
            print(f"   {axis}   {peak[axis]:9.3f}{note}")

    api.stop()
