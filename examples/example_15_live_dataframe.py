"""Analyse live RSI data in pandas/numpy instead of logging to CSV first.

Needs context("joints") (the default): only RIst (position feedback, every
context has it) and RKorr (Cartesian corrections, every context has it) are
used, so this runs on any shipped context.

get_live_data_as_dataframe() and get_live_data_as_numpy() each return ONE
snapshot, not a rolling buffer - the DataFrame's cells are the nested dicts
straight out of get_live_data() (position/velocity/acceleration/force), and
the numpy array is those same four rows padded to 6 columns. Neither is
useful for time-series analysis by itself; this script samples repeatedly,
flattens each snapshot, and concatenates them into one DataFrame with a
timestamp column - that is the actual "log a session, then analyse it in
memory" pattern.

The echo server integrates RKorr directly into RIst with no robot dynamics,
so the velocity seen here tracks the commanded rate closely. On the robot
expect a softer ramp up and down and some lag behind the command. Also
expect coarser numbers: the shipped contexts set the ETHERNET object's
Precision to 1 decimal place, so RIst arrives quantised to 0.1 mm, and a
velocity differenced over the 50 ms sample interval below moves in 2 mm/s
steps - average over many samples before reading anything into it.

    python examples/example_15_live_dataframe.py
    python examples/dry_run.py examples/example_15_live_dataframe.py
"""
import sys
import threading
import time

import numpy as np
import pandas as pd

from RSIPI import RSIAPI, context

from _confirm import confirm

# get_live_data() nests its fields as {"position": {...}, "velocity": {...},
# "acceleration": {...}, "force": {...}, "ipoc": ...}; RIst/Velocity/
# Acceleration/MACur are the wire names for those sections.
_SECTION_PREFIX = {
    "position": "RIst", "velocity": "Velocity",
    "acceleration": "Acceleration", "force": "MACur",
}


def flat_snapshot(api):
    """One get_live_data() call, flattened into a single-row dict."""
    data = api.monitoring.get_live_data()
    row = {"timestamp": time.time(), "ipoc": data["ipoc"]}
    for section, prefix in _SECTION_PREFIX.items():
        for axis, value in data[section].items():
            row[f"{prefix}_{axis}"] = value
    return row


def sample_dataframe(api, seconds=None, while_alive=None, interval=0.05):
    """Sample flat_snapshot() into a DataFrame for `seconds`, or while
    `while_alive` runs."""
    rows = []
    end = None if seconds is None else time.time() + seconds
    while True:
        if end is not None and time.time() >= end:
            break
        if while_alive is not None and not while_alive.is_alive():
            break
        rows.append(flat_snapshot(api))
        time.sleep(interval)
    return pd.DataFrame(rows)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else context("joints")
    api = RSIAPI(CONFIG)
    api.start()
    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s - is RSI_ON running?")
        api.stop()
        sys.exit(1)

    # Show what the single-snapshot helpers actually return, once, before
    # building a real buffer out of them.
    one_df = api.monitoring.get_live_data_as_dataframe()
    one_arr = api.monitoring.get_live_data_as_numpy()
    print("get_live_data_as_dataframe() - one row, nested-dict cells:")
    print(f"   shape={one_df.shape} columns={list(one_df.columns)}")
    print("get_live_data_as_numpy() - one snapshot, padded to 6 columns:")
    print(f"   shape={one_arr.shape}\n{one_arr}")

    print("\nSampling at rest for 2 s (20 ms interval)...")
    at_rest = sample_dataframe(api, seconds=2.0)
    print(f"DataFrame shape: {at_rest.shape}")
    print(f"Columns: {list(at_rest.columns)}")
    print("describe() for RIst X/Y/Z:")
    print(at_rest[["RIst_X", "RIst_Y", "RIst_Z"]].describe())

    pose = api.motion.get_current_pose()
    target_x = pose["X"] + 10.0
    # 50 steps x 10 cycles/step x 4 ms = 2.0 s for 10 mm -> 5 mm/s.
    if confirm("Move the TCP 10 mm in +X while sampling velocity/acceleration",
               "10 mm at ~5 mm/s (50 steps x 10 cycles x 4 ms = 2.0 s)."):
        mover = threading.Thread(
            target=api.motion.move_cartesian_trajectory,
            args=({"X": target_x},), kwargs={"steps": 50, "cycles_per_step": 10},
            daemon=True)
        mover.start()
        moving = sample_dataframe(api, while_alive=mover, interval=0.05)
        mover.join(timeout=5.0)

        if not moving.empty:
            speed = np.sqrt(moving["Velocity_X"] ** 2 + moving["Velocity_Y"] ** 2
                            + moving["Velocity_Z"] ** 2)
            peak = speed.max()
            mean = moving["Velocity_X"].mean()
            print(f"\n{len(moving)} samples during the move.")
            print(f"Peak speed: {peak:.2f} mm/s")
            print(f"Mean Velocity_X: {mean:.2f} mm/s (commanded: 5.00 mm/s)")
            gap = abs(mean - 5.0)
            print(f"   difference from commanded: {gap:.2f} mm/s - "
                  "the emulator integrates corrections directly so this should "
                  "be close; a large gap on the robot would mean POSCORR is "
                  "clamping the correction or the sample loop is falling "
                  "behind the 4 ms cycle.")
        else:
            print("Move finished before the first sample - nothing to report.")

    api.stop()
