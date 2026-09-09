"""Queue several trajectory legs, run them together, then cancel one mid-flight.

Shows the parts of the trajectory queue that no other example covers:
queue_cartesian_trajectory(), queue_joint_trajectory(), get_queue(),
execute_queued_trajectories(), and cancel_trajectory(). Needs context("joints")
because it queues both a Cartesian leg (RKorr) and a joint leg (AKorr) -
"joints" is the smallest context that declares both, and the emulator
integrates both corrections into RIst/AIPos, so get_position() visibly
advances during dry_run. get_applied_correction() (POSCORRMON) needs
context("max") and comes back empty here - that is expected, not a bug.

Cancellation is only this clean because rsi_mode stays at its default
'relative'. The queue methods build ABSOLUTE waypoints internally, but
execute_queued_trajectories() runs them through execute_trajectory() with
points='world', and in relative mode that path (_execute_per_cycle ->
_execute_deltas) converts them to per-cycle deltas and zeros the correction
in a `finally` block whether the leg finishes or is cancelled - so the robot
stops accumulating further correction the instant cancel_trajectory() takes
effect. execute_queued_trajectories() also clears trajectory_queue itself
once its loop exits, cancelled or not, so "does the queue still hold
anything after cancel" is answered before we even ask.

    python examples/example_14_trajectory_queue.py
    python examples/dry_run.py examples/example_14_trajectory_queue.py
"""
import sys
import threading
import time

from RSIPI import RSIAPI, context

from _confirm import confirm


def print_queue(api):
    queue = api.motion.get_queue()
    if not queue:
        print("  (queue is empty)")
        return
    print(f"  {'idx':>3}  {'space':<10}{'waypoints':>10}  {'cycles/pt':>9}  {'s/pt':>6}")
    for i, item in enumerate(queue):
        print(f"  {i:>3}  {item['space']:<10}{item['steps']:>10}  "
              f"{item['cycles_per_step']:>9}  {item['rate']:>6.3f}")


def print_position(label, api):
    pos = api.monitoring.get_position()
    print(f"{label}: X={pos['X']:.2f} Y={pos['Y']:.2f} Z={pos['Z']:.2f}")


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

    pos_start = api.monitoring.get_position()
    print(f"Start position: X={pos_start['X']:.2f} Y={pos_start['Y']:.2f} Z={pos_start['Z']:.2f}")

    # Chain each leg's start_pose from the previous leg's calculated end, all
    # anchored to the real current position - queue_cartesian_trajectory()
    # builds absolute-space waypoints, and in relative mode the FIRST waypoint
    # of a leg is diffed against the robot's actual position at execution
    # time, so an arbitrary start_pose (e.g. all zeros) would command a jump
    # equal to the real position on the very first cycle.
    leg1_start = {"X": pos_start["X"], "Y": pos_start["Y"], "Z": pos_start["Z"]}
    leg1_end = dict(leg1_start, X=leg1_start["X"] + 10.0)
    # 50 steps x 10 cycles x 4 ms = 2 s for 10 mm -> 5 mm/s
    api.motion.queue_cartesian_trajectory(leg1_start, leg1_end, steps=50, cycles_per_step=10)

    leg2_start = dict(leg1_end)
    leg2_end = dict(leg2_start, Y=leg2_start["Y"] + 10.0)
    api.motion.queue_cartesian_trajectory(leg2_start, leg2_end, steps=50, cycles_per_step=10)

    joints_start = api.motion.get_current_joints()
    joints_end = dict(joints_start, A1=joints_start["A1"] + 3.0)
    # 30 steps x 25 cycles x 4 ms = 3 s for 3 deg -> 1 deg/s; 25 is the
    # default move_joint_trajectory() uses, since joints move slower than TCP.
    api.motion.queue_joint_trajectory(joints_start, joints_end, steps=30, cycles_per_step=25)

    print("Queued legs:")
    print_queue(api)

    if confirm("Execute the queued legs (X +10mm, Y +10mm, then A1 +3deg)",
               "20mm Cartesian at ~5mm/s (~4s), then 3deg on A1 at ~1deg/s "
               "(~3s); about 7s total."):
        mover = threading.Thread(target=api.motion.execute_queued_trajectories, daemon=True)
        mover.start()
        while mover.is_alive():
            print_position("  running", api)
            time.sleep(0.5)
        mover.join(timeout=2.0)
    else:
        print("Skipped executing the queue.")

    pos_after_queue = api.monitoring.get_position()
    print(f"Position after queue: X={pos_after_queue['X']:.2f} "
          f"Y={pos_after_queue['Y']:.2f} Z={pos_after_queue['Z']:.2f}")

    # Guarantee a clean queue for the cancel demo below regardless of whether
    # the confirm() above was accepted (execute_queued_trajectories() already
    # clears it on completion, but not if it was never called).
    api.motion.clear_queue()

    pos_now = api.monitoring.get_position()
    leg3_start = {"X": pos_now["X"], "Y": pos_now["Y"], "Z": pos_now["Z"]}
    leg3_end = dict(leg3_start, Z=leg3_start["Z"] + 10.0)

    if confirm("Start one more leg (Z +10mm at ~5mm/s) and cancel it partway",
               "10mm in +Z at ~5mm/s (~2s total); cancel_trajectory() is "
               "called after about 1s, roughly halfway."):
        api.motion.queue_cartesian_trajectory(leg3_start, leg3_end, steps=50, cycles_per_step=10)
        print("Queue before executing leg 3:")
        print_queue(api)

        mover2 = threading.Thread(target=api.motion.execute_queued_trajectories, daemon=True)
        mover2.start()
        time.sleep(1.0)
        api.motion.cancel_trajectory()
        mover2.join(timeout=3.0)

        pos_after_cancel = api.monitoring.get_position()
        print(f"Position where it stopped: X={pos_after_cancel['X']:.2f} "
              f"Y={pos_after_cancel['Y']:.2f} Z={pos_after_cancel['Z']:.2f}")
        applied = api.monitoring.get_applied_correction()
        if applied:
            print(f"Applied correction after cancel: {applied}")
        else:
            print("Applied correction after cancel: (empty - needs "
                  "context(\"max\") for POSCORRMON)")
        print("Queue after cancel (already cleared by execute_queued_trajectories()):")
        print_queue(api)
    else:
        print("Skipped the cancel demo.")
        pos_after_cancel = api.monitoring.get_position()

    api.motion.clear_queue()

    print()
    print("Summary (X, Y, Z in mm):")
    print(f"  start:        X={pos_start['X']:.2f} Y={pos_start['Y']:.2f} Z={pos_start['Z']:.2f}")
    print(f"  after queue:  X={pos_after_queue['X']:.2f} Y={pos_after_queue['Y']:.2f} Z={pos_after_queue['Z']:.2f}")
    print(f"  after cancel: X={pos_after_cancel['X']:.2f} Y={pos_after_cancel['Y']:.2f} Z={pos_after_cancel['Z']:.2f}")

    api.stop()
