import sys

from RSIPI import RSIAPI

from _confirm import confirm

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:

    #   python examples/dry_run.py examples/example_09_trajectory_cartesian.py

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)
    api.start()

    # Plan a 50mm square trajectory as per-cycle deltas (mode='relative'
    # matches the default rsi_mode='relative'), then execute it through the
    # trajectory engine, which paces each delta against the robot's IPOC
    # clock and applies it exactly once. Raw update_cartesian(X=50) followed
    # by time.sleep() would instead hold that 50mm delta for ~125 robot
    # cycles (12.5 m/s) - never sleep-loop a correction in relative mode.
    points = [
        {"X": 0, "Y": 0, "Z": 0},
        {"X": 50, "Y": 0, "Z": 0},
        {"X": 50, "Y": 50, "Z": 0},
        {"X": 0, "Y": 50, "Z": 0},
        {"X": 0, "Y": 0, "Z": 0}
    ]

    trajectory = []
    for start, end in zip(points, points[1:]):
        trajectory += api.motion.generate_trajectory(start, end, steps=50, mode='relative')

    if confirm("Trace a 50 mm square with the TCP",
               f"{len(trajectory)} waypoints, one per robot cycle. The robot WILL move."):
        api.motion.execute_trajectory(trajectory, space='cartesian', points='delta')

    api.stop()
