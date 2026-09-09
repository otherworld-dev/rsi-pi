import sys

from RSIPI import RSIAPI

from _confirm import confirm

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:
    #   python examples/dry_run.py examples/example_02_send_cartesian.py
    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)
    api.start()

    # Move TCP 50mm along X-axis. Uses the trajectory executor (not a raw
    # update_cartesian() hold) so the 50mm delta is paced one-cycle-at-a-time
    # and RKorr is auto-zeroed when the move completes - a raw
    # update_cartesian(X=50, ...) call would instead retransmit X=50 as a
    # per-cycle relative delta on every ~4ms cycle for as long as it stays
    # unchanged (12.5 m/s at 250Hz with the default's no rate limit).
    # 50 mm over 50 steps is 1 mm per 4 ms cycle - 250 mm/s, which is brisk
    # for a first demo. cycles_per_step spreads each step over several cycles;
    # 10 gives 25 mm/s. RSIPI warns about the fast case either way.
    if confirm("Move the TCP 50 mm in +X",
               "50 steps at 10 cycles each = ~25 mm/s. The robot WILL move."):
        api.motion.move_cartesian_trajectory(
            {"X": 50, "Y": 0, "Z": 0}, steps=50, cycles_per_step=10)

    api.stop()
