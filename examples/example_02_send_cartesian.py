from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI('RSI_EthernetConfig.xml')
    api.start()

    # Move TCP 50mm along X-axis. Uses the trajectory executor (not a raw
    # update_cartesian() hold) so the 50mm delta is paced one-cycle-at-a-time
    # and RKorr is auto-zeroed when the move completes - a raw
    # update_cartesian(X=50, ...) call would instead retransmit X=50 as a
    # per-cycle relative delta on every ~4ms cycle for as long as it stays
    # unchanged (12.5 m/s at 250Hz with the default's no rate limit).
    api.motion.move_cartesian_trajectory({"X": 50, "Y": 0, "Z": 0}, steps=50)

    api.stop()
