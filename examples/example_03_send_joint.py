import sys

from RSIPI import RSIAPI, context

from _confirm import confirm

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Joint corrections (AKorr) are only declared in the Full config's
    # RECEIVE section - the default RSI_EthernetConfig.xml has no AKorr, so
    # update_joints() would silently no-op (a debug warning, not an
    # exception) against the default config.
    # Override from the command line to run against the echo server:
    #   python examples/dry_run.py examples/example_03_send_joint.py
    CONFIG = sys.argv[1] if len(sys.argv) > 1 else context("joints")
    api = RSIAPI(CONFIG)
    api.start()

    # Move Joint A1 by 10 degrees. Uses the trajectory executor (not a raw
    # update_joints() hold) so the correction is paced one-cycle-at-a-time
    # and AKorr is auto-zeroed once the move completes.
    if confirm("Rotate joint A1 by 10 degrees",
               "Paced one cycle at a time; AKorr is zeroed when it finishes."):
        api.motion.move_joint_trajectory({"A1": 10})

    api.stop()
