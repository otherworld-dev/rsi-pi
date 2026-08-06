from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Joint corrections (AKorr) are only declared in the Full config's
    # RECEIVE section - the default RSI_EthernetConfig.xml has no AKorr, so
    # update_joints() would silently no-op (a debug warning, not an
    # exception) against the default config.
    api = RSIAPI('RSI_EthernetConfig_Full.xml')
    api.start()

    # Move Joint A1 by 10 degrees. Uses the trajectory executor (not a raw
    # update_joints() hold) so the correction is paced one-cycle-at-a-time
    # and AKorr is auto-zeroed once the move completes.
    api.motion.move_joint_trajectory({"A1": 10})

    api.stop()
