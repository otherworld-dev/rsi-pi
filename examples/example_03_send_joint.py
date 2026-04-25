from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.start()

    # Move Joint A1 by 10 degrees
    api.motion.update_joints(A1=10)

    api.stop()
