from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.start()

    # Move TCP 50mm along X-axis
    api.motion.update_cartesian(X=50, Y=0, Z=0)

    api.stop()
