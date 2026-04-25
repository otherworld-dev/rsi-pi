from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.start()

    # Set digital output (e.g., to open gripper)
    api.io.set_output(1, True)

    api.stop()
