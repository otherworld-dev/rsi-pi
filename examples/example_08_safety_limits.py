from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()

    # Set X axis soft limits
    api.safety.set_limit("RKorr.X", -500.0, 500.0)

    api.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        api.stop()
