from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.logging.start()

    api.start()

    print("Logging robot data to CSV. Press Enter to stop.")
    input()

    api.stop()
