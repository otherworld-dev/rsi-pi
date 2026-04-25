from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()

    api.start()

    print("RSI connection started. Press Enter to stop.")
    input()

    api.stop()
    print("RSI connection stopped.")
