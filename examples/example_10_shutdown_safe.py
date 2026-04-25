from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()

    try:
        api.start()
        print("Press Ctrl+C to stop RSI safely.")
        while True:
            pass

    except KeyboardInterrupt:
        print("\nEmergency stop triggered.")
        api.safety.stop()
        api.stop()
