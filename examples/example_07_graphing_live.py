from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.viz.start_live_plot()

    api.start()

    print("Live graphing started. Press Enter to stop.")
    input()

    api.stop()
