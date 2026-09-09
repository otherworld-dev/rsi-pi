import sys

from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:
    #   python examples/dry_run.py examples/example_01_start_stop.py
    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)

    api.start()

    print("RSI connection started. Press Enter to stop.")
    input()

    api.stop()
    print("RSI connection stopped.")
