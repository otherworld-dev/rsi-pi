import sys

from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:

    #   python examples/dry_run.py examples/example_06_logging_csv.py

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)
    api.logging.start()

    api.start()

    print("Logging robot data to CSV. Press Enter to stop.")
    input()

    api.stop()
