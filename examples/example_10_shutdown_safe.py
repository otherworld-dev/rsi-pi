import sys

from RSIPI import RSIAPI
import time

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:

    #   python examples/dry_run.py examples/example_10_shutdown_safe.py

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)

    try:
        api.start()
        print("Press Ctrl+C to stop RSI safely.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nEmergency stop triggered.")
        api.safety.stop()
        api.stop()
