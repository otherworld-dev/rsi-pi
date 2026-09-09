import sys

from RSIPI import RSIAPI
import time

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:

    #   python examples/dry_run.py examples/example_08_safety_limits.py

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)

    # Set X axis soft limits. Default rsi_mode='relative', so RKorr.X is a
    # PER-CYCLE delta (mm applied every ~4ms cycle), not an absolute
    # position - +/-2.0mm/cycle caps commanded speed at ~500 mm/s, a
    # realistic bound for a per-cycle delta.
    api.safety.set_limit("RKorr.X", -2.0, 2.0)

    api.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        api.stop()
