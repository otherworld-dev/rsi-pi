"""
Basic I/O Handshake Example

Demonstrates simple bidirectional signaling between Python and KRL using
digital I/O channels. Works with templates/krl/basic_handshake.src

Flow:
1. KRL signals "ready" on output 1
2. Python waits for signal
3. Python performs processing
4. Python signals "complete" on input 1
5. KRL continues

Usage:
    python 01_basic_handshake.py --config RSI_EthernetConfig.xml
"""

import argparse
import time
import logging
from RSIPI import RSIAPI

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def basic_handshake_example(config_file: str) -> None:
    """
    Execute basic I/O handshake with KRL program.

    Args:
        config_file: Path to RSI configuration XML file
    """
    # Initialize API
    api = RSIAPI(config_file)

    try:
        # Start RSI communication
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        # Wait for KRL to signal ready (digital output 1)
        logging.info("Waiting for KRL ready signal...")

        if api.krl.wait_for_signal(1, timeout=30.0):
            logging.info("✅ KRL signaled ready!")

            # Simulate Python processing (e.g., data analysis, sensor reading)
            logging.info("Performing Python-side processing...")
            time.sleep(2.0)  # Simulated processing time

            # Optional: Do actual work here
            # process_sensor_data()
            # calculate_corrections()
            # update_database()

            logging.info("✅ Processing complete")

            # Signal completion back to KRL (digital input 1)
            api.krl.signal_complete(1)
            logging.info("✅ Signaled KRL to continue")

            # KRL will now proceed with its motion program
            logging.info("KRL is now free to continue motion")

        else:
            logging.error("❌ Timeout waiting for KRL ready signal")
            logging.error("Check that KRL program is running and I/O is configured correctly")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during handshake: {e}")

    finally:
        # Clean shutdown
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Basic I/O Handshake Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Basic I/O Handshake Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    basic_handshake_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
