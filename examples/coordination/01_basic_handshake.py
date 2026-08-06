"""
Basic I/O Handshake Example

Demonstrates simple bidirectional signaling between Python and KRL using
digital I/O channels. Works with templates/krl/basic_handshake.src

Flow:
1. An external/physical signal indicates "ready" on digital input 1
   (DiL bit 0 / $IN[1] - hardware/PLC-driven; RSIPI's IOAPI cannot read
   back a robot output (Digout.o1) as an "input" channel, so this leg
   cannot be driven by the KRL program's own $OUT[1] assignment)
2. Python waits for signal
3. Python performs processing
4. Python signals "complete" to KRL via digital output 1
   (DiO word, bit 0 - the robot receives this as a mapped $OUT bit,
   e.g. $OUT[20] per RSI_EthernetConfig_Full.xml's channel map)
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

        # Wait for the ready signal on digital input 1 (DiL bit 0 / $IN[1]).
        # group=None makes IOAPI auto-detect the DiL word - the default
        # group='Digin' does not exist in either shipped config and raises
        # immediately.
        logging.info("Waiting for ready signal on input 1...")

        if api.krl.wait_for_signal(1, timeout=30.0, group=None):
            logging.info("✅ Ready signal received!")

            # Simulate Python processing (e.g., data analysis, sensor reading)
            logging.info("Performing Python-side processing...")
            time.sleep(2.0)  # Simulated processing time

            # Optional: Do actual work here
            # process_sensor_data()
            # calculate_corrections()
            # update_database()

            logging.info("✅ Processing complete")

            # Signal completion back to KRL via digital output 1 (DiO bit 0).
            # group=None makes IOAPI auto-detect the DiO word - the default
            # group='Digout' targets a SEND-only echo group and raises.
            api.krl.signal_complete(1, group=None)
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
