"""
Parameter Passing Example

Demonstrates bidirectional numerical data exchange between Python and KRL
using RSI Tech variables. Works with controller/Program/parameter_passing.src

Flow:
1. KRL writes current position to Tech.C11-C16 (Tech.C is the
   KRL-to-Python channel - see krl_api.py's read_param()/write_param())
2. Python waits for data ready signal
3. Python reads position from Tech.C
4. Python calculates target and writes to Tech.T21-T23 (Tech.T is the
   Python-to-KRL channel)
5. Python signals completion
6. KRL reads target from Tech.T and executes motion

Usage:
    python 02_parameter_passing.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
from _confirm import confirm  # asks before the robot is told to move


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def parameter_passing_example(config_file: str) -> None:
    """
    Execute parameter passing coordination with KRL program.

    Reads current position from KRL, calculates target position,
    and sends target back to KRL for execution.

    Args:
        config_file: Path to RSI configuration XML file
    """
    api = RSIAPI(config_file)

    try:
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        # Wait for the data-ready signal on digital input 1. group=None
        # makes IOAPI auto-detect the DiL word - the default group='Digin'
        # does not exist in either shipped config and raises immediately.
        logging.info("Waiting for KRL data ready signal...")

        if api.krl.wait_for_signal(1, timeout=30.0, group=None):
            logging.info("✅ KRL signaled data ready!")

            # Read current position from Tech.C variables (KRL writes
            # Tech.C, Python reads it - see krl_api.py's read_param()).
            logging.info("Reading current position from KRL...")
            current_x = api.krl.read_param('C11')
            current_y = api.krl.read_param('C12')
            current_z = api.krl.read_param('C13')
            current_a = api.krl.read_param('C14')
            current_b = api.krl.read_param('C15')
            current_c = api.krl.read_param('C16')

            logging.info(f"Current position:")
            logging.info(f"  X: {current_x:.2f} mm")
            logging.info(f"  Y: {current_y:.2f} mm")
            logging.info(f"  Z: {current_z:.2f} mm")
            logging.info(f"  A: {current_a:.2f}°, B: {current_b:.2f}°, C: {current_c:.2f}°")

            # Calculate target position (example: move 100mm in X, 50mm in Y)
            logging.info("Calculating target position...")
            target_x = current_x + 100.0  # Move 100mm in X
            target_y = current_y + 50.0   # Move 50mm in Y
            target_z = current_z + 0.0    # Keep Z constant

            logging.info(f"Calculated target:")
            logging.info(f"  X: {target_x:.2f} mm (+100mm)")
            logging.info(f"  Y: {target_y:.2f} mm (+50mm)")
            logging.info(f"  Z: {target_z:.2f} mm (no change)")

            # Write target position to Tech.T variables for KRL to read
            # (Python writes Tech.T, KRL reads it - see krl_api.py's
            # write_param(). T21-T23 are the only Tech.T slots the
            # default config declares, via DEF_Tech.T2.)
            if not confirm(
                    f"Send KRL a target of X={target_x:.1f} Y={target_y:.1f} "
                    f"Z={target_z:.1f}",
                    "KRL moves the robot there once it is signalled below."):
                logging.info("Target not sent - leaving KRL waiting")
                return
            logging.info("Writing target position to KRL...")
            api.krl.write_param('T21', target_x)
            api.krl.write_param('T22', target_y)
            api.krl.write_param('T23', target_z)
            logging.info("✅ Target position written to Tech.T")

            # Signal KRL that calculation is complete via digital output 1
            # (DiO word, bit 0). group=None makes IOAPI auto-detect it -
            # the default group='Digout' targets a SEND-only echo group
            # and raises.
            api.krl.signal_complete(1, group=None)
            logging.info("✅ Signaled KRL that target is ready")

            # KRL will now read target and execute motion
            logging.info("KRL will now execute motion to calculated target")

        else:
            logging.error("❌ Timeout waiting for KRL data ready signal")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during parameter passing: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Parameter Passing Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Parameter Passing Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    parameter_passing_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
