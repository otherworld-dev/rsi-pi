"""
Parameter Passing Example

Demonstrates bidirectional numerical data exchange between Python and KRL
using RSI Tech variables. Works with templates/krl/parameter_passing.src

Flow:
1. KRL writes current position to Tech.T11-T16
2. Python waits for data ready signal
3. Python reads position from Tech.T
4. Python calculates target and writes to Tech.C11-C13
5. Python signals completion
6. KRL reads target from Tech.C and executes motion

Usage:
    python 02_parameter_passing.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

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

        # Wait for KRL to signal that data is ready
        logging.info("Waiting for KRL data ready signal...")

        if api.krl.wait_for_signal(1, timeout=30.0):
            logging.info("✅ KRL signaled data ready!")

            # Read current position from Tech.T variables
            logging.info("Reading current position from KRL...")
            current_x = api.krl.read_param('T11')
            current_y = api.krl.read_param('T12')
            current_z = api.krl.read_param('T13')
            current_a = api.krl.read_param('T14')
            current_b = api.krl.read_param('T15')
            current_c = api.krl.read_param('T16')

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

            # Write target position to Tech.C variables for KRL to read
            logging.info("Writing target position to KRL...")
            api.krl.write_param('C11', target_x)
            api.krl.write_param('C12', target_y)
            api.krl.write_param('C13', target_z)
            logging.info("✅ Target position written to Tech.C")

            # Signal KRL that calculation is complete
            api.krl.signal_complete(1)
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
