"""
State Machine Coordination Example

Demonstrates a multi-state coordination workflow between Python and KRL.
Works with templates/krl/state_machine.src

States:
  0: IDLE - Waiting to start
  1: CALIBRATING - Python performing calibration
  2: READY - Calibration complete, ready for motion
  3: EXECUTING - Robot executing motion task
  4: COMPLETE - Task finished
  9: ERROR - Error condition

Usage:
    python 03_state_machine.py --config RSI_EthernetConfig.xml
"""

import argparse
import time
import logging
from enum import IntEnum
from RSIPI import RSIAPI

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


class State(IntEnum):
    """State machine states matching KRL program."""
    IDLE = 0
    CALIBRATING = 1
    READY = 2
    EXECUTING = 3
    COMPLETE = 4
    ERROR = 9


def perform_calibration() -> tuple[float, float, float]:
    """
    Perform calibration routine.

    In a real application, this would:
    - Read sensor data
    - Analyze calibration target
    - Calculate position offsets
    - Validate results

    Returns:
        Tuple of (offset_x, offset_y, offset_z) in mm
    """
    logging.info("Performing calibration routine...")

    # Simulate calibration processing
    time.sleep(2.0)

    # Example calibration results (in real app, calculated from sensors)
    offset_x = 5.0
    offset_y = -2.0
    offset_z = 0.5

    logging.info(f"Calibration complete:")
    logging.info(f"  Offset X: {offset_x:+.2f} mm")
    logging.info(f"  Offset Y: {offset_y:+.2f} mm")
    logging.info(f"  Offset Z: {offset_z:+.2f} mm")

    return (offset_x, offset_y, offset_z)


def state_machine_example(config_file: str) -> None:
    """
    Execute state machine coordination with KRL program.

    Monitors KRL state transitions and responds appropriately
    to each state change.

    Args:
        config_file: Path to RSI configuration XML file
    """
    api = RSIAPI(config_file)

    try:
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        # Main state monitoring loop
        logging.info("Monitoring state machine...")
        last_state = None
        calibration_offsets = (0.0, 0.0, 0.0)

        while True:
            # Read current state from KRL
            current_state = int(api.krl.read_param('T11'))

            # Only log state changes
            if current_state != last_state:
                logging.info(f"State changed: {State(last_state or 0).name} → {State(current_state).name}")
                last_state = current_state

            # Handle each state
            if current_state == State.IDLE:
                # Waiting for KRL to start
                logging.debug("Waiting in IDLE state...")
                time.sleep(0.5)

            elif current_state == State.CALIBRATING:
                logging.info("✅ State: CALIBRATING - Starting calibration")

                # Perform calibration
                calibration_offsets = perform_calibration()

                # Write calibration results to Tech.C for KRL
                logging.info("Writing calibration offsets to KRL...")
                api.krl.write_param('C12', calibration_offsets[0])  # X offset
                api.krl.write_param('C13', calibration_offsets[1])  # Y offset
                api.krl.write_param('C14', calibration_offsets[2])  # Z offset

                # Signal calibration complete
                api.krl.signal_complete(1)
                logging.info("✅ Calibration complete, signaled KRL")

            elif current_state == State.READY:
                logging.info("✅ State: READY - System ready for motion")
                # KRL is ready to execute, no action needed from Python
                time.sleep(0.1)

            elif current_state == State.EXECUTING:
                logging.info("✅ State: EXECUTING - Robot in motion")

                # Monitor execution (could send real-time corrections here)
                # Example: Send RSI corrections based on sensor feedback
                # api.motion.update_cartesian(X=calibration_offsets[0])

                # For this example, just monitor
                time.sleep(0.1)

            elif current_state == State.COMPLETE:
                logging.info("✅ State: COMPLETE - Task finished successfully!")
                logging.info("State machine workflow complete")
                break  # Exit loop, task is done

            elif current_state == State.ERROR:
                logging.error("❌ State: ERROR - Error detected in KRL program!")
                logging.error("Aborting state machine workflow")
                break  # Exit loop, error occurred

            else:
                logging.warning(f"⚠️  Unknown state: {current_state}")
                time.sleep(0.1)

            # Prevent tight loop
            time.sleep(0.05)  # Check state every 50ms

        logging.info("State machine monitoring ended")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")
        # Signal error to KRL
        try:
            api.io.set_output(2, True)  # Error signal
            logging.info("Signaled error to KRL")
        except:
            pass

    except Exception as e:
        logging.error(f"❌ Error during state machine: {e}")
        # Signal error to KRL
        try:
            api.io.set_output(2, True)
        except:
            pass

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='State Machine Coordination Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - State Machine Coordination Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    state_machine_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    main()
