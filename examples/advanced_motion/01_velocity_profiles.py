"""
Velocity Profile Example

Demonstrates velocity profiling for smooth, time-optimal motion with
configurable acceleration and jerk limits.

Compares trapezoidal (bang-bang) and S-curve (jerk-limited) profiles.

Usage:
    python 01_velocity_profiles.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
from _confirm import confirm  # asks before the robot moves


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def velocity_profile_example(config_file: str) -> None:
    """
    Demonstrate velocity profiling with trapezoidal and S-curve profiles.

    Args:
        config_file: Path to RSI configuration XML file
    """
    api = RSIAPI(config_file)

    try:
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        if not api.wait_for_connection(timeout=10.0):
            raise RuntimeError("Timed out waiting for the robot's first RSI packet")

        # Define waypoints for straight-line motion, anchored to the robot's
        # actual current position. execute_trajectory()'s first waypoint is
        # applied as a single-cycle correction from wherever the robot really
        # is, so starting from a hardcoded pose would command an instant jump.
        p0 = api.motion.get_current_pose()
        p1 = {"X": p0["X"] + 200, "Y": p0["Y"] + 100, "Z": p0["Z"]}

        logging.info("Generating base trajectory...")
        trajectory = api.motion.generate_trajectory(p0, p1, steps=100)
        logging.info(f"Generated {len(trajectory)} waypoints")

        # ==================================================
        # Example 1: Trapezoidal Velocity Profile
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 1: Trapezoidal Velocity Profile")
        logging.info("=" * 60)

        profiled_trap = api.motion.generate_velocity_profile(
            trajectory,
            max_velocity=200.0,  # mm/s
            max_acceleration=500.0,  # mm/s²
            profile='trapezoidal'
        )

        logging.info("Trapezoidal profile generated:")
        logging.info(f"  Total waypoints: {len(profiled_trap)}")

        # Sample velocities at key points
        logging.info("  Sample velocities:")
        for i in [0, 25, 50, 75, 99]:
            _, velocity = profiled_trap[i]
            logging.info(f"    Point {i}: {velocity:.2f} mm/s")

        # ==================================================
        # Example 2: S-Curve Velocity Profile
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 2: S-Curve Velocity Profile (Jerk-Limited)")
        logging.info("=" * 60)

        profiled_scurve = api.motion.generate_velocity_profile(
            trajectory,
            max_velocity=200.0,  # mm/s
            max_acceleration=500.0,  # mm/s²
            profile='s-curve'
        )

        logging.info("S-curve profile generated:")
        logging.info(f"  Total waypoints: {len(profiled_scurve)}")

        logging.info("  Sample velocities:")
        for i in [0, 25, 50, 75, 99]:
            _, velocity = profiled_scurve[i]
            logging.info(f"    Point {i}: {velocity:.2f} mm/s")

        # ==================================================
        # Comparison
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Profile Comparison")
        logging.info("=" * 60)

        # Compare acceleration characteristics
        trap_velocities = [v for _, v in profiled_trap]
        scurve_velocities = [v for _, v in profiled_scurve]

        trap_max = max(trap_velocities)
        scurve_max = max(scurve_velocities)

        logging.info(f"Trapezoidal peak velocity: {trap_max:.2f} mm/s")
        logging.info(f"S-curve peak velocity: {scurve_max:.2f} mm/s")

        logging.info("\nCharacteristics:")
        logging.info("  Trapezoidal:")
        logging.info("    - Sharp velocity transitions (instant acceleration changes)")
        logging.info("    - Faster overall motion time")
        logging.info("    - Higher mechanical stress")
        logging.info("    - Suitable for rigid structures")

        logging.info("  S-Curve:")
        logging.info("    - Smooth velocity transitions (limited jerk)")
        logging.info("    - Slightly longer motion time")
        logging.info("    - Reduced vibration and mechanical stress")
        logging.info("    - Recommended for sensitive applications")

        # ==================================================
        # Example 3: Using Profiled Trajectory
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 3: Executing Profiled Motion")
        logging.info("=" * 60)

        logging.info("Using S-curve profile for smooth motion...")

        logging.info("Executing trajectory with profiled velocities...")
        # execute_profiled_trajectory() resamples each (waypoint, velocity)
        # segment at the robot cycle time, so the computed profile shape is
        # actually followed - execute_trajectory() with a flat cycles_per_step
        # would ignore the velocities and ship a plain constant-rate motion.
        if confirm("Execute the profiled trajectory (profiled scurve)", "The robot WILL move."):
            api.motion.execute_profiled_trajectory(profiled_scurve, space='cartesian')

        logging.info("✅ Profiled motion complete")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during velocity profiling: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Velocity Profile Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Velocity Profile Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    velocity_profile_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
