"""
Geometric Motion Primitives Example

Demonstrates arc, circle, and spiral trajectory generation for
complex motion patterns.

Usage:
    python 02_geometric_primitives.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def geometric_primitives_example(config_file: str) -> None:
    """
    Demonstrate geometric motion primitives.

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

        # execute_trajectory() applies its first waypoint as a single-cycle
        # correction from the robot's actual current pose, so every shape
        # below is preceded by move_cartesian_trajectory() to travel there
        # first instead of jumping straight to a hardcoded absolute pose.
        cycles_per_step = max(1, round(0.02 / api.cycle_time))

        center = {"X": 100, "Y": 0, "Z": 500}

        # ==================================================
        # Example 1: Circular Arc
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 1: Circular Arc (90 degrees)")
        logging.info("=" * 60)

        arc = api.motion.generate_arc(
            center=center,
            radius=50.0,  # mm
            start_angle=0,  # degrees
            end_angle=90,  # degrees
            steps=50,
            plane='XY'
        )

        logging.info(f"Generated arc with {len(arc)} waypoints")
        logging.info(f"Start point: {arc[0]}")
        logging.info(f"End point: {arc[-1]}")

        logging.info("Moving to arc start position...")
        api.motion.move_cartesian_trajectory(arc[0])

        logging.info("Executing arc motion...")
        api.motion.execute_trajectory(arc, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Arc complete")

        # ==================================================
        # Example 2: Full Circle
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 2: Full Circle (360 degrees)")
        logging.info("=" * 60)

        circle = api.motion.generate_circle(
            center=center,
            radius=30.0,  # mm
            steps=100,
            plane='XY'
        )

        logging.info(f"Generated circle with {len(circle)} waypoints")
        logging.info(f"Radius: 30.0 mm")
        logging.info(f"Circumference: ~{2 * 3.14159 * 30.0:.2f} mm")

        logging.info("Moving to circle start position...")
        api.motion.move_cartesian_trajectory(circle[0])

        logging.info("Executing circular motion...")
        api.motion.execute_trajectory(circle, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Circle complete")

        # ==================================================
        # Example 3: Expanding Spiral (Drilling Pattern)
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 3: Expanding Spiral (Drilling Pattern)")
        logging.info("=" * 60)

        spiral_expand = api.motion.generate_spiral(
            center={"X": 100, "Y": 0, "Z": 500},
            start_radius=5.0,  # Start at 5mm
            end_radius=40.0,  # End at 40mm
            pitch=-10.0,  # Descend 10mm per revolution (generate_spiral does
                          # point[axis] += pitch_offset, so a NEGATIVE pitch
                          # descends and a positive pitch climbs)
            revolutions=3.0,  # 3 complete turns
            steps=150,
            plane='XY',
            axis='Z'
        )

        logging.info(f"Generated expanding spiral:")
        logging.info(f"  Waypoints: {len(spiral_expand)}")
        logging.info(f"  Start radius: 5.0 mm")
        logging.info(f"  End radius: 40.0 mm")
        logging.info(f"  Pitch: -10.0 mm/rev (descending)")
        logging.info(f"  Revolutions: 3.0")
        logging.info(f"  Total Z travel: -30.0 mm (descending, Z: 500 -> 470)")

        logging.info("Moving to spiral start position...")
        api.motion.move_cartesian_trajectory(spiral_expand[0])

        logging.info("Executing expanding spiral...")
        api.motion.execute_trajectory(spiral_expand, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Expanding spiral complete")

        # ==================================================
        # Example 4: Contracting Spiral (Retraction Pattern)
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 4: Contracting Spiral (Retraction Pattern)")
        logging.info("=" * 60)

        spiral_contract = api.motion.generate_spiral(
            center={"X": 100, "Y": 0, "Z": 470},  # Start at bottom (matches
                                                    # the expanding spiral's
                                                    # actual end height above)
            start_radius=40.0,  # Start wide
            end_radius=5.0,  # End narrow
            pitch=10.0,  # Ascend 10mm per revolution (generate_spiral does
                         # point[axis] += pitch_offset, so a POSITIVE pitch
                         # climbs here)
            revolutions=3.0,
            steps=150,
            plane='XY',
            axis='Z'
        )

        logging.info(f"Generated contracting spiral:")
        logging.info(f"  Waypoints: {len(spiral_contract)}")
        logging.info(f"  Start radius: 40.0 mm")
        logging.info(f"  End radius: 5.0 mm")
        logging.info(f"  Pitch: 10.0 mm/rev (ascending)")
        logging.info(f"  Revolutions: 3.0")
        logging.info(f"  Total Z travel: 30.0 mm (upward, Z: 470 -> 500)")

        logging.info("Moving to spiral start position...")
        api.motion.move_cartesian_trajectory(spiral_contract[0])

        logging.info("Executing contracting spiral...")
        api.motion.execute_trajectory(spiral_contract, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Contracting spiral complete")

        # ==================================================
        # Example 5: Different Planes
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 5: Circles in Different Planes")
        logging.info("=" * 60)

        # Circle in XZ plane (vertical)
        circle_xz = api.motion.generate_circle(
            center={"X": 100, "Y": 0, "Z": 500},
            radius=25.0,
            steps=80,
            plane='XZ'
        )

        logging.info("Circle in XZ plane (vertical):")
        logging.info(f"  Waypoints: {len(circle_xz)}")
        logging.info(f"  Plane: XZ (vertical circle)")

        logging.info("Moving to XZ circle start position...")
        api.motion.move_cartesian_trajectory(circle_xz[0])

        logging.info("Executing XZ circle...")
        api.motion.execute_trajectory(circle_xz, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ XZ circle complete")

        # ==================================================
        # Application Examples
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Application Examples")
        logging.info("=" * 60)

        logging.info("\nDrilling/Milling Applications:")
        logging.info("  - Expanding spiral: Drill out large holes, pocket milling")
        logging.info("  - Contracting spiral: Retracting from deep holes")
        logging.info("  - Circular: Bore existing holes, circular pockets")

        logging.info("\nAssembly Applications:")
        logging.info("  - Circular: Screw driving, peg insertion with clearance")
        logging.info("  - Arc: Curved insertion paths, avoiding obstacles")

        logging.info("\nInspection Applications:")
        logging.info("  - Circle: Scanning circular features")
        logging.info("  - Spiral: Scanning large areas with overlap")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during geometric primitives: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Geometric Primitives Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Geometric Motion Primitives Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    geometric_primitives_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
