"""
Path Blending Example

Demonstrates smooth trajectory transitions using cubic interpolation for
eliminating stop-and-go motion at trajectory boundaries.

Usage:
    python 03_path_blending.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def path_blending_example(config_file: str) -> None:
    """
    Demonstrate path blending for smooth trajectory transitions.

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
        # correction from the robot's actual current pose, so pace/timing
        # below is expressed via cycles_per_step (the rate= kwarg is
        # deprecated) and we travel to p0 before running any demo trajectory.
        cycles_per_step = max(1, round(0.02 / api.cycle_time))

        # ==================================================
        # Example 1: Sharp Corner vs Blended Corner
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 1: Sharp Corner vs Blended Corner")
        logging.info("=" * 60)

        # Define three points forming a right angle
        p0 = {"X": 100, "Y": 0, "Z": 500}
        p1 = {"X": 200, "Y": 0, "Z": 500}  # Corner point
        p2 = {"X": 200, "Y": 100, "Z": 500}

        logging.info("Moving to start position p0...")
        api.motion.move_cartesian_trajectory(p0)

        # Generate two separate trajectories
        traj1 = api.motion.generate_trajectory(p0, p1, steps=50)
        traj2 = api.motion.generate_trajectory(p1, p2, steps=50)

        logging.info("\nWithout blending:")
        logging.info("  - Robot will stop at corner point")
        logging.info("  - Visible acceleration/deceleration")
        logging.info("  - Less smooth motion")

        # Execute sharp corner (no blending)
        logging.info("\nExecuting sharp corner motion...")
        api.motion.execute_trajectory(traj1, space='cartesian', cycles_per_step=cycles_per_step)
        api.motion.execute_trajectory(traj2, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Sharp corner complete")

        # Return to start
        api.motion.execute_trajectory(
            api.motion.generate_trajectory(p2, p0, steps=50),
            space='cartesian',
            cycles_per_step=cycles_per_step
        )

        # Now execute with blending
        logging.info("\nWith blending:")
        logging.info("  - Smooth transition through corner")
        logging.info("  - No visible stop at corner point")
        logging.info("  - Continuous velocity")

        blended = api.motion.blend_trajectories(
            traj1,
            traj2,
            blend_radius=20.0,  # Start blending 20mm before corner
            blend_steps=20
        )

        logging.info(f"\nBlended trajectory: {len(blended)} waypoints")
        logging.info(f"Original: {len(traj1) + len(traj2)} waypoints")
        logging.info(f"Blend zone: 20.0 mm radius")

        logging.info("Executing blended corner motion...")
        api.motion.execute_trajectory(blended, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Blended corner complete")

        # ==================================================
        # Example 2: Multiple Blend Zones (Continuous Path)
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 2: Continuous Path with Multiple Blends")
        logging.info("=" * 60)

        # Define waypoints for a square pattern
        waypoints = [
            {"X": 100, "Y": 0, "Z": 500},
            {"X": 150, "Y": 0, "Z": 500},
            {"X": 150, "Y": 50, "Z": 500},
            {"X": 100, "Y": 50, "Z": 500},
            {"X": 100, "Y": 0, "Z": 500}  # Return to start
        ]

        logging.info(f"Square pattern with {len(waypoints)} waypoints")

        # Generate segments
        segments = []
        for i in range(len(waypoints) - 1):
            segment = api.motion.generate_trajectory(
                waypoints[i],
                waypoints[i + 1],
                steps=30
            )
            segments.append(segment)

        logging.info(f"Generated {len(segments)} path segments")

        # Blend all segments together
        logging.info("Blending all corners...")
        blended_path = segments[0]

        for i in range(1, len(segments)):
            blended_path = api.motion.blend_trajectories(
                blended_path,
                segments[i],
                blend_radius=10.0,
                blend_steps=15
            )
            logging.info(f"  Blended corner {i}/{len(segments)-1}")

        logging.info(f"\nFinal blended path: {len(blended_path)} waypoints")
        logging.info("Executing continuous square pattern...")
        api.motion.execute_trajectory(blended_path, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Continuous square complete")

        # ==================================================
        # Example 3: Different Blend Radii
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 3: Effect of Different Blend Radii")
        logging.info("=" * 60)

        # Same trajectories as Example 1
        traj1 = api.motion.generate_trajectory(p0, p1, steps=50)
        traj2 = api.motion.generate_trajectory(p1, p2, steps=50)

        blend_radii = [5.0, 15.0, 30.0]

        for radius in blend_radii:
            logging.info(f"\n--- Blend Radius: {radius} mm ---")

            blended = api.motion.blend_trajectories(
                traj1,
                traj2,
                blend_radius=radius,
                blend_steps=20
            )

            logging.info(f"Blend radius: {radius} mm")
            logging.info(f"Blended waypoints: {len(blended)}")

            if radius < 10:
                logging.info("Effect: Tighter blend, closer to sharp corner")
            elif radius < 20:
                logging.info("Effect: Moderate blend, balanced smoothness")
            else:
                logging.info("Effect: Wide blend, very smooth but cuts corner more")

            logging.info(f"Executing blend with radius={radius}mm...")
            api.motion.execute_trajectory(blended, space='cartesian', cycles_per_step=cycles_per_step)
            logging.info(f"✅ Blend radius {radius}mm complete")

            # Return to start
            api.motion.execute_trajectory(
                api.motion.generate_trajectory(p2, p0, steps=50),
                space='cartesian',
                cycles_per_step=cycles_per_step
            )

        # ==================================================
        # Example 4: Blending with Different Orientations
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 4: Blending Trajectories with Orientation Changes")
        logging.info("=" * 60)

        # Define points with different orientations
        p0_rot = {"X": 100, "Y": 0, "Z": 500, "A": 0, "B": 0, "C": 0}
        p1_rot = {"X": 150, "Y": 0, "Z": 500, "A": 0, "B": 0, "C": 45}
        p2_rot = {"X": 150, "Y": 50, "Z": 500, "A": 0, "B": 0, "C": 90}

        traj1_rot = api.motion.generate_trajectory(p0_rot, p1_rot, steps=50)
        traj2_rot = api.motion.generate_trajectory(p1_rot, p2_rot, steps=50)

        logging.info("Trajectory with orientation change:")
        logging.info(f"  Start: C = 0°")
        logging.info(f"  Corner: C = 45°")
        logging.info(f"  End: C = 90°")

        blended_rot = api.motion.blend_trajectories(
            traj1_rot,
            traj2_rot,
            blend_radius=15.0,
            blend_steps=20
        )

        logging.info(f"\nBlending also smooths orientation transitions")
        logging.info(f"Blended waypoints: {len(blended_rot)}")

        logging.info("Executing blended motion with rotation...")
        api.motion.execute_trajectory(blended_rot, space='cartesian', cycles_per_step=cycles_per_step)
        logging.info("✅ Blended rotation complete")

        # ==================================================
        # Application Examples
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Application Examples")
        logging.info("=" * 60)

        logging.info("\nPick and Place Applications:")
        logging.info("  - Smooth transitions between pick/place points")
        logging.info("  - Reduced cycle time by eliminating stops")
        logging.info("  - Lower mechanical stress on robot")

        logging.info("\nWelding/Gluing Applications:")
        logging.info("  - Continuous bead at corners without stop marks")
        logging.info("  - Consistent material deposition rate")
        logging.info("  - Professional finish quality")

        logging.info("\nMachining Applications:")
        logging.info("  - Smooth tool paths without witness marks")
        logging.info("  - Reduced vibration and tool wear")
        logging.info("  - Better surface finish")

        logging.info("\nPainting/Coating Applications:")
        logging.info("  - Even coating thickness at corners")
        logging.info("  - No overspray from deceleration")
        logging.info("  - Faster throughput")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during path blending: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Path Blending Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Path Blending Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    path_blending_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
