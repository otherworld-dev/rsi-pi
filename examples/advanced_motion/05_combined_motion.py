"""
Combined Advanced Motion Example

Demonstrates combining multiple Phase 4 features to create a complete,
production-ready motion application with velocity profiling, geometric
primitives, path blending, and coordinate transformations.

Application: Automated drilling and inspection pattern

Usage:
    python 05_combined_motion.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def combined_motion_example(config_file: str) -> None:
    """
    Execute a complete motion application combining all Phase 4 features.

    Scenario: Automated drilling and inspection of a workpiece
    - Navigate to inspection position with smooth blending
    - Inspect with spiral pattern
    - Navigate to drilling position
    - Execute drilling pattern with optimized velocity
    - Return to home position

    Args:
        config_file: Path to RSI configuration XML file
    """
    api = RSIAPI(config_file)

    try:
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        # ==================================================
        # Setup: Define Work Object and Tool
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Application Setup")
        logging.info("=" * 60)

        # Work object offset (pallet position)
        workpiece_offset = {
            "X": 500.0,
            "Y": -200.0,
            "Z": 50.0,
            "A": 0.0,
            "B": 0.0,
            "C": 15.0  # Pallet at slight angle
        }

        # Tool offset (inspection camera + drill)
        tool_offset = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 120.0,  # Tool length
            "A": 0.0,
            "B": 0.0,
            "C": 0.0
        }

        logging.info("Work object configuration:")
        logging.info(f"  Position: X={workpiece_offset['X']}, Y={workpiece_offset['Y']}, Z={workpiece_offset['Z']}")
        logging.info(f"  Rotation: C={workpiece_offset['C']}°")

        logging.info(f"\nTool configuration:")
        logging.info(f"  TCP offset: Z={tool_offset['Z']} mm")

        # ==================================================
        # Step 1: Navigate to Inspection Position
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Step 1: Navigate to Inspection Position")
        logging.info("=" * 60)

        # Define waypoints in work object frame
        home_work = {"X": 0, "Y": 0, "Z": 200}  # Safe height above workpiece
        approach_work = {"X": 100, "Y": 100, "Z": 100}  # Approach position
        inspect_work = {"X": 100, "Y": 100, "Z": 30}  # Inspection height

        # Transform to BASE coordinates
        home_base = api.motion.transform_coordinates(
            home_work, 'WORK', 'BASE', workpiece_offset
        )
        approach_base = api.motion.transform_coordinates(
            approach_work, 'WORK', 'BASE', workpiece_offset
        )
        inspect_base = api.motion.transform_coordinates(
            inspect_work, 'WORK', 'BASE', workpiece_offset
        )

        logging.info("Navigation waypoints (work frame):")
        logging.info(f"  Home: {home_work}")
        logging.info(f"  Approach: {approach_work}")
        logging.info(f"  Inspect: {inspect_work}")

        # Generate navigation segments
        seg1 = api.motion.generate_trajectory(home_base, approach_base, steps=40)
        seg2 = api.motion.generate_trajectory(approach_base, inspect_base, steps=30)

        # Blend for smooth motion
        navigation = api.motion.blend_trajectories(
            seg1, seg2,
            blend_radius=25.0,
            blend_steps=15
        )

        # Apply velocity profile for fast navigation
        navigation_profiled = api.motion.generate_velocity_profile(
            navigation,
            max_velocity=300.0,  # Fast navigation
            max_acceleration=800.0,
            profile='s-curve'  # Smooth acceleration
        )

        logging.info(f"\nNavigation trajectory:")
        logging.info(f"  Waypoints: {len(navigation)}")
        logging.info(f"  Velocity profile: S-curve (smooth)")
        logging.info(f"  Max velocity: 300 mm/s")

        logging.info("Executing navigation...")
        api.motion.execute_profiled_trajectory(navigation_profiled, space="cartesian")
        logging.info("✅ Reached inspection position")

        # ==================================================
        # Step 2: Execute Spiral Inspection Pattern
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Step 2: Execute Spiral Inspection Pattern")
        logging.info("=" * 60)

        # Generate spiral inspection pattern in work frame
        spiral_work = api.motion.generate_spiral(
            center={"X": 100, "Y": 100, "Z": 30},
            start_radius=5.0,
            end_radius=25.0,
            pitch=0.0,  # Stay at constant Z (no vertical motion)
            revolutions=2.5,
            steps=120,
            plane='XY'
        )

        # Transform spiral to BASE frame
        spiral_base = []
        for waypoint in spiral_work:
            transformed = api.motion.transform_coordinates(
                waypoint, 'WORK', 'BASE', workpiece_offset
            )
            spiral_base.append(transformed)

        # Apply slower velocity for inspection
        spiral_profiled = api.motion.generate_velocity_profile(
            spiral_base,
            max_velocity=50.0,  # Slow for inspection
            max_acceleration=200.0,
            profile='s-curve'
        )

        logging.info("Inspection pattern:")
        logging.info(f"  Type: Expanding spiral")
        logging.info(f"  Radius: 5mm → 25mm")
        logging.info(f"  Revolutions: 2.5")
        logging.info(f"  Velocity: 50 mm/s (inspection speed)")
        logging.info(f"  Waypoints: {len(spiral_base)}")

        logging.info("Executing inspection spiral...")
        api.motion.execute_profiled_trajectory(spiral_profiled, space="cartesian")
        # In real application: capture camera frame here
        logging.info("✅ Inspection complete")

        # ==================================================
        # Step 3: Navigate to Drilling Position
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Step 3: Navigate to Drilling Position")
        logging.info("=" * 60)

        # Return to safe height
        retract_work = {"X": 100, "Y": 100, "Z": 100}
        drill_approach_work = {"X": 150, "Y": 50, "Z": 100}
        drill_start_work = {"X": 150, "Y": 50, "Z": 35}

        retract_base = api.motion.transform_coordinates(
            retract_work, 'WORK', 'BASE', workpiece_offset
        )
        drill_approach_base = api.motion.transform_coordinates(
            drill_approach_work, 'WORK', 'BASE', workpiece_offset
        )
        drill_start_base = api.motion.transform_coordinates(
            drill_start_work, 'WORK', 'BASE', workpiece_offset
        )

        # Navigate to drilling position with blending
        seg1 = api.motion.generate_trajectory(inspect_base, retract_base, steps=25)
        seg2 = api.motion.generate_trajectory(retract_base, drill_approach_base, steps=40)
        seg3 = api.motion.generate_trajectory(drill_approach_base, drill_start_base, steps=30)

        # Blend all segments
        traj_temp = api.motion.blend_trajectories(seg1, seg2, blend_radius=20.0, blend_steps=12)
        drill_navigation = api.motion.blend_trajectories(traj_temp, seg3, blend_radius=20.0, blend_steps=12)

        # Apply fast velocity profile
        drill_nav_profiled = api.motion.generate_velocity_profile(
            drill_navigation,
            max_velocity=250.0,
            max_acceleration=700.0,
            profile='trapezoidal'  # Fast point-to-point
        )

        logging.info("Navigation to drilling position:")
        logging.info(f"  Waypoints: {len(drill_navigation)}")
        logging.info(f"  Profile: Trapezoidal (fast)")

        logging.info("Executing navigation to drill position...")
        api.motion.execute_profiled_trajectory(drill_nav_profiled, space="cartesian")
        logging.info("✅ Reached drilling position")

        # ==================================================
        # Step 4: Execute Drilling Pattern
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Step 4: Execute Drilling Pattern")
        logging.info("=" * 60)

        # Generate expanding spiral drilling pattern
        drill_spiral_work = api.motion.generate_spiral(
            center={"X": 150, "Y": 50, "Z": 35},
            start_radius=2.0,
            end_radius=15.0,
            pitch=5.0,  # Descend 5mm per revolution
            revolutions=3.0,
            steps=150,
            plane='XY',
            axis='Z'
        )

        # Transform to BASE
        drill_spiral_base = []
        for waypoint in drill_spiral_work:
            transformed = api.motion.transform_coordinates(
                waypoint, 'WORK', 'BASE', workpiece_offset
            )
            drill_spiral_base.append(transformed)

        # Apply drilling velocity profile
        drill_profiled = api.motion.generate_velocity_profile(
            drill_spiral_base,
            max_velocity=30.0,  # Slow drilling speed
            max_acceleration=100.0,
            profile='s-curve'
        )

        logging.info("Drilling pattern:")
        logging.info(f"  Type: Expanding spiral with descent")
        logging.info(f"  Radius: 2mm → 15mm")
        logging.info(f"  Pitch: 5mm/revolution (descending)")
        logging.info(f"  Total depth: 15mm")
        logging.info(f"  Velocity: 30 mm/s (drilling speed)")
        logging.info(f"  Waypoints: {len(drill_spiral_base)}")

        logging.info("Executing drilling spiral...")
        api.motion.execute_profiled_trajectory(drill_profiled, space="cartesian")
        # In real application: control spindle speed, feed rate
        logging.info("✅ Drilling complete")

        # ==================================================
        # Step 5: Return to Home Position
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Step 5: Return to Home Position")
        logging.info("=" * 60)

        # Retract from drilling position
        drill_end_work = {"X": 150, "Y": 50, "Z": 20}  # Bottom of hole
        drill_retract_work = {"X": 150, "Y": 50, "Z": 100}

        drill_end_base = api.motion.transform_coordinates(
            drill_end_work, 'WORK', 'BASE', workpiece_offset
        )
        drill_retract_base = api.motion.transform_coordinates(
            drill_retract_work, 'WORK', 'BASE', workpiece_offset
        )

        # Return path with blending
        seg1 = api.motion.generate_trajectory(drill_end_base, drill_retract_base, steps=30)
        seg2 = api.motion.generate_trajectory(drill_retract_base, home_base, steps=50)

        return_path = api.motion.blend_trajectories(seg1, seg2, blend_radius=30.0, blend_steps=15)

        # Fast return profile
        return_profiled = api.motion.generate_velocity_profile(
            return_path,
            max_velocity=350.0,  # Fast return
            max_acceleration=900.0,
            profile='trapezoidal'
        )

        logging.info("Return to home:")
        logging.info(f"  Waypoints: {len(return_path)}")
        logging.info(f"  Max velocity: 350 mm/s")

        logging.info("Executing return to home...")
        api.motion.execute_profiled_trajectory(return_profiled, space="cartesian")
        logging.info("✅ Returned to home position")

        # ==================================================
        # Summary
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Application Complete - Summary")
        logging.info("=" * 60)

        logging.info("\nPhase 4 Features Used:")
        logging.info("  ✅ Coordinate Transformations (work object & tool)")
        logging.info("  ✅ Path Blending (smooth navigation)")
        logging.info("  ✅ Velocity Profiling (optimized speeds)")
        logging.info("  ✅ Geometric Primitives (spiral patterns)")

        logging.info("\nMotion Segments:")
        logging.info("  1. Navigate to inspection (blended, S-curve, 300mm/s)")
        logging.info("  2. Spiral inspection (S-curve, 50mm/s)")
        logging.info("  3. Navigate to drilling (blended, trapezoidal, 250mm/s)")
        logging.info("  4. Drilling spiral (S-curve, 30mm/s)")
        logging.info("  5. Return home (blended, trapezoidal, 350mm/s)")

        logging.info("\nProduction Benefits:")
        logging.info("  - Optimized cycle time with velocity profiling")
        logging.info("  - Smooth motion reduces mechanical stress")
        logging.info("  - Coordinate transforms enable flexible part placement")
        logging.info("  - Geometric primitives simplify complex patterns")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during combined motion: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Combined Advanced Motion Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Combined Advanced Motion Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)
    logging.info("\nScenario: Automated Drilling and Inspection")
    logging.info("=" * 60)

    combined_motion_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()
    main()
