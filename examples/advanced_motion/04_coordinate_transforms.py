"""
Coordinate Frame Transformation Example

Demonstrates transformation of positions and trajectories between different
coordinate frames (BASE, TOOL, WORLD, ROBROOT).

Usage:
    python 04_coordinate_transforms.py --config RSI_EthernetConfig.xml
"""

import argparse
import logging
from RSIPI import RSIAPI

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def coordinate_transform_example(config_file: str) -> None:
    """
    Demonstrate coordinate frame transformations.

    Args:
        config_file: Path to RSI configuration XML file
    """
    api = RSIAPI(config_file)

    try:
        logging.info("Starting RSI communication...")
        api.start()
        logging.info("✅ RSI started successfully")

        # ==================================================
        # Example 1: BASE to WORLD Transformation
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 1: BASE to WORLD Transformation")
        logging.info("=" * 60)

        # Define BASE coordinate system offset
        base_offset = {
            "X": 500.0,  # Base is 500mm offset in X
            "Y": 200.0,  # 200mm offset in Y
            "Z": 0.0,
            "A": 0.0,
            "B": 0.0,
            "C": 45.0    # Base rotated 45° around Z
        }

        # Position in BASE coordinates
        pose_base = {"X": 100, "Y": 50, "Z": 500, "A": 0, "B": 0, "C": 0}

        logging.info("BASE coordinate system offset:")
        logging.info(f"  Translation: X={base_offset['X']}, Y={base_offset['Y']}, Z={base_offset['Z']}")
        logging.info(f"  Rotation: A={base_offset['A']}, B={base_offset['B']}, C={base_offset['C']}")

        logging.info(f"\nPosition in BASE frame:")
        logging.info(f"  X={pose_base['X']}, Y={pose_base['Y']}, Z={pose_base['Z']}")
        logging.info(f"  A={pose_base['A']}, B={pose_base['B']}, C={pose_base['C']}")

        # Transform to WORLD coordinates
        pose_world = api.motion.transform_coordinates(
            pose_base,
            from_frame='BASE',
            to_frame='WORLD',
            frame_offset=base_offset
        )

        logging.info(f"\nPosition in WORLD frame:")
        logging.info(f"  X={pose_world['X']}, Y={pose_world['Y']}, Z={pose_world['Z']}")
        logging.info(f"  A={pose_world['A']}, B={pose_world['B']}, C={pose_world['C']}")

        # ==================================================
        # Example 2: TOOL Frame Offset
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 2: TOOL Frame Transformation")
        logging.info("=" * 60)

        # Define TOOL coordinate system (e.g., gripper with offset)
        tool_offset = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 150.0,  # Tool extends 150mm in Z
            "A": 0.0,
            "B": 0.0,
            "C": 0.0
        }

        # Position expressed at tool center point (TCP)
        pose_tcp = {"X": 200, "Y": 100, "Z": 400}

        logging.info("TOOL offset from flange:")
        logging.info(f"  Z offset: {tool_offset['Z']} mm")

        logging.info(f"\nPosition at TCP:")
        logging.info(f"  X={pose_tcp['X']}, Y={pose_tcp['Y']}, Z={pose_tcp['Z']}")

        # Transform to flange coordinates
        pose_flange = api.motion.transform_coordinates(
            pose_tcp,
            from_frame='TOOL',
            to_frame='BASE',
            frame_offset=tool_offset
        )

        logging.info(f"\nPosition at flange:")
        logging.info(f"  X={pose_flange['X']}, Y={pose_flange['Y']}, Z={pose_flange['Z']}")
        logging.info(f"  Note: Z decreased by {tool_offset['Z']}mm (tool length)")

        # ==================================================
        # Example 3: Transforming Entire Trajectories
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 3: Transforming Trajectories Between Frames")
        logging.info("=" * 60)

        # Generate a circular trajectory in TOOL frame
        circle_tcp = api.motion.generate_circle(
            center={"X": 0, "Y": 0, "Z": 50},  # Circle around TCP
            radius=20.0,
            steps=50,
            plane='XY'
        )

        logging.info(f"Generated circle in TOOL frame:")
        logging.info(f"  Center: X=0, Y=0, Z=50 (relative to TCP)")
        logging.info(f"  Radius: 20mm")
        logging.info(f"  Waypoints: {len(circle_tcp)}")

        # Transform entire trajectory to BASE frame
        circle_base = []
        for waypoint in circle_tcp:
            transformed = api.motion.transform_coordinates(
                waypoint,
                from_frame='TOOL',
                to_frame='BASE',
                frame_offset=tool_offset
            )
            circle_base.append(transformed)

        logging.info(f"\nTransformed to BASE frame:")
        logging.info(f"  First waypoint: X={circle_base[0]['X']:.2f}, Y={circle_base[0]['Y']:.2f}, Z={circle_base[0]['Z']:.2f}")
        logging.info(f"  Circle now expressed relative to flange")

        # ==================================================
        # Example 4: Work Object Offset
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 4: Work Object (Pallet) Transformation")
        logging.info("=" * 60)

        # Define work object position (e.g., pallet location)
        pallet_offset = {
            "X": 800.0,
            "Y": -300.0,
            "Z": 50.0,   # Pallet height
            "A": 0.0,
            "B": 0.0,
            "C": 30.0    # Pallet rotated 30° for better access
        }

        # Define pick points relative to pallet corner (work object frame)
        pick_points_pallet = [
            {"X": 50, "Y": 50, "Z": 20},
            {"X": 150, "Y": 50, "Z": 20},
            {"X": 50, "Y": 150, "Z": 20},
            {"X": 150, "Y": 150, "Z": 20}
        ]

        logging.info("Pallet location in BASE frame:")
        logging.info(f"  X={pallet_offset['X']}, Y={pallet_offset['Y']}, Z={pallet_offset['Z']}")
        logging.info(f"  Rotation: C={pallet_offset['C']}°")

        logging.info(f"\nPick points defined relative to pallet:")
        for i, point in enumerate(pick_points_pallet, 1):
            logging.info(f"  Point {i}: X={point['X']}, Y={point['Y']}, Z={point['Z']}")

        # Transform pick points to robot BASE frame
        pick_points_base = []
        for point in pick_points_pallet:
            transformed = api.motion.transform_coordinates(
                point,
                from_frame='WORK',
                to_frame='BASE',
                frame_offset=pallet_offset
            )
            pick_points_base.append(transformed)

        logging.info(f"\nPick points in robot BASE frame:")
        for i, point in enumerate(pick_points_base, 1):
            logging.info(f"  Point {i}: X={point['X']:.2f}, Y={point['Y']:.2f}, Z={point['Z']:.2f}")

        logging.info("\nAdvantage: Pallet can be moved/rotated by updating offset only")

        # ==================================================
        # Example 5: Practical Application - Sensor-Guided Motion
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Example 5: Sensor-Guided Motion with Frame Transforms")
        logging.info("=" * 60)

        # Simulated sensor detects part offset
        sensor_offset = {
            "X": 5.2,    # Part detected 5.2mm offset in X
            "Y": -2.1,   # 2.1mm offset in Y
            "Z": 0.0,
            "A": 0.0,
            "B": 0.0,
            "C": 1.5     # Part rotated 1.5° from expected
        }

        logging.info("Sensor detected part offset:")
        logging.info(f"  ΔX = {sensor_offset['X']:+.1f} mm")
        logging.info(f"  ΔY = {sensor_offset['Y']:+.1f} mm")
        logging.info(f"  ΔC = {sensor_offset['C']:+.1f}°")

        # Nominal pick position (taught position)
        nominal_pick = {"X": 300, "Y": 200, "Z": 100, "A": 0, "B": 0, "C": 0}

        logging.info(f"\nNominal pick position:")
        logging.info(f"  X={nominal_pick['X']}, Y={nominal_pick['Y']}, Z={nominal_pick['Z']}")

        # Apply sensor correction
        corrected_pick = api.motion.transform_coordinates(
            nominal_pick,
            from_frame='BASE',
            to_frame='BASE',  # Same frame, just applying offset
            frame_offset=sensor_offset
        )

        logging.info(f"\nCorrected pick position:")
        logging.info(f"  X={corrected_pick['X']:.1f}, Y={corrected_pick['Y']:.1f}, Z={corrected_pick['Z']:.1f}")
        logging.info(f"  C={corrected_pick['C']:.1f}°")

        logging.info("\nRobot will pick from corrected position based on sensor feedback")

        # ==================================================
        # Application Examples
        # ==================================================
        logging.info("\n" + "=" * 60)
        logging.info("Application Examples")
        logging.info("=" * 60)

        logging.info("\nMultiple Work Objects:")
        logging.info("  - Define multiple pallet/fixture locations")
        logging.info("  - Teach trajectories once relative to work object")
        logging.info("  - Execute on any pallet by changing offset")

        logging.info("\nTool Changes:")
        logging.info("  - Different tools have different TCP offsets")
        logging.info("  - Transform taught positions for new tool")
        logging.info("  - No need to reteach all positions")

        logging.info("\nVision/Sensor Integration:")
        logging.info("  - Sensor detects part position/orientation")
        logging.info("  - Apply correction transform to taught path")
        logging.info("  - Robot adapts to part variations")

        logging.info("\nMulti-Robot Cells:")
        logging.info("  - Each robot has its own BASE frame")
        logging.info("  - Transform positions to shared WORLD frame")
        logging.info("  - Coordinate motion between robots")

    except KeyboardInterrupt:
        logging.warning("\n⚠️  Interrupted by user")

    except Exception as e:
        logging.error(f"❌ Error during coordinate transforms: {e}")

    finally:
        logging.info("Stopping RSI communication...")
        api.stop()
        logging.info("✅ API stopped successfully")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Coordinate Transform Example')
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI configuration file'
    )

    args = parser.parse_args()

    logging.info("=" * 60)
    logging.info("RSIPI - Coordinate Frame Transformation Example")
    logging.info("=" * 60)
    logging.info(f"Config: {args.config}")
    logging.info("=" * 60)

    coordinate_transform_example(args.config)

    logging.info("=" * 60)
    logging.info("Example complete!")
    logging.info("=" * 60)


if __name__ == '__main__':
    main()
