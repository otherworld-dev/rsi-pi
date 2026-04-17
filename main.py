"""
RSIPI Test Runner
=================
Uncomment one example at a time and run this file.
Uses RSI_EthernetConfig.xml from the project root by default.

Usage:
    python main.py
    python main.py --config path/to/other_config.xml
"""

import argparse
import time
from multiprocessing import freeze_support
from RSIPI import RSIAPI

if __name__ == '__main__':
    freeze_support()

    parser = argparse.ArgumentParser(description="RSIPI Test Runner")
    parser.add_argument("--config", type=str, default="RSI_EthernetConfig.xml",
                        help="Path to RSI config XML file")
    args = parser.parse_args()

    api = RSIAPI(args.config)

    # =========================================================================
    # Example 01: Start / Stop
    # =========================================================================
    # api.start()
    # print("RSI started. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 02: Send Cartesian correction (move TCP 50mm along X)
    # =========================================================================
    api.start()
    print("Waiting for robot connection...")
    while not api.is_running():
        time.sleep(0.1)
    # Wait for first IPOC to confirm packets are flowing
    while api.monitoring.get_ipoc() == "N/A" or api.monitoring.get_ipoc() == 0:
        time.sleep(0.1)
    print(f"Connected. IPOC: {api.monitoring.get_ipoc()}")
    api.motion.update_cartesian(X=0.01, Y=0, Z=0)
    print("Sent Cartesian correction. Press Enter to stop.")
    input()
    api.stop()

    # =========================================================================
    # Example 03: Send Joint correction (move A1 by 10 degrees)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # api.motion.update_joints(A1=10)
    # print("Sent joint correction. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 04: External axes (move E1 by 100mm)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # api.motion.move_external_axis('E1', 100)
    # print("Sent external axis correction. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 05: Digital I/O (toggle outputs)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # api.io.set_output(1, True)
    # api.io.set_output(2, False)
    # api.io.set_output(3, True)
    # print("Set digital outputs. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 06: CSV Logging
    # =========================================================================
    # api.logging.start()
    # api.start()
    # print("Logging to CSV. Press Enter to stop.")
    # input()
    # api.logging.stop()
    # api.stop()

    # =========================================================================
    # Example 07: Live Graphing
    # =========================================================================
    # api.start()
    # api.viz.start_live_plot('3d')
    # print("Live graph running. Press Enter to stop.")
    # input()
    # api.viz.stop_live_plot()
    # api.stop()

    # =========================================================================
    # Example 08: Safety Limits (restrict X-axis to +/-500mm)
    # =========================================================================
    # api.safety.set_limit("RKorr.X", -500, 500)
    # api.start()
    # print("Safety limits active. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 09: Cartesian Trajectory (square pattern)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # waypoints = [
    #     {"X": 50, "Y": 0, "Z": 0},
    #     {"X": 50, "Y": 50, "Z": 0},
    #     {"X": 0, "Y": 50, "Z": 0},
    #     {"X": 0, "Y": 0, "Z": 0},
    # ]
    # for wp in waypoints:
    #     api.motion.update_cartesian(**wp)
    #     time.sleep(0.5)
    # print("Trajectory complete. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 10: Safe Shutdown (Ctrl+C handler)
    # =========================================================================
    # try:
    #     api.start()
    #     print("Running. Press Ctrl+C for safe shutdown.")
    #     while True:
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     print("Emergency stop triggered.")
    #     api.safety.stop()
    #     api.stop()

    # =========================================================================
    # Coordination 01: Basic Handshake (KRL <-> Python I/O signalling)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # print("Waiting for KRL ready signal on input 1...")
    # if api.krl.wait_for_signal(1, timeout=10.0):
    #     print("KRL ready. Processing...")
    #     time.sleep(1)
    #     api.krl.signal_complete(1)
    #     print("Handshake complete.")
    # else:
    #     print("Timeout waiting for KRL signal.")
    # input("Press Enter to stop.")
    # api.stop()

    # =========================================================================
    # Coordination 02: Parameter Passing (read/write Tech variables)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # pos_x = api.krl.read_param('T11')
    # pos_y = api.krl.read_param('T12')
    # pos_z = api.krl.read_param('T13')
    # print(f"Current position from KRL: X={pos_x}, Y={pos_y}, Z={pos_z}")
    # api.krl.write_param('C11', pos_x + 50)
    # api.krl.write_param('C12', pos_y)
    # api.krl.write_param('C13', pos_z)
    # api.krl.signal_complete(1)
    # print("Parameters sent. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Coordination 03: State Machine (multi-state workflow)
    # =========================================================================
    # IDLE, CALIBRATING, READY, EXECUTING, COMPLETE, ERROR = 0, 1, 2, 3, 4, 5
    # api.start()
    # time.sleep(1)
    # state = IDLE
    # print("State machine running. Press Ctrl+C to exit.")
    # try:
    #     while state != COMPLETE:
    #         krl_state = int(api.krl.read_param('T11'))
    #         if krl_state == CALIBRATING:
    #             print("Calibrating...")
    #             time.sleep(2)
    #             api.krl.write_param('C11', READY)
    #             state = READY
    #         elif krl_state == EXECUTING:
    #             print("Executing motion...")
    #             state = EXECUTING
    #         elif krl_state == COMPLETE:
    #             print("Complete.")
    #             state = COMPLETE
    #         elif krl_state == ERROR:
    #             print("Error detected!")
    #             break
    #         time.sleep(0.05)
    # except KeyboardInterrupt:
    #     pass
    # api.stop()

    # =========================================================================
    # Advanced Motion 01: Velocity Profiles (trapezoidal vs S-curve)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # traj = api.motion.generate_trajectory(
    #     {"X": 0, "Y": 0, "Z": 0},
    #     {"X": 100, "Y": 0, "Z": 0},
    #     steps=50)
    # trap_profile = api.motion.generate_velocity_profile(
    #     traj, max_velocity=50, max_acceleration=100, profile='trapezoidal')
    # scurve_profile = api.motion.generate_velocity_profile(
    #     traj, max_velocity=50, max_acceleration=100, profile='s-curve')
    # print(f"Trapezoidal: {len(trap_profile)} points")
    # print(f"S-curve: {len(scurve_profile)} points")
    # print("Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Advanced Motion 02: Geometric Primitives (arc, circle, spiral)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # arc = api.motion.generate_arc(
    #     center={"X": 0, "Y": 0, "Z": 0},
    #     radius=50, start_angle=0, end_angle=90, steps=20)
    # circle = api.motion.generate_circle(
    #     center={"X": 0, "Y": 0, "Z": 0},
    #     radius=50, steps=36)
    # spiral = api.motion.generate_spiral(
    #     center={"X": 0, "Y": 0, "Z": 0},
    #     start_radius=10, end_radius=50, pitch=5.0, revolutions=3, steps=60)
    # print(f"Arc: {len(arc)} pts, Circle: {len(circle)} pts, Spiral: {len(spiral)} pts")
    # print("Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Advanced Motion 03: Path Blending (smooth corner transitions)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # traj1 = [{"X": i, "Y": 0, "Z": 0} for i in range(0, 50, 5)]
    # traj2 = [{"X": 50, "Y": i, "Z": 0} for i in range(0, 50, 5)]
    # blended = api.motion.blend_trajectories(traj1, traj2, blend_radius=15)
    # print(f"Blended trajectory: {len(blended)} points")
    # print("Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Advanced Motion 04: Coordinate Transforms (frame conversions)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # base_pos = {"X": 500, "Y": 100, "Z": 800, "A": 0, "B": 0, "C": 0}
    # world_pos = api.motion.transform_coordinates(
    #     base_pos, from_frame="BASE", to_frame="WORLD",
    #     frame_offset={"X": 100, "Y": 50, "Z": 0})
    # print(f"Base: {base_pos}")
    # print(f"World: {world_pos}")
    # print("Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Advanced Motion 05: Combined Motion (production drilling pattern)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # # Generate trajectory to work area
    # nav_traj = api.motion.generate_trajectory(
    #     {"X": 0, "Y": 0, "Z": 0},
    #     {"X": 200, "Y": 200, "Z": 0},
    #     steps=30)
    # # Apply S-curve velocity profile
    # profiled = api.motion.generate_velocity_profile(
    #     nav_traj, max_velocity=80, max_acceleration=200, profile='s-curve')
    # print(f"Navigation: {len(profiled)} profiled points")
    # # Spiral inspection pattern
    # inspection = api.motion.generate_spiral(
    #     center={"X": 200, "Y": 200, "Z": 0},
    #     start_radius=5, end_radius=40, pitch=0, revolutions=2, steps=40)
    # # Drilling descent spiral
    # drill = api.motion.generate_spiral(
    #     center={"X": 200, "Y": 200, "Z": 0},
    #     start_radius=3, end_radius=3, pitch=-5.0, revolutions=5, steps=50)
    # print(f"Inspection: {len(inspection)} pts, Drill: {len(drill)} pts")
    # print("Press Enter to stop.")
    # input()
    # api.stop()
