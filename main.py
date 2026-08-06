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
import time  # noqa: F401 - used by commented examples
from multiprocessing import freeze_support
from RSIPI import RSIAPI

if __name__ == '__main__':
    freeze_support()

    parser = argparse.ArgumentParser(description="RSIPI Test Runner")
    parser.add_argument("--config", type=str, default="RSI_EthernetConfig.xml",
                        help="Path to RSI config XML file")
    parser.add_argument("--mode", type=str, default="relative", choices=["absolute", "relative"],
                        help="RSI correction mode (must match KRL program)")
    parser.add_argument("--max-cart-rate", type=float, default=0.5,
                        help="Max Cartesian correction per cycle in mm (0 = no limit)")
    parser.add_argument("--max-joint-rate", type=float, default=0.2,
                        help="Max joint correction per cycle in degrees (0 = no limit)")
    parser.add_argument("--cycle-time", type=float, default=0.004,
                        help="RSI cycle time in seconds (0.004 = 4ms/250Hz, 0.012 = 12ms/83Hz)")
    args = parser.parse_args()

    api = RSIAPI(
        args.config,
        rsi_mode=args.mode,
        max_cartesian_rate=args.max_cart_rate,
        max_joint_rate=args.max_joint_rate,
        cycle_time=args.cycle_time
    )

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
    # api.start()
    # print("Waiting for robot connection...")
    # if not api.wait_for_connection(timeout=10):
    #     print("Timeout waiting for robot. Exiting.")
    #     api.stop()
    #     exit(1)
    # print(f"Connected. IPOC: {api.monitoring.get_ipoc()}")
    # api.motion.update_cartesian(X=0.01, Y=0, Z=0)
    # print("Sent Cartesian correction. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 03: Send Joint correction (move A1 by 10 degrees)
    # DOESNT WORK with the default config - AKorr is only declared in
    # RSI_EthernetConfig_Full.xml; fails silently via a warning log, not an
    # exception.
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # api.motion.update_joints(A1=50)
    # print("Sent joint correction. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Example 04: External axes (move E1 by 100mm)
    # Requires: python main.py --config RSI_EthernetConfig_Full.xml
    # (the default config has no EKorr - move_external_axis() raises
    # RSIVariableError without the Full config's RECEIVE section)
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
    # time.sleep(5)
    # api.io.set_output(1, False)
  
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
    # corners = [
    #     {"X": 0, "Y": 0, "Z": 0},
    #     {"X": 50, "Y": 0, "Z": 0},
    #     {"X": 50, "Y": 50, "Z": 0},
    #     {"X": 0, "Y": 50, "Z": 0},
    #     {"X": 0, "Y": 0, "Z": 0},
    # ]
    # square = []
    # for start, end in zip(corners, corners[1:]):
    #     square += api.motion.generate_trajectory(start, end, steps=50, mode="relative")
    # api.motion.execute_trajectory(square, space="cartesian", points="delta")
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
    # # group=None: neither shipped config declares a 'Digin' per-bit group
    # # (wait_for_signal's default) - fall back to the DiL word's bit 0 via
    # # get_input()'s auto-detection instead.
    # if api.krl.wait_for_signal(1, timeout=10.0, group=None):
    #     print("KRL ready. Processing...")
    #     time.sleep(1)
    #     # signal_complete() defaults to group='Digout', which is SEND-only
    #     # (the robot's own readback) and not writable in either shipped
    #     # config - use io.set_output() directly, which auto-detects the
    #     # writable DiO word instead.
    #     api.io.set_output(1, True)
    #     print("Handshake complete.")
    # else:
    #     print("Timeout waiting for KRL signal.")
    # input("Press Enter to stop.")
    # api.stop()

    # =========================================================================
    # Coordination 02: Parameter Passing (read/write Tech variables)
    # Requires: python main.py --config RSI_EthernetConfig_Full.xml
    # (Tech.T1 - needed for T11-T13 - is only declared in the Full config's
    # RECEIVE section; the default config only declares Tech.T2)
    # =========================================================================
    # api.start()
    # time.sleep(1)
    # # C = KRL -> Python state channel (send_variables), T = Python -> KRL
    # # command channel (receive_variables) - see krl_api.py docstrings.
    # pos_x = api.krl.read_param('C11')
    # pos_y = api.krl.read_param('C12')
    # pos_z = api.krl.read_param('C13')
    # print(f"Current position from KRL: X={pos_x}, Y={pos_y}, Z={pos_z}")
    # api.krl.write_param('T11', pos_x + 50)
    # api.krl.write_param('T12', pos_y)
    # api.krl.write_param('T13', pos_z)
    # # signal_complete() defaults to group='Digout', which is SEND-only and
    # # not writable in either shipped config - use io.set_output() instead.
    # api.io.set_output(1, True)
    # print("Parameters sent. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Coordination 03: State Machine (multi-state workflow)
    # Requires: python main.py --config RSI_EthernetConfig_Full.xml
    # (Tech.T1 - needed for T11 - is only declared in the Full config's
    # RECEIVE section; the default config only declares Tech.T2)
    # =========================================================================
    # IDLE, CALIBRATING, READY, EXECUTING, COMPLETE, ERROR = 0, 1, 2, 3, 4, 5
    # api.start()
    # time.sleep(1)
    # state = IDLE
    # print("State machine running. Press Ctrl+C to exit.")
    # # C = KRL -> Python state channel, T = Python -> KRL command channel.
    # try:
    #     while state != COMPLETE:
    #         krl_state = int(api.krl.read_param('C11'))
    #         if krl_state == CALIBRATING:
    #             print("Calibrating...")
    #             time.sleep(2)
    #             api.krl.write_param('T11', READY)
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
    # time.sleep(4)
    # # Relative mode: each point is a per-cycle delta
    # # 200 steps × 0.5mm = 100mm total at max rate limit
    # traj = api.motion.generate_trajectory(
    #     {"X": 0, "Y": 0, "Z": 0},
    #     {"X": 100, "Y": 0, "Z": 0},
    #     steps=200,
    #     mode="relative")
    # print(f"Executing trajectory: {len(traj)} steps, {traj[0]['X']:.2f}mm per step")
    # api.motion.execute_trajectory(traj, space="cartesian", cycles_per_step=3, points="delta")
    # print("Trajectory executed. Press Enter to stop.")
    # input()
    # api.stop()

    # =========================================================================
    # Advanced Motion 02: Geometric Primitives (arc, circle, spiral)
    # =========================================================================
    api.start()
    print("Waiting for robot connection...")
    if not api.wait_for_connection(timeout=10):
        print("Timeout waiting for robot. Exiting.")
        api.stop()
        exit(1)
    print(f"Connected. IPOC: {api.monitoring.get_ipoc()}")
    input("Press Enter to start movement...")

    # Generate absolute circle waypoints, convert to relative deltas
    circle_abs = api.motion.generate_circle(
        center={"X": 0, "Y": 0, "Z": 0},
        radius=5, steps=200)

    # Convert absolute → relative (delta between consecutive points)
    # Start prev at first point so there's no initial jump
    circle_rel = []
    prev = circle_abs[0]
    for pt in circle_abs[1:]:
        delta = {k: pt[k] - prev.get(k, 0) for k in pt}
        circle_rel.append(delta)
        prev = pt

    print(f"Executing circle: {len(circle_rel)} relative steps, radius=5mm")
    # points="delta": circle_rel already contains per-cycle deltas; the
    # trajectory executor's one-shot latch zeroes RKorr automatically once
    # the last waypoint is applied, so no manual zero-out is needed after.
    api.motion.execute_trajectory(circle_rel, space="cartesian", cycles_per_step=3, points="delta")
    print("Circle complete. Press Enter to stop.")
    input()
    api.stop()

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
