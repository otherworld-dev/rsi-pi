"""
RSIPI Comprehensive Test Script
================================
Matching Python counterpart for RSIPI_Test.src KRL program.
Tests all RSIPI functionality in coordination with the robot.

Protocol (Tech.C11 = state from KRL, Tech.T21 = command from Python):
  KRL States:  0=Idle, 1=Waiting, 2=Corrections, 3=SEN_PREA, 4=I/O, 5=Done
  Python Cmds: 1=Ready, 2=Stop, 3=SEN_PREA written, 4=Start I/O, 5=Shutdown

Usage:
    1. Copy controller/Program/RSIPI_Test.src to KRC:\\R1\\Program
    2. Run this script FIRST (the robot breaks off 0.4 s after RSI_ON if
       nothing answers):
           python examples/rsipi_test.py [config.xml]
       Default config: the packaged 'basic' context
       Pass the Joints config instead if RSIPI_Test.src loads RSIPI_Joints.rsi
       - the KRL program and this script must name the same pair.
    3. Start the KRL program on the pendant
"""

import time
import sys
import os
from multiprocessing import freeze_support

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from RSIPI import RSIAPI

# ── Helpers ──────────────────────────────────────────────────────────────

def wait_for_state(api, state, timeout=30):
    """Wait for KRL to reach a specific state via Tech.C11."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            krl_state = int(api.krl.read_param('C11'))
            if krl_state == state:
                return True
        except Exception:
            pass
        time.sleep(0.05)
    print(f"  TIMEOUT waiting for KRL state {state}")
    return False


def send_command(api, cmd):
    """Send a command to KRL via Tech.T21."""
    api.krl.write_param('T21', cmd)
    print(f"  -> Sent command: {cmd}")


# ── Main Test Sequence ───────────────────────────────────────────────────

if __name__ == '__main__':
    freeze_support()

    default_config = os.path.join(
        os.path.dirname(__file__), '..', 'controller', 'SensorInterface',
        'RSI_EthernetConfig_Basic.xml')
    config = sys.argv[1] if len(sys.argv) > 1 else default_config

    api = RSIAPI(
        config,
        rsi_mode='relative',
        max_cartesian_rate=0.5,
        max_joint_rate=0.2,
        cycle_time=0.004
    )

    print("=" * 60)
    print("RSIPI Comprehensive Test")
    print("=" * 60)

    # ── Start RSI and wait for robot ────────────────────────────────
    print("\n[1] Starting RSI connection...")
    api.start()
    if not api.wait_for_connection(timeout=30):
        print("  FAILED: No connection. Is the KRL program running?")
        api.stop()
        sys.exit(1)
    print(f"  Connected! IPOC: {api.monitoring.get_ipoc()}")

    # ── Wait for KRL to reach state 1 (RSI active, waiting) ────────
    print("\n[2] Waiting for KRL program to initialise RSI...")
    if not wait_for_state(api, 1):
        print("  FAILED: KRL did not reach state 1")
        api.stop()
        sys.exit(1)
    print("  KRL is ready and waiting for us")

    # Read the position KRL sent us
    try:
        pos_x = api.krl.read_param('C12')
        pos_y = api.krl.read_param('C13')
        pos_z = api.krl.read_param('C14')
        print(f"  Robot position from KRL: X={pos_x:.1f} Y={pos_y:.1f} Z={pos_z:.1f}")
    except Exception as e:
        print(f"  Could not read position: {e}")

    # ── Signal ready and start corrections ──────────────────────────
    print("\n[3] Signalling ready, starting correction phase...")
    send_command(api, 1)

    if not wait_for_state(api, 2):
        print("  FAILED: KRL did not enter correction mode")
        api.stop()
        sys.exit(1)
    print("  KRL is in RSI_MOVECORR - sending corrections...")

    # Send a small circle pattern as corrections
    circle = api.motion.generate_circle(
        center={"X": 0, "Y": 0, "Z": 0},
        radius=3, steps=100)

    # Convert to relative deltas
    circle_rel = []
    prev = circle[0]
    for pt in circle[1:]:
        delta = {k: pt[k] - prev.get(k, 0) for k in pt}
        circle_rel.append(delta)
        prev = pt

    print(f"  Executing circle: {len(circle_rel)} steps, radius=3mm")
    # circle_rel holds per-cycle deltas - points='delta' executes them via the
    # exactly-once path (default points='world' would re-diff them as poses)
    api.motion.execute_trajectory(circle_rel, space="cartesian",
                                  cycles_per_step=3, points="delta")
    print("  Circle complete!")

    time.sleep(1)

    # Tell KRL to stop corrections, then actually end the sensor-guided
    # motion. RSI_MOVECORR() blocks the KRL program forever on its own -
    # only a STOP object (Mode=ExitMoveCorr) can cancel it, so the command
    # below is what lets KRL advance to the next phase.
    print("  Stopping corrections...")
    api.motion.update_cartesian(X=0.0, Y=0.0, Z=0.0)
    send_command(api, 2)
    try:
        print("  " + api.motion.exit_movecorr())
    except Exception as e:
        print(f"  WARNING: could not cancel RSI_MOVECORR ({e})")
        print("  The context needs a STOP object - see controller/README.md")
    time.sleep(1)

    # ── SEN_PREA test ───────────────────────────────────────────────
    print("\n[4] Testing SEN_PREA variable exchange...")
    if not wait_for_state(api, 3):
        print("  FAILED: KRL did not reach SEN_PREA phase")
        api.stop()
        sys.exit(1)

    # Write test values to $SEN_PREA[1-3] via the dedicated SenP1-3 channels
    # (MAP2SEN_PREA1-3 in the RSI context). These are deliberately NOT the
    # RKorr channels: KUKA's example wires MAP2SEN_PREA to ETHERNET outputs
    # 1-3, which are RKorr.X/Y/Z, so sending sensor data would also command a
    # Cartesian correction - and motion rate limiting would clamp the data
    # (measured: 42.0/123.456/-99.9 arrived as 0.5/0.5/-0.5).
    test_vals = [42.0, 123.456, -99.9]
    print(f"  Writing SEN_PREA test values: {test_vals}")
    try:
        for i, value in enumerate(test_vals, start=1):
            api.tools.update_variable(f"SenP{i}", value)
    except Exception as e:
        print(f"  FAILED to write SenP channels ({e})")
        print("  This context has no dedicated $SEN_PREA channels - rebuild it")
    time.sleep(0.5)

    # Signal KRL to read them
    send_command(api, 3)
    time.sleep(1)

    # Read back what KRL echoed via Tech.C18-C110
    try:
        echo1 = api.krl.read_param('C18')
        echo2 = api.krl.read_param('C19')
        echo3 = api.krl.read_param('C110')
        print(f"  KRL echoed back: [{echo1}, {echo2}, {echo3}]")
        print(f"  Match: {abs(echo1 - test_vals[0]) < 0.1 and abs(echo2 - test_vals[1]) < 0.1}")
    except Exception as e:
        print(f"  Could not read echo values: {e}")

    # Zero corrections
    api.motion.update_cartesian(X=0, Y=0, Z=0)

    # ── Digital I/O test ────────────────────────────────────────────
    print("\n[5] Testing Digital I/O coordination...")
    if not wait_for_state(api, 4, timeout=10):
        print("  Skipping I/O test (KRL not in I/O phase)")
    else:
        send_command(api, 4)

        # Wait for KRL to activate gripper (state 41)
        if wait_for_state(api, 41, timeout=10):
            # Read digital output state from robot
            live = api.monitoring.get_live_data()
            print(f"  KRL activated gripper ($OUT[1])")
            print(f"  Live position: {live['position']}")

            # Acknowledge
            send_command(api, 41)

            # Wait for KRL to deactivate (state 42)
            if wait_for_state(api, 42, timeout=10):
                print(f"  KRL deactivated gripper ($OUT[1])")
                send_command(api, 42)
            else:
                print("  TIMEOUT waiting for gripper off")
        else:
            print("  TIMEOUT waiting for gripper on")

    # ── Shutdown ────────────────────────────────────────────────────
    print("\n[6] Shutting down...")
    if wait_for_state(api, 5, timeout=10):
        send_command(api, 5)
        print("  KRL acknowledged shutdown")
    else:
        print("  KRL did not reach shutdown state, stopping anyway")

    time.sleep(1)
    api.stop()

    print("\n" + "=" * 60)
    print("RSIPI Test Complete!")
    print("=" * 60)
