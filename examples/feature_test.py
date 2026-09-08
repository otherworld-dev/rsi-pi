"""Session 1: exercise every RSIPI feature that runs on the Basic context.

Staged and prompted - each motion stage asks before moving, and a failing
stage never aborts the rest. A PASS/FAIL summary is printed at the end.

Stages
  1  Connection + diagnostics      no motion
  2  Digital I/O                   no motion (watch $OUT[161] on the pendant)
  3  Trajectory accuracy           20 mm out and back
  4  Velocity profile              profiled move, timed
  5  Safety limits                 clamp + rejection, small motion
  6  E-stop drill                  stop mid-move, verify no resume on reset
  7  CSV logging + report          logs a move, then analyses the file
  8  Reconnect                     tears the link down (RSI will break off)

Order: start this FIRST, then let RSIPI_Minimal past its HALT.

    python examples/feature_test.py controller/SensorInterface/RSI_EthernetConfig_Basic.xml
"""
import os
import sys
import time

from RSIPI import RSIAPI

CONFIG = "controller/SensorInterface/RSI_EthernetConfig_Basic.xml"
RATE_LIMIT = 0.1          # mm/cycle -> ~25 mm/s
LOG_FILE = "logs/feature_test.csv"

results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))


def ask(prompt):
    return input(f"\n{prompt} [yes/NO]: ").strip().lower() == "yes"


def stage(n, title):
    print(f"\n{'=' * 62}\nSTAGE {n}: {title}\n{'=' * 62}")


def x_of(api):
    return api.motion.get_current_pose().get("X", 0.0)


# ---------------------------------------------------------------- stages

def s1_diagnostics(api):
    stage(1, "Connection and diagnostics (no motion)")
    pose = api.motion.get_current_pose()
    record("pose is non-zero", any(abs(v) > 0.01 for v in pose.values()), str(pose))

    time.sleep(0.5)                       # let the shared IPOC settle
    ipoc0 = api.client.current_ipoc()
    time.sleep(2)
    advanced = api.client.current_ipoc() - ipoc0
    # 4 ms cycle -> IPOC is a millisecond stamp, so ~1000 units per second.
    record("IPOC advancing ~1000/s", 1800 < advanced < 2200, f"{advanced} in 2s")

    stats = dict(api.client.metrics_dict)
    mean_ms = stats.get("mean_cycle_time", 0) * 1000
    record("mean cycle time near 4 ms", 3.0 < mean_ms < 6.0, f"{mean_ms:.2f} ms")
    record("jitter under 2 ms", stats.get("jitter", 1) < 0.002,
           f"{stats.get('jitter', 0) * 1000:.2f} ms")
    record("no packet loss", stats.get("packet_loss_rate", 100) < 1.0,
           f"{stats.get('packet_loss_rate', 0):.2f}%")
    record("diagnostics report healthy", api.diagnostics.is_healthy())

    delay = api.client.send_variables.get("Delay")
    d = delay.get("D") if isinstance(delay, dict) else delay
    record("robot reports no late packets", float(d or 0) == 0, f"DEF_Delay={d}")


def s2_digital_io(api):
    stage(2, "Digital I/O (no motion)")
    print("MAP2DIGOUT1 uses Index=20 with DataSize=Word. Per the RSI object")
    print("reference, a non-Bit DataSize makes Index a BYTE index - so byte 20")
    print("is $OUT[161]..$OUT[176]: DiO bit 0 -> $OUT[161], bit 1 -> $OUT[162].")
    print("On the pendant: Display > Inputs/Outputs > Digital Outputs, scroll to")
    print("161-162. (If those are assigned in your cell, change MAP2DIGOUT1's")
    print("Index to a free byte.)")
    if not ask("Ready to toggle outputs?"):
        record("digital output write", False, "skipped")
        return
    try:
        for word, label in ((3, "BOTH $OUT[161] and $OUT[162] ON"), (0, "both OFF")):
            api.tools.update_variable("DiO", word)
            sent = api.client.receive_variables.get("DiO")
            print(f"  wrote DiO={word} (shared dict now {sent!r}) - holding 3 s")
            time.sleep(3)
            if word:
                on = ask(f"Did you see {label}?")
            else:
                off = ask(f"Did you see {label}?")
        record("DiO word drives $OUT[161]/$OUT[162]", on and off)
    except Exception as e:
        record("DiO word drives $OUT[161]/$OUT[162]", False, f"{type(e).__name__}: {e}")

    try:
        val = api.io.get_input(1)
        raw = api.client.send_variables.get("DiL")
        record("digital input read", isinstance(val, bool), f"input1={val} DiL={raw}")
    except Exception as e:
        record("digital input read", False, f"{type(e).__name__}: {e}")


def s3_trajectory(api):
    stage(3, "Trajectory accuracy (20 mm out and back)")
    if not ask("Move 20 mm in +X, then back?"):
        record("trajectory accuracy", False, "skipped")
        return
    start = x_of(api)
    api.motion.move_cartesian_trajectory({"X": start + 20.0}, steps=200)
    time.sleep(0.8)
    out = x_of(api)
    api.motion.move_cartesian_trajectory({"X": start}, steps=200)
    time.sleep(0.8)
    back = x_of(api)
    # RIst is reported to 1 decimal (ETHERNET Precision=1), so allow 0.3 mm.
    record("outward move accurate", abs((out - start) - 20.0) < 0.3,
           f"moved {out - start:+.2f} of 20.0 mm")
    record("returns to start", abs(back - start) < 0.3,
           f"net {back - start:+.2f} mm")


def s4_velocity_profile(api):
    stage(4, "Velocity profile execution")
    if not ask("Run a velocity-profiled 10 mm move?"):
        record("profiled execution", False, "skipped")
        return
    start = x_of(api)
    traj = api.motion.generate_trajectory({"X": start}, {"X": start + 10.0}, steps=100)
    profiled = api.motion.generate_velocity_profile(traj, max_velocity=20.0,
                                                    max_acceleration=50.0)
    record("profile returns (waypoint, velocity) pairs",
           len(profiled) == len(traj) and isinstance(profiled[0], tuple),
           f"{len(profiled)} points, peak {max(v for _, v in profiled):.1f} mm/s")
    t0 = time.time()
    api.motion.execute_profiled_trajectory(profiled, space="cartesian")
    elapsed = time.time() - t0
    time.sleep(0.8)
    moved = x_of(api) - start
    record("profiled move reaches target", abs(moved - 10.0) < 0.4,
           f"moved {moved:+.2f} mm in {elapsed:.1f}s")
    api.motion.move_cartesian_trajectory({"X": start}, steps=100)
    time.sleep(0.8)


def s5_safety(api):
    stage(5, "Safety limits")
    from RSIPI.exceptions import RSISafetyViolation   # RSILimitExceeded subclasses it
    api.safety.set_limit("RKorr.X", -0.5, 0.5)
    status = api.safety.status()
    record("limit registered", status["limits"].get("RKorr.X") == (-0.5, 0.5),
           str(status["limits"].get("RKorr.X")))
    try:
        api.motion.update_cartesian(X=5.0)      # far outside the limit
        record("write-time validation rejects out-of-range", False, "no exception")
    except RSISafetyViolation as e:
        record("write-time validation rejects out-of-range", True, type(e).__name__)
    except Exception as e:
        record("write-time validation rejects out-of-range", False,
               f"unexpected {type(e).__name__}")
    finally:
        api.motion.update_cartesian(X=0.0)

    # Runaway protection: bypass the API entirely and write a large value
    # straight into the shared dict. Nothing validates it there, so the only
    # things standing between this and a 5 mm/cycle (1250 mm/s) command are
    # the send-time clamp and the per-cycle rate limiter. Exact clamp
    # behaviour is unit-tested; here we only prove the robot cannot run away.
    exposure = 0.4
    budget = RATE_LIMIT * (exposure / api.client.cycle_time) * 1.6   # + margin
    before = x_of(api)
    corr = dict(api.client.receive_variables.get("RKorr", {}))
    corr["X"] = 5.0
    api.client.receive_variables["RKorr"] = corr
    time.sleep(exposure)
    api.client.zero_corrections()
    time.sleep(0.6)
    drift = abs(x_of(api) - before)
    record("direct write cannot run away", drift < budget,
           f"drifted {drift:.2f} mm, bounded by {budget:.1f} "
           f"(unlimited would be ~{5.0 * exposure / api.client.cycle_time:.0f} mm)")
    api.safety.set_limit("RKorr.X", -6.0, 6.0)
    if drift > 0.5 and ask("Return to the pre-clamp position?"):
        api.motion.move_cartesian_trajectory({"X": before}, steps=200)
        time.sleep(0.8)


def s6_estop(api):
    stage(6, "E-stop drill")
    if not ask("Start a slow 20 mm move and fire the software E-stop mid-way?"):
        record("e-stop", False, "skipped")
        return
    import threading
    start = x_of(api)

    def fire():
        time.sleep(1.5)
        api.safety.stop()
        print("  >> E-STOP sent")

    threading.Thread(target=fire, daemon=True).start()
    try:
        api.motion.move_cartesian_trajectory({"X": start + 20.0}, steps=400)
    except Exception as e:
        print(f"  trajectory aborted: {type(e).__name__}")
    time.sleep(0.6)
    stopped = x_of(api)
    record("e-stop halts motion early", abs(stopped - start) < 19.0,
           f"stopped after {stopped - start:+.2f} of 20 mm")

    api.safety.reset()
    time.sleep(2.5)
    after = x_of(api)
    record("reset does NOT resume motion", abs(after - stopped) < 0.3,
           f"drift after reset {after - stopped:+.2f} mm")
    if ask("Return to the start position?"):
        api.motion.move_cartesian_trajectory({"X": start}, steps=200)
        time.sleep(0.8)


def s7_logging(api):
    stage(7, "CSV logging and reporting")
    os.makedirs("logs", exist_ok=True)
    api.logging.start(LOG_FILE)
    time.sleep(0.5)
    record("logging reports active", api.logging.is_active())
    start = x_of(api)
    if ask("Log a 5 mm move?"):
        api.motion.move_cartesian_trajectory({"X": start + 5.0}, steps=100)
        time.sleep(0.5)
        api.motion.move_cartesian_trajectory({"X": start}, steps=100)
    else:
        time.sleep(3)
    api.logging.stop()
    time.sleep(1.0)

    exists = os.path.exists(LOG_FILE)
    record("log file created", exists, LOG_FILE)
    if not exists:
        return
    with open(LOG_FILE) as f:
        header = f.readline().strip().split(",")
        rows = sum(1 for _ in f)
    record("log has robot-state columns", any(c.startswith("Send.RIst.") for c in header),
           f"{len(header)} columns, {rows} rows")
    record("log captured many cycles", rows > 50, f"{rows} rows")
    try:
        print("  " + api.tools.generate_report(LOG_FILE, "csv"))
        record("report generated from log", True)
    except Exception as e:
        record("report generated from log", False, f"{type(e).__name__}: {e}")


def s8_reconnect(api):
    stage(8, "Reconnect (the robot WILL break off - do this last)")
    print("This drops the UDP link; the pendant will report RSIBad and stop.")
    if not ask("Run the reconnect test?"):
        record("reconnect", False, "skipped")
        return
    api.client.reconnect()
    time.sleep(1.0)
    record("client restarts after reconnect", api.is_running(),
           f"state={api.state.name}")
    print("  Re-select and restart RSIPI_Minimal on the pendant to reconnect.")
    ok = api.wait_for_connection(60)
    record("reconnected to robot", ok)


# ------------------------------------------------------------------ main

if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    config = sys.argv[1] if len(sys.argv) > 1 else CONFIG
    api = RSIAPI(config, max_cartesian_rate=RATE_LIMIT)
    api.safety.set_limit("RKorr.X", -6.0, 6.0)
    print(f"Config: {config}")
    api.start()
    print("\nListening - now start RSIPI_Minimal and press Start past the HALT "
          "(up to 120 s)...")
    if not api.wait_for_connection(120):
        print("No packets. Check the order: Python first, then the pendant.")
        api.stop()
        sys.exit(1)
    print("CONNECTED\n")

    try:
        for fn in (s1_diagnostics, s2_digital_io, s3_trajectory, s4_velocity_profile,
                   s5_safety, s6_estop, s7_logging, s8_reconnect):
            try:
                fn(api)
            except KeyboardInterrupt:
                raise
            except Exception as e:
                record(fn.__name__, False, f"stage crashed: {type(e).__name__}: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            api.client.zero_corrections()
            api.stop()
        except Exception:
            pass

    print(f"\n{'=' * 62}\nSUMMARY\n{'=' * 62}")
    for name, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    failed = [n for n, ok, _ in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    if failed:
        print("Failed/skipped: " + ", ".join(failed))
