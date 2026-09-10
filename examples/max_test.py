"""Hardware acceptance test for the RSIPI_Max context.

Everything RSIPI_Max adds over RSIPI_Joints is unverified on a robot. This
walks the additions in order of risk, measuring rather than assuming, and
prints a PASS/FAIL summary at the end.

Pair with controller/Program/RSIPI_Minimal.src (edited to load
RSIPI_Max.rsi), or any program that reaches RSI_MOVECORR.

    python examples/max_test.py [config.xml]

Order matters: start this FIRST, then press Start past the HALT on the
pendant. Hand on the enabling switch and the hardware E-stop throughout.

THE BIGGEST RISK IS SIMPLY LOADING. RSIPI_Max wires ANIN, SEN_PINT, OV_PRO,
MAP2ANOUT, MAP2SEN_PINT, MAP2OV_PRO, POSCORRMON and AXISCORRMON. RSIVisual
opened and re-saved the context without complaint, so the XML is structurally
valid, but only the controller can say whether every object binds on this
robot. If RSI_CREATE or RSI_ON fails, that is the result - note which object
and fall back to RSIPI_Joints.
"""
import sys
import time

from RSIPI import RSIAPI, context

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
CONFIG = _args[0] if _args else context("max")
RATE_LIMIT = 0.1      # mm per robot cycle (~25 mm/s at 4 ms)
MOVE_MM = 5.0

results = {}


def check(name, passed, detail=""):
    results[name] = passed
    mark = "PASS" if passed else "FAIL"
    print(f"  [{mark}] {name}{'  - ' + detail if detail else ''}")
    return passed


def ask(prompt):
    return input(f"\n{prompt} [yes/NO]: ").strip().lower() == "yes"


def test_override(api):
    """$OV_PRO is the pendant's speed dial, and MAP2OV_PRO writes it EVERY
    cycle. RSIPI defaults OvProW to 100 precisely so RSI cannot silently
    command 0% - this confirms that on real hardware."""
    print("\n== 1. Program override ==")
    value = api.monitoring.get_override()
    print(f"  robot reports $OV_PRO = {value}")
    print("  >>> LOOK AT THE PENDANT: the override % should be unchanged,")
    print("      NOT 0. If it dropped to 0, MAP2OV_PRO is commanding a stop.")
    ok = check("override reads back", value is not None and value > 0,
               f"got {value}")

    if ok and ask("Set override to 50% from Python? (pendant should follow)"):
        api.monitoring.set_override(50)
        time.sleep(1.0)
        now = api.monitoring.get_override()
        check("override is writable", now == 50, f"reads {now}, pendant should show 50%")
        api.monitoring.set_override(100)
        time.sleep(0.5)
        print(f"  restored to {api.monitoring.get_override()}")


def test_monitors(api):
    """POSCORRMON/AXISCORRMON report what the controller ACTUALLY applied -
    the one thing a commanded value cannot tell you."""
    print("\n== 2. Applied-correction monitors ==")
    if not ask(f"Move +{MOVE_MM:.0f} mm in X at ~25 mm/s? Robot WILL move."):
        print("  skipped")
        return

    start = api.motion.get_current_pose()["X"]
    api.motion.move_cartesian_trajectory({"X": start + MOVE_MM}, steps=50)
    time.sleep(0.5)

    moved = api.motion.get_current_pose()["X"] - start
    applied = api.monitoring.get_applied_correction()
    print(f"  measured move      {moved:+.2f} mm")
    print(f"  PosCorrMon reports {applied.get('X', 0):+.2f} mm")

    check("robot actually moved", abs(moved) > 0.3, f"{moved:+.2f} mm")
    check("PosCorrMon tracks the applied correction",
          abs(applied.get("X", 0) - moved) < 0.5,
          f"monitor {applied.get('X', 0):+.2f} vs measured {moved:+.2f}")

    api.motion.move_cartesian_trajectory({"X": start}, steps=50)
    time.sleep(0.5)
    print(f"  returned to X={api.motion.get_current_pose()['X']:.2f}")

    joints = api.monitoring.get_applied_joint_correction()
    check("AxisCorrMon is present and reporting", bool(joints), str(joints))


def test_motor_currents(api):
    print("\n== 3. Motor currents ==")
    print("  DEF_MACur is an INTERNAL declaration - no object, no channel.")
    currents = api.monitoring.get_force()
    print(f"  {currents}")
    live = any(abs(float(v or 0)) > 1e-6 for v in currents.values())
    check("motor currents are non-zero", live,
          "all zero - either the robot is perfectly still or MACur is not reported")


def test_analog(api):
    print("\n== 4. Analogue I/O ==")
    try:
        value = api.io.read_analog()
        print(f"  $ANIN[1] = {value}")
        check("analogue input readable", True, f"{value}")
    except Exception as e:
        check("analogue input readable", False, f"{type(e).__name__}: {e}")

    if ask("Write 0.5 to $ANOUT[1]? (check on the pendant: Display > Analog I/O)"):
        try:
            api.io.set_analog(0.5)
            time.sleep(1.0)
            check("analogue output writable", True, "pendant should show 0.5")
        except Exception as e:
            check("analogue output writable", False, f"{type(e).__name__}: {e}")
        api.io.set_analog(0.0)


def test_sen_pint(api):
    """$SEN_PINT is the integer channel to the KRL program."""
    print("\n== 5. $SEN_PINT round trip ==")
    try:
        api.krl.write_sen_pint(7)
        time.sleep(1.0)
        got = api.krl.read_sen_pint()
        check("$SEN_PINT round trip", got == 7, f"wrote 7, read {got}")
        print("  (confirm on the pendant: Display > Variable > Single, $SEN_PINT[1])")
        api.krl.write_sen_pint(0)
    except Exception as e:
        check("$SEN_PINT round trip", False, f"{type(e).__name__}: {e}")


def test_correction_clamping(api):
    """POSCORR clamps silently - Stat is the only thing that says so.

    Stat is INSTANTANEOUS: it reports what the correction is doing right now.
    So this holds a correction and samples Stat WHILE it is being applied.
    Reading it after a trajectory finishes returns 0, because by then nothing
    is being corrected - which looks like a failure and is not one.

    A held level accumulates every 4 ms cycle in relative mode, so it walks
    out to the POSCORR limit and then stays pinned there, which is exactly the
    condition the limit bits report.
    """
    print("\n== 6. Correction clamping (POSCORR Stat) ==")
    if api.monitoring.get_correction_limit_status() is None:
        print("  no PosCorrStat channel - skipping")
        return
    print("  Holds a correction until it hits the +/-50 mm POSCORR limit, then")
    print("  reads Stat WHILE it is pinned there. The robot stopping at the")
    print("  limit is expected; the question is whether it TELLS us.")
    if not ask("Drive X out to the correction limit? Robot WILL move ~50 mm."):
        print("  skipped")
        return

    start_x = api.motion.get_current_pose()["X"]
    api.safety.set_limit("RKorr.X", -60.0, 60.0)   # let the request through

    best = None
    last_x = start_x
    try:
        api.motion.update_cartesian(X=0.5)         # held: accumulates per cycle
        for _ in range(60):                        # up to ~12 s
            time.sleep(0.2)
            status = api.monitoring.get_correction_limit_status()
            if status and (best is None or status["raw"] > best["raw"]):
                best = status
            now_x = api.motion.get_current_pose()["X"]
            if status and status["limited"]:
                break                              # clamped - that is the answer
            if abs(now_x - last_x) < 0.01 and abs(now_x - start_x) > 1.0:
                break                              # stopped advancing = clamped
            last_x = now_x
    finally:
        api.motion.update_cartesian(X=0.0)
        time.sleep(0.3)

    moved = last_x - start_x
    print(f"  moved {moved:+.2f} mm before it stopped advancing")
    print(f"  best Stat seen: {best}")
    if best and best["limited"]:
        check("clamping is reported, not silent", True,
              f"at_limit={best['at_limit']}, raw={best['raw']}")
    elif abs(moved) > 1.0:
        check("clamping is reported, not silent", False,
              "the move was truncated but Stat never reported a limit - "
              "either it never reached the cap, or Stat is not wired through")
    else:
        print("  INCONCLUSIVE - the robot barely moved, so the limit was never")
        print("  reached. Not a Stat failure; re-run with a longer hold.")

    api.motion.move_cartesian_trajectory({"X": start_x}, steps=300)
    time.sleep(0.5)
    api.safety.set_limit("RKorr.X", -10.0, 10.0)
    print(f"  returned to X={api.motion.get_current_pose()['X']:.2f}")


def test_digital_word(api):
    """DIGOUT4's DataSize was corrected from Byte to Word on 2026-09-09, so
    the read-back should now cover all 16 outputs rather than the low 8."""
    print("\n== 7. Digital output word (16-bit read-back) ==")
    if "DoutW" not in api.client.send_variables:
        print("  no DoutW channel - skipping")
        return
    if not ask("Drive $OUT[17..32] (Index 3: the word starts at (Index-1)*8+1)? Make sure nothing is wired to them."):
        print("  skipped")
        return

    ok = True
    for word in (3, 300, 0):      # 300 needs bit 8 - the byte/word fix
        api.tools.update_variable("DiO", word)
        time.sleep(1.0)
        got = api.client.send_variables.get("DoutW")
        match = int(float(got or 0)) == word
        ok &= match
        print(f"  wrote {word:<4} read back {got!r:<8} {'MATCH' if match else 'MISMATCH'}")
    check("16-bit output word round trip", ok,
          "300 needs bit 8, so a mismatch there means DataSize is still Byte")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()

    print(f"Config: {CONFIG}")
    api = RSIAPI(CONFIG, rsi_mode="relative", max_cartesian_rate=RATE_LIMIT)
    for axis in ("X", "Y", "Z"):
        api.safety.set_limit(f"RKorr.{axis}", -10.0, 10.0)

    api.start()
    print("Waiting for the robot - press Start on the pendant (up to 120 s)...")
    if not api.wait_for_connection(120):
        print("\nNO PACKETS. If the pendant shows RSIBad or an RSI_CREATE error,")
        print("that IS the result: RSIPI_Max has an object this robot cannot bind.")
        print("Note which one from the pendant message, then fall back to")
        print("context('joints').")
        api.stop()
        sys.exit(1)

    print("\nCONNECTED - RSIPI_Max loaded and exchanging packets.\n")
    check("context loads and runs", True, "no RSIBad at RSI_ON")

    try:
        test_override(api)
        test_monitors(api)
        test_motor_currents(api)
        test_analog(api)
        test_sen_pint(api)
        test_correction_clamping(api)
        test_digital_word(api)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            api.client.zero_corrections()
        except Exception as e:
            print(f"cleanup warning: {type(e).__name__}: {e}")
        api.stop()

    print("\n" + "=" * 62)
    for name, passed in results.items():
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print("=" * 62)
    failed = [n for n, p in results.items() if not p]
    print(f"{len(results) - len(failed)}/{len(results)} passed"
          + (f" - failures: {', '.join(failed)}" if failed else ""))
    print("\nCancel the KRL program on the pendant.")
