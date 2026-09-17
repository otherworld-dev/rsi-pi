"""How does the robot handle a PC that stalls? Two questions, measured.

1. HOLDON: DOES A STALL KEEP THE ROBOT MOVING?   (the robot moves ~7 mm)
   In relative mode corrections add up, and HOLDON=1 keeps "the most recent
   valid value" at the output while replies are missing. Read together, every
   cycle the PC misses repeats the last step it sent - so a stall during a
   relative plunge would go deeper. Nobody has watched it happen.

   The script streams a constant 0.04 mm/cycle in +X (10 mm/s), freezes its
   network process for 25 cycles mid-move, then stops and measures:

       moved ~ every cycle x step          -> KEEPS MOVING (held steps repeat)
       moved ~ (every cycle - missed) x step -> STOPS (missed cycles add nothing)

   The two differ by 1 mm, ten times the 0.1 mm the Basic context reports
   (Precision=1). The robot is then driven back to where it started.

2. TIMEOUT: CONSECUTIVE LATE PACKETS, OR A RUNNING TOTAL?   (no motion)
   KUKA's object reference calls Timeout the "maximum number of cycles
   tolerated without an answer from the communication partner"; neither it
   nor the manual says whether a good reply resets the count. RSIPI's echo
   server assumes the stricter reading, a running total.

   The script freezes its network process several times, each freeze a
   quarter of Timeout long, together at least twice Timeout, and reads
   DEF_Delay (the controller's late-packet counter) after each:

       RSI keeps running, Delay passes Timeout  -> CONSECUTIVE
       RSI stops with RSIBad (KSS29002)         -> RUNNING TOTAL. The robot was
                                                   stationary; acknowledge it.
       RSI keeps running, Delay stays below     -> INCONCLUSIVE (raise --freezes)

   Delay should rise about one per frozen cycle. Each line also shows how
   many queued robot packets RSIPI passed over on resuming, which checks on
   the robot that the PC answers only the newest packet after a stall.

This order is deliberate: if Timeout is a running total, stage 2 ends RSI.
Each stage asks first; answer anything but "yes" to skip it.

Pair with controller/Program/RSIPI_Minimal.src, which creates RSIPI_Basic
(Timeout=100, HOLDON=1 on RKorr). For another context, pass its .rsi.xml with
--rsi.

    python examples/stall_probe.py [config.xml] [--rsi FILE] [--freezes N] [--cycles N]

Order matters: start this FIRST, then press Start past the HALT on the
pendant. Hand on the enabling switch and the hardware E-stop throughout -
while a freeze lasts, the PC cannot stop anything either.
"""
import argparse
import math
import textwrap
import time

from RSIPI import RSIAPI, context
from RSIPI.context import CONTEXT_DIR
from RSIPI.fault_injection import suspended
from RSIPI.rsi_limit_parser import parse_ethernet_timeout

from _confirm import confirm

DEFAULT_RSI = CONTEXT_DIR / "RSIPI_Basic.rsi.xml"
MOVE_RATE = 0.04      # mm per robot cycle in stage 1: 10 mm/s at 4 ms; also the rate limit
MOVE_S = 0.3          # stage 1 streams this long before and after the freeze
SETTLE_S = 2.0        # after each freeze: the link recovers and Delay reaches the PC


def read_delay(api):
    """DEF_Delay as last received, or None when the config does not stream it."""
    value = api.client.send_variables.get("Delay")
    if isinstance(value, dict):
        value = value.get("D")
    return None if value is None else int(value)


def robot_sending(api, window=0.5):
    """True while the robot's IPOC is still advancing."""
    before = api.client.current_ipoc()
    time.sleep(window)
    return api.client.current_ipoc() != before


def skipped_packets(api):
    """Queued robot packets the network process has passed over so far."""
    return api.diagnostics.get_stats().get("skipped_packets", 0)


def report(name, reason):
    print(f"\n  => {name}")
    print(textwrap.fill(reason, width=66, initial_indent="     ",
                        subsequent_indent="     "))


# ------------------------------------------------------------------ stage 1

def probe_holdon(api, cycles, cycle_s):
    """Stream a constant step, freeze mid-move, and measure what the robot did."""
    client = api.client
    print("\n== Stage 1: does a stall keep the robot moving? ==")
    print(f"  config: RKorr.X HOLDON={client.config_parser.holdon_map.get('RKorr.X', 1)}")
    start_x = api.motion.get_current_pose()["X"]
    delay_before = read_delay(api)

    client.publish_corrections({"RKorr": {"X": MOVE_RATE}})
    ipoc_on = client.current_ipoc()
    try:
        time.sleep(MOVE_S)
        with suspended(client.network_process.pid):
            time.sleep(cycles * cycle_s)
        time.sleep(MOVE_S)
    finally:
        client.zero_corrections()
    ipoc_off = client.current_ipoc()
    time.sleep(SETTLE_S)

    moved = api.motion.get_current_pose()["X"] - start_x
    streamed = round((ipoc_off - ipoc_on) / (cycle_s * 1000))
    missed = read_delay(api) - delay_before
    if_held = streamed * MOVE_RATE
    if_reset = (streamed - missed) * MOVE_RATE
    print(f"  streamed for {streamed} cycles, {missed} of them missed by the PC")
    print(f"  moved {moved:+.2f} mm: {if_held:+.2f} if held steps repeat, "
          f"{if_reset:+.2f} if missed cycles add nothing")

    back_steps = max(1, math.ceil(abs(moved) / (MOVE_RATE / 2)))
    api.motion.move_cartesian_trajectory({"X": start_x}, steps=back_steps)
    time.sleep(0.5)
    print(f"  returned to X={api.motion.get_current_pose()['X']:.2f} "
          f"(started at {start_x:.2f})")

    difference = abs(if_held - if_reset)
    if difference < 0.5:
        return "INCONCLUSIVE", (
            f"the PC missed only {missed} cycles, too few to tell the outcomes "
            f"apart at this precision. Rerun with more --cycles.")
    if abs(moved - if_held) < difference / 2:
        return "KEEPS MOVING", (
            "During the freeze the robot carried on at the last step it was "
            "sent: every missed cycle repeats the last relative correction. A "
            "stall during a relative plunge goes deeper by missed cycles x "
            "step, for up to Timeout cycles before RSI breaks off.")
    if abs(moved - if_reset) < difference / 2:
        return "STOPS", (
            "During the freeze the robot held still: a missed cycle applies no "
            "correction. A stall during a relative move falls short by the "
            "missed steps instead of overshooting.")
    return "NEITHER", (
        f"moved {moved:+.2f} mm, which matches neither prediction. Check the "
        f"POSCORR limits and whether corrections are applied at all.")


# ------------------------------------------------------------------ stage 2

def probe_timeout(api, freezes, cycles, cycle_s):
    """Freeze the network process up to *freezes* times; one row per freeze."""
    pid = api.client.network_process.pid
    rows = []
    for number in range(1, freezes + 1):
        before = read_delay(api)
        skipped_before = skipped_packets(api)
        with suspended(pid):
            time.sleep(cycles * cycle_s)
        time.sleep(SETTLE_S)
        alive = robot_sending(api)
        after = read_delay(api)
        skipped = skipped_packets(api) - skipped_before
        rows.append({"before": before, "after": after, "alive": alive})
        print(f"  freeze {number}/{freezes}: Delay {before} -> {after} "
              f"(+{after - before}), {skipped} packets skipped on resume, "
              f"robot {'sending' if alive else 'SILENT'}")
        if not alive:
            break
    return rows


def timeout_verdict(rows, start_delay, timeout, cycles):
    """(name, explanation) from the freeze rows."""
    rises = [row["after"] - row["before"] for row in rows if row["alive"]]
    if rises:
        per_cycle = sum(rises) / (len(rises) * cycles)
        if per_cycle > 1.5:
            how = "the controller counts more than the unanswered cycles"
        elif per_cycle >= 0.5:
            how = "one per unanswered cycle, as expected"
        else:
            how = "fewer than the cycles frozen - is the cycle really 4 ms? (--cycle-ms)"
        print(f"\n  Delay rose {per_cycle:.2f} per frozen cycle: {how}.")

    last = rows[-1]
    if not last["alive"]:
        if rises and max(rises) >= timeout:
            return "INCONCLUSIVE", (
                f"one freeze alone cost {max(rises)} late packets, more than "
                f"Timeout={timeout}. Rerun with fewer --cycles.")
        return "RUNNING TOTAL", (
            f"RSI stopped during freeze {len(rows)}, with Delay last seen at "
            f"{last['after']}, although no freeze came near Timeout={timeout}. "
            f"Acknowledge KSS29002 on the pendant.")

    total = last["after"] - start_delay
    if total > timeout:
        return "CONSECUTIVE", (
            f"Delay rose by {total}, past Timeout={timeout}, and RSI kept running.")
    return "INCONCLUSIVE", (
        f"Delay rose by only {total}, not past Timeout={timeout}. "
        f"Rerun with more --freezes.")


# ------------------------------------------------------------------ main

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python examples/stall_probe.py",
        description="Measure how the robot handles PC stalls: HOLDON in "
                    "relative mode, and how ETHERNET Timeout counts.")
    parser.add_argument("config", nargs="?", default=context("basic"),
                        help="Ethernet config (default: the basic context)")
    parser.add_argument("--rsi", default=str(DEFAULT_RSI),
                        help="the deployed context's .rsi.xml, for its Timeout "
                             "(default: RSIPI_Basic)")
    parser.add_argument("--freezes", type=int, default=None,
                        help="stage 2 freezes (default: enough for twice Timeout)")
    parser.add_argument("--cycles", type=int, default=None,
                        help="robot cycles per freeze (default: a quarter of Timeout)")
    parser.add_argument("--cycle-ms", type=float, default=4.0,
                        help="robot cycle in ms (default: 4, IPO_FAST)")
    args = parser.parse_args(argv)

    timeout = parse_ethernet_timeout(args.rsi)
    if timeout is None:
        print(f"No ETHERNET Timeout found in {args.rsi}.")
        return 2
    cycles = args.cycles or max(1, timeout // 4)
    freezes = args.freezes or math.ceil(2 * timeout / cycles)
    if cycles >= timeout:
        print(f"--cycles must stay below Timeout={timeout}, or one freeze alone "
              "breaks RSI off and proves nothing.")
        return 2
    cycle_s = args.cycle_ms / 1000
    freeze_ms = cycles * args.cycle_ms

    print(f"Config:  {args.config}")
    print(f"Timeout: {timeout} (from {args.rsi})")
    print(f"Stage 1: +X at {MOVE_RATE / cycle_s:.0f} mm/s for about "
          f"{2 * MOVE_S * 1000 + freeze_ms:.0f} ms with one {freeze_ms:.0f} ms "
          f"freeze, then back")
    print(f"Stage 2: {freezes} freezes of {cycles} cycles ({freeze_ms:.0f} ms), "
          f"{SETTLE_S:.0f} s apart: at least {freezes * cycles} late packets, "
          f"about {cycles} in a row at most")

    api = RSIAPI(args.config, rsi_mode="relative", max_cartesian_rate=MOVE_RATE)
    for axis in ("X", "Y", "Z"):
        api.safety.set_limit(f"RKorr.{axis}", -MOVE_RATE, MOVE_RATE)
    api.start()
    print("Waiting for the robot - press Start on the pendant now (up to 120 s)...")
    if not api.wait_for_connection(120):
        print("\nNO PACKETS. Did the KRL program reach RSI_ON? Is the context "
              "on the controller with its Ethernet config beside it?")
        api.stop()
        return 1
    print("\nCONNECTED.")

    results = {}
    try:
        time.sleep(1.0)
        if read_delay(api) is None:
            print("This config does not stream DEF_Delay - nothing to measure.")
            return 1

        distance = (2 * MOVE_S + freeze_ms / 1000) / cycle_s * MOVE_RATE
        if confirm(
                f"Stage 1: move about {distance:.0f} mm in +X at "
                f"{MOVE_RATE / cycle_s:.0f} mm/s, freezing the PC mid-move?",
                f"Robot WILL move, and may carry on during the {freeze_ms:.0f} ms "
                "freeze - that is the question. It is driven back afterwards. "
                "Clear space in +X; hardware E-stop only while frozen."):
            results["stall during a relative move"] = probe_holdon(api, cycles, cycle_s)
            report(*results["stall during a relative move"])
        else:
            print("Stage 1 skipped.")

        if not robot_sending(api):
            print("\nThe robot has stopped sending - stage 2 cannot run.")
        elif confirm(
                f"Stage 2: freeze RSIPI's network process {freezes} times, "
                f"{freeze_ms:.0f} ms each?",
                "The robot does not move. If Timeout is a running total, RSI "
                "stops with RSIBad partway through. While frozen the PC cannot "
                "stop anything - hardware E-stop only."):
            print("\n== Stage 2: how does Timeout count? ==")
            start_delay = read_delay(api)
            print(f"  Delay at start: {start_delay}")
            rows = probe_timeout(api, freezes, cycles, cycle_s)
            results["Timeout counts"] = timeout_verdict(rows, start_delay, timeout, cycles)
            report(*results["Timeout counts"])
        else:
            print("Stage 2 skipped.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        try:
            api.client.zero_corrections()
        except Exception as e:
            print(f"cleanup warning: {type(e).__name__}: {e}")
        api.stop()

    print("\n" + "=" * 60)
    for question, (name, _) in results.items():
        print(f"  {question:<30} {name}")
    print("=" * 60)
    if results:
        print("Record the results in docs/hardware-findings.md.")
    return 0


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    raise SystemExit(main())
