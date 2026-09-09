"""Characterise RSI timing on this PC - no motion, so no confirm() needed.

Needs context("joints") (the default): the timing metrics below come from
IPOC/packet arrival, which every context reports, plus the robot-side
Delay counter, which every shipped context declares as DEF_Delay (INTERNAL,
no channel cost) except it is not exposed through a public accessor -
reading api.client.send_variables.get("Delay") directly is the documented
exception for that (see the comment at that line below).

What good looks like: mean cycle time close to 4 ms (0.004 s) with jitter
(the cycle-time standard deviation) well under 1 ms, and Delay staying at 0
- Delay is the robot's own count of late packets, so any nonzero value
there is real, robot-observed lateness, not just this script's opinion.

Under the emulator every number here describes loopback UDP on this PC
between the RSIPI client and rsi_echo_server - there is no real network,
switch, or robot-side interpolator in the loop, so this run cannot show
what a real KRC4 link's timing looks like. It is a baseline for "is this
machine capable of holding a steady 4 ms loop at all", not a network test.

    python examples/example_16_timing_diagnostics.py
    python examples/dry_run.py examples/example_16_timing_diagnostics.py
"""
import sys
import time

from RSIPI import RSIAPI, context

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else context("joints")
    api = RSIAPI(CONFIG)
    api.start()
    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s - is RSI_ON running?")
        api.stop()
        sys.exit(1)

    print("Built-in live view for 3 s (watch_network):")
    api.monitoring.watch_network(duration=3.0)

    print("\nSampling timing metrics for 5 s...")
    start = time.time()
    end = start + 5.0
    next_print = 0.0
    while time.time() < end:
        # Metrics accumulate in the background (NetworkProcess updates them
        # every cycle) - polling once a second just shows the run settling,
        # the real summary is the snapshot taken after the loop.
        if time.time() >= next_print:
            snap = api.diagnostics.get_stats()
            print(f"  t={time.time() - start:4.1f}s  "
                  f"cycles={snap.get('total_cycles', 0):5d}  "
                  f"jitter={snap.get('jitter', 0.0) * 1000:5.3f}ms")
            next_print = time.time() + 1.0
        time.sleep(0.05)

    # api.diagnostics wraps TimingMetrics (src/RSIPI/timing_metrics.py),
    # updated by NetworkProcess every cycle and shared back through
    # RSIClient.metrics_dict (src/RSIPI/rsi_client.py:118). total_cycles is
    # cumulative since start(); mean/max/jitter are over the rolling window
    # of the last 1000 cycles (4 s at 4 ms).
    stats = api.diagnostics.get_stats()
    quality = api.diagnostics.get_network_quality()

    # No public accessor exposes Delay (the robot's own late-packet count,
    # DEF_Delay in every shipped context) - reading send_variables directly
    # is the one exception the API allows for this, per its own docstring
    # in monitoring_api.py (get_motor_currents/get_force do the same).
    delay = api.client.send_variables.get("Delay")

    print("\nTiming summary:")
    print(f"  packets received (cumulative): {stats.get('total_cycles', 0)}")
    print(f"  mean cycle interval:  {stats.get('mean_cycle_time', 0.0) * 1000:.3f} ms")
    print(f"  max cycle interval:   {stats.get('max_cycle_time', 0.0) * 1000:.3f} ms")
    print(f"  jitter (std dev):     {stats.get('jitter', 0.0) * 1000:.3f} ms")
    print(f"  IPOC gap rate:        {quality.get('ipoc_gap_rate', 0.0):.2f} per 1000 cycles "
          "(client-side: gaps in the robot's IPOC sequence)")
    print(f"  packet loss rate:     {stats.get('packet_loss_rate', 0.0):.3f} %")
    print(f"  robot-side Delay:     {delay!r} (robot's own late-packet counter)")
    print(f"  healthy:              {stats.get('is_healthy', False)}")
    warnings = stats.get('warnings', [])
    if warnings:
        print("  warnings:")
        for w in warnings:
            print(f"    - {w}")

    api.stop()
