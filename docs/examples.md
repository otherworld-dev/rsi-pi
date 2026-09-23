# Examples

The [`examples/`](https://github.com/otherworld-dev/rsi-pi/tree/main/examples) directory contains runnable scripts. Every one that moves
the robot stops and asks first (`Proceed? [yes/NO]`), stating the distance
and speed; anything but `yes` skips that step. All of them run against the
emulator without a robot:

```bash
python examples/dry_run.py examples/example_02_send_cartesian.py
```

| Script | Description |
|--------|-------------|
| `example_01_start_stop.py` | Basic lifecycle: connect, wait, disconnect |
| `example_02_send_cartesian.py` | Send Cartesian corrections (RKorr) |
| `example_03_send_joint.py` | Send joint corrections (AKorr) |
| `example_04_external_axes.py` | Control external axes (E1, E2, ...) |
| `example_05_digital_io.py` | Digital I/O: set outputs, read inputs, pulse |
| `example_06_logging_csv.py` | Start/stop CSV logging |
| `example_07_graphing_live.py` | Live 3D plot during operation |
| `example_08_safety_limits.py` | Configure and test safety limits |
| `example_09_trajectory_cartesian.py` | Generate and execute Cartesian trajectory |
| `example_10_shutdown_safe.py` | Graceful shutdown pattern |
| `example_11_motor_currents.py` | Motor currents at rest and during a move (Max context) |
| `example_12_contact_detection.py` | Stop a move when an idle axis's current departs from baseline (Max) |
| `example_13_override_speed.py` | Set and read back `$OV_PRO`; time the robot's response at 50 % and 100 % (Max) |
| `example_14_trajectory_queue.py` | Queue several legs, execute them, cancel one mid-run |
| `example_15_live_dataframe.py` | Live data into pandas; velocity and acceleration during a move |
| `example_16_timing_diagnostics.py` | Cycle interval, jitter, late-packet counts on this PC |
| `example_17_context_to_deploy.py` | Context → generated config → deploy folder; no robot needed |
| `example_18_rsi_lifecycle.py` | Client states, loading limits from the `.rsi.xml`, stop and `reconnect()` |

Advanced examples:

| Directory | Scripts |
|-----------|---------|
| `examples/advanced_motion/` | Velocity profiles, arcs/circles/spirals, path blending, coordinate transforms |
| `examples/coordination/` | Python-KRL handshake, parameter passing via Tech variables, state machine coordination |
