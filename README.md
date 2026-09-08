# RSIPI: Robot Sensor Interface for Python

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

RSIPI is a Python library for real-time control of KUKA industrial robots via the Robot Sensor Interface (RSI) protocol. The robot controller sends its state over UDP at a configurable cycle rate (4ms at 250Hz or 12ms at 83Hz), and RSIPI sends back position corrections, I/O commands, and Tech parameters. Communication uses XML packets over a dedicated Ethernet link, managed in a separate process so your control logic never blocks the real-time loop.

---

## Safety Notice

RSIPI directly controls industrial robot motion. Misuse can cause damage or injury.

- **Test offline first** using the built-in echo server before connecting to a real robot.
- **Hardware E-stops** must be present and functional. RSIPI's software E-stop is not safety-rated.
- **Limit correction ranges** via `api.safety.set_limit()` and KUKA Workspaces.
- **Isolate the RSI network** -- use a dedicated Ethernet interface with no external access.
- **Never run unattended** without proper risk assessment and safety measures.

Deploying on a real KUKA controller: see [docs/controller-setup.md](docs/controller-setup.md) for setup and [docs/hardware-findings.md](docs/hardware-findings.md) for hardware-verified behavior, protocol gotchas, and troubleshooting.

---

## Installation

Requires Python 3.10+.

```bash
# Development install (editable)
pip install -e .

# Or install dependencies directly
pip install pandas>=2.0 numpy>=1.22 matplotlib>=3.5 lxml>=4.9 scipy>=1.8
```

---

## Quick Start

```python
from RSIPI import RSIAPI, context

# context() returns a config that ships with RSIPI. "joints" is the default:
# the most capable one that works on any 6-axis robot. Pass your own path
# instead once you have a config of your own — it is a required argument
# either way, because it must match what the controller loads.
#
# The SAME context must be on the controller. List the files to copy with:
#     python -c "from RSIPI import context_files; print(*context_files(), sep='\n')"

# Context manager handles cleanup on exit
with RSIAPI(context("joints")) as api:
    api.start()

    if api.wait_for_connection(timeout=10.0):
        # Default rsi_mode="relative": this adds 10mm/cycle to X, re-applied
        # every ~4ms cycle until changed (see "RSI Mode and Rate Limiting"
        # below). For a one-shot offset instead, pass rsi_mode="absolute"
        # to RSIAPI().
        api.motion.update_cartesian(X=10.0)

        # Read current TCP position
        pose = api.motion.get_current_pose()
        print(f"TCP: X={pose['X']}, Y={pose['Y']}, Z={pose['Z']}")
    else:
        print("No robot connection within 10s")

# api.stop() called automatically
```

Without the context manager:

```python
api = RSIAPI("RSI_EthernetConfig.xml", rsi_mode="relative")
api.start()
api.wait_for_connection()

api.motion.update_cartesian(X=5.0, Y=-3.0)

api.stop()
```

---

## Configuration

RSIPI reads an RSI Ethernet config to determine network settings and which variables are exchanged with the robot. Five ship with the package — `basic`, `joints` (default), `full`, `onlysend`, `stop` — resolved by name with `context()`; `available_contexts()` lists them and `describe_contexts()` explains what each is for. There is deliberately no "everything wired up" config: the maximal one (`full`) is the *least* portable, because its external-axis objects cannot bind on a robot without external axes and the ETHERNET object reports `RSIBad` at `RSI_ON`.

```xml
<ROOT>
   <CONFIG>
      <IP_NUMBER>10.10.10.10</IP_NUMBER>   <!-- External PC IP -->
      <PORT>64000</PORT>                    <!-- UDP port -->
      <SENTYPE>ImFree</SENTYPE>             <!-- XML root element name -->
      <ONLYSEND>FALSE</ONLYSEND>            <!-- FALSE = bidirectional -->
   </CONFIG>

   <!-- SEND: What the robot sends TO us (read-only from Python) -->
   <SEND>
      <ELEMENTS>
         <ELEMENT TAG="DEF_RIst" TYPE="DOUBLE" INDX="INTERNAL" />  <!-- TCP position -->
         <ELEMENT TAG="DEF_RSol" TYPE="DOUBLE" INDX="INTERNAL" />  <!-- Commanded position -->
         <ELEMENT TAG="DEF_Delay" TYPE="LONG" INDX="INTERNAL" />   <!-- Packet delay count -->
         <ELEMENT TAG="Digout.o1" TYPE="BOOL" INDX="2" />          <!-- Digital output state -->
      </ELEMENTS>
   </SEND>

   <!-- RECEIVE: What the robot receives FROM us (writable from Python) -->
   <RECEIVE>
      <ELEMENTS>
         <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" HOLDON="1" />  <!-- Cartesian correction -->
         <ELEMENT TAG="RKorr.Y" TYPE="DOUBLE" INDX="2" HOLDON="1" />
         <ELEMENT TAG="RKorr.Z" TYPE="DOUBLE" INDX="3" HOLDON="1" />
         <ELEMENT TAG="RKorr.A" TYPE="DOUBLE" INDX="4" HOLDON="1" />
         <ELEMENT TAG="RKorr.B" TYPE="DOUBLE" INDX="5" HOLDON="1" />
         <ELEMENT TAG="RKorr.C" TYPE="DOUBLE" INDX="6" HOLDON="1" />
         <ELEMENT TAG="DiO" TYPE="LONG" INDX="8" HOLDON="1" />        <!-- Digital I/O -->
      </ELEMENTS>
   </RECEIVE>
</ROOT>
```

*(This is a trimmed illustrative excerpt, not a literal copy of the shipped file --
`RSI_EthernetConfig.xml` also declares `DiL`, `Tech.C1`, `Digout.o2`/`o3`, and `Source1`
in SEND, and `EStr`, `Tech.T2`, and `FREE` in RECEIVE.)*

Key points:
- `DEF_` prefixed tags are expanded internally (e.g., `DEF_RIst` becomes `RIst: {X, Y, Z, A, B, C}`).
- `HOLDON="1"` means the last value is held if no new value is sent.
- SEND variables are read via `api.monitoring` (position/force/IPOC), `api.krl.read_param()` (Tech.C), and `api.io.get_input()` (digital inputs); RECEIVE variables are written via `api.motion`, `api.io`, and `api.krl.write_param()`.
- The config must match the RSI object configuration on the KUKA controller.

---

## API Reference

### Core Lifecycle

```python
api = RSIAPI(
    config_file="RSI_EthernetConfig.xml",
    rsi_mode="relative",        # "absolute" or "relative" -- must match KRL
    max_cartesian_rate=0.5,     # Max mm/cycle for RKorr (0 = unlimited)
    max_joint_rate=0.1,         # Max deg/cycle for AKorr (0 = unlimited)
    cycle_time=0.004            # 0.004 = 4ms/250Hz, 0.012 = 12ms/83Hz
)

api.start()                     # Start UDP listener in background thread
api.wait_for_connection(10.0)   # Block until first robot packet (returns bool)
api.is_running()                # Check if communication is active
api.stop()                      # Graceful shutdown
api.reconnect()                 # Restart network with fresh resources
```

### `api.motion` -- Motion Control

```python
# Cartesian corrections (RKorr) -- mm for XYZ, degrees for ABC
api.motion.update_cartesian(X=10.0, Y=-5.0, Z=0.0)
api.motion.update_cartesian(A=2.5, B=0.0, C=0.0)

# Joint corrections (AKorr) -- degrees
api.motion.update_joints(A1=5.0, A2=-3.0)

# Read current state
pose = api.motion.get_current_pose()      # {X, Y, Z, A, B, C}
joints = api.motion.get_current_joints()   # {A1, A2, A3, A4, A5, A6}

# External axes (value is a per-cycle delta under rsi_mode="relative" -- see Core Lifecycle)
api.motion.move_external_axis("E1", 2.5)

# Tech parameters (runtime motion adjustment)
api.motion.adjust_speed("Tech.T21", 0.5)
```

#### Trajectories

```python
# Generate linear trajectory
traj = api.motion.generate_trajectory(
    start={"X": 0, "Y": 0, "Z": 500},
    end={"X": 100, "Y": 0, "Z": 500},
    steps=50,
    space="cartesian"
)

# Execute (blocking). cycles_per_step paces one waypoint per N robot cycles
# (3 * 4ms default cycle_time = one waypoint every 12ms here). `rate=`
# (seconds/waypoint) still works but is deprecated in favor of cycles_per_step.
api.motion.execute_trajectory(traj, space="cartesian", cycles_per_step=3)

# Or generate + execute in one call (end_pose first, start_pose defaults to current position)
api.motion.move_cartesian_trajectory(
    end_pose={"X": 100, "Y": 0, "Z": 500},
    start_pose={"X": 0, "Y": 0, "Z": 500},
    steps=50, cycles_per_step=5
)
api.motion.move_joint_trajectory(
    end_joints={"A1": 30, "A2": -15, "A3": 45, "A4": 0, "A5": 30, "A6": 0},
    start_joints={"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0},
    steps=100, cycles_per_step=100
)

# Cancel a running trajectory from another thread
api.motion.cancel_trajectory()
```

#### Trajectory Queue

```python
api.motion.queue_cartesian_trajectory(p0, p1, steps=50)
api.motion.queue_cartesian_trajectory(p1, p2, steps=50)
api.motion.queue_joint_trajectory(j0, j1, steps=100, rate=0.4)

print(api.motion.get_queue())       # Metadata for queued items
api.motion.execute_queued_trajectories()  # Run all in sequence
api.motion.clear_queue()            # Discard without executing
```

#### Geometric Primitives

```python
# Circular arc
arc = api.motion.generate_arc(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=50.0,
    start_angle=0, end_angle=90,
    steps=50, plane="XY"
)

# Full circle
circle = api.motion.generate_circle(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=50.0, steps=100, plane="XY"
)

# Spiral
spiral = api.motion.generate_spiral(
    center={"X": 100, "Y": 0, "Z": 500},
    start_radius=10.0, end_radius=50.0,
    pitch=5.0, revolutions=5,
    steps=200, plane="XY", axis="Z"
)

api.motion.execute_trajectory(arc, space="cartesian")
```

#### Velocity Profiles

```python
traj = api.motion.generate_trajectory(p0, p1, steps=100)

# Trapezoidal (bang-bang acceleration)
profiled = api.motion.generate_velocity_profile(
    traj, max_velocity=200.0, max_acceleration=500.0,
    profile="trapezoidal"
)

# S-curve (jerk-limited, smoother)
profiled = api.motion.generate_velocity_profile(
    traj, max_velocity=200.0, max_acceleration=500.0,
    profile="s-curve"
)

# Each element is (waypoint_dict, velocity_float)
for waypoint, velocity in profiled:
    print(f"Velocity: {velocity:.2f} mm/s")
```

#### Path Blending

```python
traj1 = api.motion.generate_trajectory(p0, p1, 50)
traj2 = api.motion.generate_trajectory(p1, p2, 50)

blended = api.motion.blend_trajectories(
    traj1, traj2,
    blend_radius=10.0,   # mm from junction
    blend_steps=20
)
api.motion.execute_trajectory(blended)
```

#### Coordinate Transforms

```python
world_pose = api.motion.transform_coordinates(
    pose={"X": 100, "Y": 0, "Z": 500},
    from_frame="BASE", to_frame="WORLD",
    frame_offset={"X": 500, "Y": 200, "Z": 0}
)
```

### `api.io` -- Digital I/O

```python
# Set output by channel number
api.io.set_output(1, True)       # sets bit 0 of the DiO word (Digout.o1 is SEND-only readback in the shipped configs)
api.io.set_output(3, False)      # clears bit 2 of the DiO word

# Generic toggle (any per-bit group declared writable in RECEIVE)
api.io.toggle("MyOutputs", "o1", True)

# Read input (Digin.i1 if declared, else bit 0 of the DiL word)
if api.io.get_input(1):
    print("Sensor triggered")

# Timed pulse (blocking)
api.io.pulse(2, duration=0.1)    # 100ms pulse on output 2
```

### `api.krl` -- KRL Coordination

`wait_for_signal()`/`signal_complete()` default to per-bit `Digin`/`Digout` groups,
which the shipped configs don't declare (digital I/O there is exposed only via the
`DiL`/`DiO` words). Add per-bit `Digin.i*`/`Digout.o*` groups to your RSI config's
SEND/RECEIVE sections to use the defaults below, or use `api.io.get_input()` /
`api.io.set_output()` (no `group=`) against the shipped configs instead.

```python
# Wait for KRL to set a digital input (synchronization)
if api.krl.wait_for_signal(3, timeout=10.0):
    print("KRL ready")

# Signal KRL that Python is done
api.krl.signal_complete(2)       # Sets Digout.o2 = HIGH (per-bit group required)

# Pass data to KRL via Tech.T variables (slots 11-199)
api.krl.write_param("T22", 120.0)   # KRL reads $TECHPAR[2,2]
api.krl.write_param(13, -50.0)

# Read data from KRL via Tech.C variables
force = api.krl.read_param("C11")    # KRL writes $TECHPAR_C[1,1]
actual_x = api.krl.read_param(12)

# Parse KRL .src/.dat files to CSV
api.krl.parse_to_csv("robot_prog.src", "robot_prog.dat", "output.csv")

# Inject RSI commands into existing KRL program
api.krl.inject_rsi("robot_prog.src", "robot_prog_rsi.src")
```

### `api.safety` -- Safety Management

```python
# Emergency stop (software-level, NOT safety-rated)
api.safety.stop()
api.safety.is_stopped()          # True

# Reset E-stop
api.safety.reset()

# Configure correction limits
api.safety.set_limit("RKorr.X", -50.0, 50.0)
api.safety.set_limit("AKorr.A1", -10.0, 10.0)

# View all limits
limits = api.safety.get_limits()
for var, (lo, hi) in limits.items():
    print(f"{var}: [{lo}, {hi}]")

# Full status
status = api.safety.status()
# {"emergency_stop": False, "safety_override": False, "limits": {...}}

# Override limits (use with extreme caution)
api.safety.override(True)
# ... calibration work ...
api.safety.override(False)
```

### `api.monitoring` -- Live Data

```python
# Comprehensive snapshot
data = api.monitoring.get_live_data()
# {"position": {X,Y,Z,A,B,C}, "velocity": {...}, "force": {...}, "ipoc": 123456}

# Individual reads
pos = api.monitoring.get_position()      # {X, Y, Z, A, B, C}
force = api.monitoring.get_force()       # {A1, A2, A3, A4, A5, A6} motor currents
ipoc = api.monitoring.get_ipoc()         # Interrupt point counter

# NumPy/Pandas formats
arr = api.monitoring.get_live_data_as_numpy()        # shape (4, 6)
df = api.monitoring.get_live_data_as_dataframe()     # single-row DataFrame

# Console watch (blocking, Ctrl+C to stop)
api.monitoring.watch_network(duration=10, rate=0.2)
```

### `api.diagnostics` -- Network Health

```python
stats = api.diagnostics.get_stats()
timing = api.diagnostics.get_timing()      # cycle time, jitter
quality = api.diagnostics.get_network_quality()  # packet loss, IPOC gaps

if not api.diagnostics.is_healthy():
    for w in api.diagnostics.get_warnings():
        print(f"Warning: {w}")

if api.diagnostics.check_watchdog():
    api.reconnect()

print(api.diagnostics.format_stats())
# Network Diagnostics:
#   Cycle Time: 4.01ms (+/-0.12ms jitter)
#   Packet Loss: 0.05%
#   ...
```

### `api.logging` -- CSV Logging

```python
# Start logging (auto-generates filename in logs/)
path = api.logging.start()
print(path)  # logs/17-04-2026_14-32-45.csv

# Or specify filename
api.logging.start("my_experiment.csv")

api.logging.is_active()   # True
api.logging.stop()
```

Logs include British-format timestamps, all send/receive variables per cycle. Logging runs in a separate process to avoid interfering with the 4 ms control loop.

### `api.viz` -- Visualization

```python
# Static plots from CSV logs
api.viz.plot_static("logs/test.csv", "3d")
api.viz.plot_static("logs/test.csv", "position")    # Position vs time
api.viz.plot_static("logs/test.csv", "velocity")
api.viz.plot_static("logs/test.csv", "joints")
api.viz.plot_static("logs/test.csv", "force")
api.viz.plot_static("logs/test.csv", "2d_xy")       # 2D projections

# Deviation from planned path
api.viz.plot_static("logs/actual.csv", "deviation", overlay_path="logs/planned.csv")

# Comprehensive multi-plot visualization
api.viz.visualize_csv_log("logs/test.csv")
api.viz.visualize_csv_log("logs/test.csv", export=True)  # Save to disk

# Compare two runs
api.viz.compare_runs("run1.csv", "run2.csv")

# Live plotting (runs in background thread)
api.viz.start_live_plot("3d", interval=100)     # 100ms update
api.viz.change_live_plot_mode("position")
api.viz.stop_live_plot()
```

### `api.tools` -- Utilities

```python
# Low-level variable access
api.tools.update_variable("RKorr.X", 10.0)
api.tools.show_variables()     # Print all available variables
api.tools.show_config()        # Network settings + variable structure
api.tools.reset_variables()    # Zero out corrections

# Reports and comparison
api.tools.generate_report("logs/test.csv", "pdf")
diffs = api.tools.compare_runs("run1.csv", "run2.csv")
```

---

## RSI Mode and Rate Limiting

### Absolute vs Relative Mode

The `rsi_mode` parameter must match what your KRL program uses with `RSI_MOVECORR()`:

| Mode | Behavior | Use Case |
|------|----------|----------|
| `"relative"` | Corrections are **added** to the programmed path each cycle. Sending `X=1.0` every cycle moves 1mm/cycle continuously. | Continuous adjustments, sensor feedback |
| `"absolute"` | Corrections specify **total offset** from programmed path. Sending `X=10.0` holds 10mm offset regardless of how many cycles. | Target position offsets |

### Cycle Time

KUKA RSI supports two cycle rates, configured on the robot controller side. RSIPI's network loop is reactive (it responds to whatever the robot sends), but the `cycle_time` parameter ensures diagnostics, health checks, and jitter warnings use the correct baseline:

```python
# 4ms cycle / 250Hz (default)
api = RSIAPI("RSI_EthernetConfig.xml", cycle_time=0.004)

# 12ms cycle / 83Hz
api = RSIAPI("RSI_EthernetConfig.xml", cycle_time=0.012)
```

| Cycle Time | Frequency | Use Case |
|------------|-----------|----------|
| `0.004` (4ms) | 250 Hz | High-frequency corrections, sensor feedback loops |
| `0.012` (12ms) | 83 Hz | Standard motion corrections, less demanding applications |

### Rate Limiting

Rate limiting caps the per-cycle change to prevent sudden jumps:

```python
api = RSIAPI(
    "RSI_EthernetConfig.xml",
    rsi_mode="relative",
    max_cartesian_rate=0.5,   # Max 0.5 mm per cycle
    max_joint_rate=0.1,       # Max 0.1 deg per cycle
    cycle_time=0.004          # 4ms cycle
)
```

Set rates to `0.0` (default) to disable rate limiting. Clamping is applied in the network process right before the response is sent to the robot.

---

## Testing

### Echo Server

RSIPI includes an echo server that simulates a KUKA controller for offline development:

```bash
python -m RSIPI.rsi_echo_server
```

The echo server binds to the same UDP port as a real robot, sends XML state packets at 250 Hz, and accepts correction responses. Use it to test your control logic without hardware.

### Running with pytest

```bash
pip install -e ".[dev]"
pytest
```

Test files go in the `tests/` directory. The project uses `src` layout with `pythonpath = ["src"]` configured in `pyproject.toml`.

---

## Architecture

```
KUKA Robot Controller
        |
     UDP/XML (4ms cycle)
        |
   NetworkProcess          <- multiprocessing.Process, owns the socket
        |
   Manager().dict()        <- shared send_variables / receive_variables
        |
     RSIClient             <- orchestrator: config, safety, network
        |
      RSIAPI               <- runs RSIClient in daemon thread
     /  |  \  \
motion  io  krl  safety  monitoring  logging  viz  diagnostics  tools
```

- **NetworkProcess** runs in a separate OS process. It receives XML from the robot, parses it into `send_variables` (what the robot tells us), and builds the response XML from `receive_variables` (what we tell the robot). IPOC synchronization -- echoing the robot's IPOC value unchanged each cycle (the robot advances its own clock) -- is handled automatically.
- **RSIClient** creates the `multiprocessing.Manager` dicts for cross-process variable sharing, initializes the `ConfigParser` and `SafetyManager`, and manages the network process lifecycle.
- **RSIAPI** wraps RSIClient in a daemon thread and exposes the namespaced sub-APIs (`motion`, `io`, `krl`, etc.).

The 4 ms cycle is driven by the robot controller, not by RSIPI. If a response is not sent within the cycle window, the robot uses the last held values (for `HOLDON="1"` variables) or drops to zero.

---

## Examples

The `examples/` directory contains runnable scripts:

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

Advanced examples:

| Directory | Scripts |
|-----------|---------|
| `examples/advanced_motion/` | Velocity profiles, arcs/circles/spirals, path blending, coordinate transforms |
| `examples/coordination/` | Python-KRL handshake, parameter passing via Tech variables, state machine coordination |

---

## CLI

Start the interactive command-line interface:

```bash
python -m RSIPI.rsi_cli --config RSI_EthernetConfig.xml
```

The CLI covers the common operations through text commands -- start/stop, variable set, safety stop/reset/limits, logging, trajectories, and plots (e.g. `start`, `stop`, `set <var> <value>`, `move_cartesian`, `log start`, `safety-stop`). The Python API is the full surface; use it directly for anything not exposed by the CLI.

---

## How to Cite

If you use RSIPI in academic work, please cite it:

```bibtex
@software{morgan_rsipi,
  author  = {Morgan, Adam},
  title   = {{RSIPI}: Robot Sensor Interface for Python},
  year    = {2026},
  url     = {https://github.com/otherworld-dev/rsi-pi},
  version = {0.1.1}
}
```

Or in plain text:

> Morgan, A. (2026). *RSIPI: Robot Sensor Interface for Python* (Version 0.1.1) [Computer software]. https://github.com/otherworld-dev/rsi-pi

A [CITATION.cff](CITATION.cff) file is included, so you can also use GitHub's
**Cite this repository** button for APA/BibTeX output.

---

## Support This Project

RSIPI is developed and maintained in spare time. If it saves you hours of
KUKA head-scratching, consider supporting development:

- [GitHub Sponsors](https://github.com/sponsors/otherworld-dev)
- [PayPal](https://www.paypal.com/donate/?hosted_button_id=MA56N6K8FSTQ2)

Bug reports, docs improvements, and pull requests are equally appreciated.

---

## License

[Apache License 2.0](LICENSE)
