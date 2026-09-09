# RSIPI Example Scripts

All commands below assume the repo venv and are run from the repo root:

```
.venv\Scripts\python.exe examples\<script>.py <config>
```

Everything here runs against the echo server with no robot attached, except
where a script is explicitly a hardware bring-up tool.

## Feature demonstrations

One feature each, minimal and readable.

| Example | Description |
|:--------|:------------|
| `example_01_start_stop.py` | Start and stop RSI communication |
| `example_02_send_cartesian.py` | Move the robot TCP |
| `example_03_send_joint.py` | Move robot joints |
| `example_04_external_axes.py` | Move external axes |
| `example_05_digital_io.py` | Write digital outputs |
| `example_06_logging_csv.py` | Record robot data to CSV |
| `example_07_graphing_live.py` | Live plot robot movements |
| `example_08_safety_limits.py` | Apply and enforce motion limits |
| `example_09_trajectory_cartesian.py` | Execute simple Cartesian path |
| `example_10_shutdown_safe.py` | Safe shutdown with emergency handling |

Two subfolders extend these:

- **`advanced_motion/`** — velocity profiles, geometric primitives
  (arcs/circles/spirals), path blending, coordinate transforms, and a
  combined-motion script.
- **`coordination/`** — Python↔KRL handshake patterns: a basic handshake,
  parameter passing over `Tech` variables, and a KRL state machine driven
  from Python. Each has a matching `.src` template in `controller/Program/`.

## Bring-up and verification tools

Built for getting RSIPI onto a real controller. Results from an actual
KR 16-2 are in [docs/hardware-findings.md](../docs/hardware-findings.md).

| Script | Proves |
|:-------|:-------|
| `validate_context.py` | **Run this before copying anything to the controller.** Checks offline that a context and its config agree on channel numbering — catches `RSI_CREATE: Invalid index - signal output` |
| `udp_probe.py` | Raw UDP listener — that packets physically reach the PC, independent of RSIPI |
| `first_contact.py` | Three gated stages: connect-only, a small paced move, an E-stop drill |
| `kuka_example_server.py` | Drop-in replacement for KUKA's `TestServer.exe`; packet monitoring, optional `--jog`/`--joints`, automatic DiO read-back |
| `feature_test.py` | Staged session: diagnostics, I/O, trajectory accuracy, velocity profiles, safety limits, E-stop, CSV + report, reconnect. Prints PASS/FAIL |
| `onlysend_monitor.py` | ONLYSEND one-way streaming (pair with `RSIPI_OnlySend.src`). Refuses to run against a non-ONLYSEND config so it cannot pass while quietly replying |
| `stop_test.py` | Whether `RSI_MOVECORR()` can be ended from the PC (pair with `RSIPI_Stop.src`). Measures a real move **first**, because the original failure was indistinguishable from success at the network level |
| `max_test.py` | Acceptance test for `RSIPI_Max` (pair with `RSIPI_Max.src`): override, applied-correction monitors, motor currents, analogue I/O, `$SEN_PINT`, clamping, the 16-bit output word |
| `rsipi_test.py` | Full acceptance test paired with `RSIPI_Test.src`: corrections, `$SEN_PREA`, digital I/O, `Tech.C`/`Tech.T` handshake |
| `dry_run.py` | **Run before a lab session.** Runs any script in this table against the emulated controller with prompts auto-answered, proving it runs end to end. Proves no *result*: the emulator applies no limits, has no `RSI_MOVECORR` to end, and binds no objects |
| `stability_test.py` | Long-duration soak (`--duration` in hours). A standalone tool, not a pytest test — which is why it lives here |

## Testing offline

The echo server plays the robot side of the link, so every script above can
be exercised with no hardware:

```
python -m RSIPI.rsi_echo_server --config RSI_EthernetConfig.xml
```

It owns the IPOC clock, validates replies, emulates `HOLDON` late-cycle
behaviour, counts faulty packets and breaks off past the `Timeout` budget.
Useful flags: `--mode absolute|relative`, `--rsi-file <context.rsi>`,
`--onlysend`.

There is also a GUI front end for it, if you prefer watching to reading:

```
python -m RSIPI.echo_server_gui
```

It wraps the same `EchoServer` with a tkinter control panel and a 3D plot of
the emulated robot's pose (needs `tkinter`, `matplotlib` and `numpy`).
