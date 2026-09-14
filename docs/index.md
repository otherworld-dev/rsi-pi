# RSIPI

RSIPI is a Python library for real-time control of KUKA industrial robots via the Robot Sensor Interface (RSI) protocol. The robot controller sends its state over UDP at a configurable cycle rate (4ms at 250Hz or 12ms at 83Hz), and RSIPI sends back position corrections, I/O commands, and Tech parameters. Communication uses XML packets over a dedicated Ethernet link, managed in a separate process so your control logic never blocks the real-time loop.

!!! danger "Safety notice"
    RSIPI directly controls industrial robot motion. Misuse can cause damage or injury.

    - **Test offline first** using the built-in echo server before connecting to a real robot.
    - **Hardware E-stops** must be present and functional. RSIPI's software E-stop is not safety-rated.
    - **Limit correction ranges** via `api.safety.set_limit()` and KUKA Workspaces.
    - **Isolate the RSI network** -- use a dedicated Ethernet interface with no external access.
    - **Never run unattended** without proper risk assessment and safety measures.

## Installation

Requires Python 3.10+.

```bash
pip install RSIPI
```

For a development checkout:

```bash
# Editable install with the test dependencies
pip install -e ".[dev]"
```

## Quick start

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
        # in the guide). For a one-shot offset instead, pass rsi_mode="absolute"
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

## Where next

| | |
|---|---|
| [Controller setup](controller-setup.md) | Getting a KRC4 talking to RSIPI: files, network, KRL, first run |
| [Configuration](configuration.md) | The six shipped contexts, `deploy` and `config_builder` |
| [RSI mode and rate limiting](rsi-mode.md) | Relative vs absolute corrections, cycle time, per-cycle clamps |
| [API guide](api/lifecycle.md) | Every namespace with worked snippets and the generated reference |
| [Testing offline](testing.md) | The echo server, dry runs and pytest |
| [Hardware findings](hardware-findings.md) | What was verified on a KR 16-2, and what bit |
| [RSI objects](rsi-objects.md) | All 74 RSI objects and how each reaches KRL |
