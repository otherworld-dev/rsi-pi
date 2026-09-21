# RSIPI: Robot Sensor Interface for Python

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)
[![Docs](https://img.shields.io/badge/docs-rsipi.otherworld.dev-orange.svg)](https://rsipi.otherworld.dev/)

RSIPI is a Python library for real-time control of KUKA industrial robots via the Robot Sensor Interface (RSI) protocol. The robot controller sends its state over UDP at a configurable cycle rate (4ms at 250Hz or 12ms at 83Hz), and RSIPI sends back position corrections, I/O commands, and Tech parameters. Communication uses XML packets over a dedicated Ethernet link, managed in a separate process so your control logic never blocks the real-time loop.

Full documentation: **https://rsipi.otherworld.dev/**

---

## Safety Notice

RSIPI directly controls industrial robot motion. Misuse can cause damage or injury.

- **Test offline first** using the built-in echo server before connecting to a real robot.
- **Hardware E-stops** must be present and functional. RSIPI's software E-stop is not safety-rated.
- **Limit correction ranges** via `api.safety.set_limit()` and KUKA Workspaces.
- **Isolate the RSI network** -- use a dedicated Ethernet interface with no external access.
- **Never run unattended** without proper risk assessment and safety measures.

Deploying on a real KUKA controller: start with the [controller setup guide](https://rsipi.otherworld.dev/controller-setup/), then read the [hardware findings](https://rsipi.otherworld.dev/hardware-findings/) for what was verified on a real robot and what bit.

---

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

---

## Documentation

| Page | What it covers |
|------|----------------|
| [Controller setup](https://rsipi.otherworld.dev/controller-setup/) | Getting a KRC4 talking to RSIPI: files, network, KRL, first run |
| [Configuration](https://rsipi.otherworld.dev/configuration/) | The six shipped contexts, `deploy` and `config_builder` |
| [RSI mode and rate limiting](https://rsipi.otherworld.dev/rsi-mode/) | Relative vs absolute corrections, cycle time, per-cycle clamps |
| [API guide](https://rsipi.otherworld.dev/api/lifecycle/) | Every namespace with worked snippets and the generated reference |
| [Testing offline](https://rsipi.otherworld.dev/testing/) | The echo server, dry runs and pytest |
| [Architecture](https://rsipi.otherworld.dev/architecture/) | Processes, shared state and the 4 ms cycle |
| [Hardware findings](https://rsipi.otherworld.dev/hardware-findings/) | What was verified on a KR 16-2, and what bit |
| [RSI objects](https://rsipi.otherworld.dev/rsi-objects/) | All 74 RSI objects and how each reaches KRL |

---

## Examples

`examples/` holds 18 numbered scripts plus advanced-motion and Python-KRL coordination sets. Every one that moves the robot asks first, and all of them run against the emulator without a robot:

```bash
python examples/dry_run.py examples/example_02_send_cartesian.py
```

The full list: [https://rsipi.otherworld.dev/examples/](https://rsipi.otherworld.dev/examples/).

---

## How to Cite

If you use RSIPI in academic work, please cite it:

```bibtex
@software{morgan_rsipi,
  author  = {Morgan, Adam},
  title   = {{RSIPI}: Robot Sensor Interface for Python},
  year    = {2026},
  url     = {https://github.com/otherworld-dev/rsi-pi},
  version = {0.3.0}
}
```

Or in plain text:

> Morgan, A. (2026). *RSIPI: Robot Sensor Interface for Python* (Version 0.3.0) [Computer software]. https://github.com/otherworld-dev/rsi-pi

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
