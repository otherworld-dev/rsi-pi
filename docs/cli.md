# CLI

Start the interactive command-line interface:

```bash
python -m RSIPI.rsi_cli --config RSI_EthernetConfig.xml
```

The CLI covers the common operations through text commands -- start/stop, variable set, safety stop/reset/limits, logging, trajectories, and plots (e.g. `start`, `stop`, `set <var> <value>`, `move_cartesian`, `log start`, `safety-stop`). The Python API is the full surface; use it directly for anything not exposed by the CLI.
