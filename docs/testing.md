# Testing offline

### Echo Server

RSIPI includes an echo server that plays the robot side of the link for offline development:

```bash
python -m RSIPI.rsi_echo_server                                  # the default config
python -m RSIPI.rsi_echo_server --config path/to/RSI_EthernetConfig_Max.xml
python -m RSIPI.rsi_echo_server --onlysend                        # stream, expect no replies
```

It binds UDP port 50000, sends state packets at the 4 ms cycle, integrates the corrections it receives into the position it reports, and breaks off after the ETHERNET `Timeout` budget of faulty packets like a real controller. A reply that misses its cycle is dropped, and the server reads past it to the current reply. A stall costs the cycles it missed, not the session. Silence before a client attaches is never counted. Neither is silence longer than a controller tolerates (`Timeout` cycles): a real robot would break off there, but the server treats it as the client restarting. It does not model dynamics, motor current or correction clamping — it proves the plumbing, not the physics.

[`examples/dry_run.py`](https://github.com/otherworld-dev/rsi-pi/blob/main/examples/dry_run.py)` <script>` wraps this: it starts the echo server for the script's context, answers every confirmation prompt automatically, runs the script, and reports whether it ran to completion or raised.

## Running with pytest

```bash
pip install -e ".[dev]"
pytest
```

Test files go in the `tests/` directory. The project uses `src` layout with `pythonpath = ["src"]` configured in `pyproject.toml`. The loopback tests run a real `RSIClient` against the echo server over UDP and need port 50000 free — an orphaned echo server from an earlier run shows up as spurious failures.

A soak test streams for minutes against the echo server, freezing the client past a reply window at intervals. It is skipped unless you give it a duration:

```bash
RSIPI_SOAK_SECONDS=600 pytest -m soak -s
```
