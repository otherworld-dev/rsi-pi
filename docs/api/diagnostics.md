# Diagnostics

Cycle timing, jitter, packet loss, watchdog and health checks on the UDP link.

## Usage

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

## Reference

::: RSIPI.diagnostics_api.DiagnosticsAPI
