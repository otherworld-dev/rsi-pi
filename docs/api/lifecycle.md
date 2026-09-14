# Core lifecycle

`RSIAPI` is the entry point: it owns the client, the network process and the namespaces on the other API pages.

## Usage

```python
api = RSIAPI(
    config_file=context("joints"),  # required: a shipped context, or the path to your own config
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

## Reference

::: RSIPI.rsi_api.RSIAPI
