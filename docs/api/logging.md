# Logging

Per-cycle CSV logging of every exchanged variable, in its own process.

## Usage

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

## Reference

::: RSIPI.logging_api.LoggingAPI
