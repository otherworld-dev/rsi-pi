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

The `Timestamp` column is wall-clock time taken when the row is written, so it can lag the packet by however long the row sat in the queue. The `RxTimeNs` column is `time.perf_counter_ns()` taken the moment the packet arrived. On Windows this is the system-wide performance counter, so it can be compared directly with `perf_counter_ns()` stamps from other processes, for example a DAQ or camera logger running alongside, and used with `Send.IPOC` to line robot data up with them.

## Reference

::: RSIPI.logging_api.LoggingAPI
