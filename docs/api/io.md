# I/O

Digital outputs and inputs, timed pulses, and analogue I/O on contexts that wire it.

## Usage

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

# Analogue I/O (Max context: ANIN / MAP2ANOUT objects). Raises if not wired.
value = api.io.read_analog()     # $ANIN[1]
api.io.set_analog(0.5)           # $ANOUT[1]
```

## Reference

::: RSIPI.io_api.IOAPI
