# KRL

Handshakes with the running KRL program, `$TECHPAR` and `$SEN_PINT` exchange, and KRL file tooling.

## Usage

`wait_for_signal()`/`signal_complete()` default to per-bit `Digin`/`Digout` groups,
which the shipped configs don't declare (digital I/O there is exposed only via the
`DiL`/`DiO` words). Add per-bit `Digin.i*`/`Digout.o*` groups to your RSI config's
SEND/RECEIVE sections to use the defaults below, or use `api.io.get_input()` /
`api.io.set_output()` (no `group=`) against the shipped configs instead.

```python
# Wait for KRL to set a digital input (synchronization)
if api.krl.wait_for_signal(3, timeout=10.0):
    print("KRL ready")

# Signal KRL that Python is done
api.krl.signal_complete(2)       # Sets Digout.o2 = HIGH (per-bit group required)

# Pass data to KRL via Tech.T variables (slots 11-199)
api.krl.write_param("T22", 120.0)   # KRL reads $TECHPAR[2,2]
api.krl.write_param(13, -50.0)

# Read data from KRL via Tech.C variables
force = api.krl.read_param("C11")    # KRL writes $TECHPAR_C[1,1]
actual_x = api.krl.read_param(12)

# $SEN_PINT (Max context: SEN_PINT / MAP2SEN_PINT objects) - an integer
# both sides can read and write, a cleaner handshake than a digital bit
api.krl.write_sen_pint(7)            # KRL reads $SEN_PINT[1]
n = api.krl.read_sen_pint()          # KRL wrote $SEN_PINT[1]

# Parse KRL .src/.dat files to CSV
api.krl.parse_to_csv("robot_prog.src", "robot_prog.dat", "output.csv")

# Inject RSI commands into existing KRL program
api.krl.inject_rsi("robot_prog.src", "robot_prog_rsi.src")
```

## Reference

::: RSIPI.krl_api.KRLAPI
