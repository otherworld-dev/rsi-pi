# KRL Coordination Templates

This directory contains KRL program templates demonstrating common Python-KRL coordination patterns using RSIPI.

## Available Templates

### 1. basic_handshake.src
**Simple I/O handshaking between Python and KRL**

- An external/physical signal indicates "ready" on digital input 1
  (DiL bit 0 / $IN[1]; hardware/PLC-driven, since RSIPI cannot read a
  robot output readback as an "input" channel)
- Python waits for signal, processes data
- Python signals "complete" via digital output (DiO word, bit 0)
- KRL waits for its mapped $OUT bit, then continues

**Use Case**: Basic synchronization, ensuring Python completes processing before KRL continues.

**Coordination Methods Used**:
- `api.krl.wait_for_signal(channel, timeout, group=None)`
- `api.krl.signal_complete(channel, group=None)`

### 2. parameter_passing.src
**Bidirectional numerical data exchange via Tech variables**

- KRL writes current position to Tech.C variables
- Python reads position data
- Python calculates target and writes to Tech.T variables
- KRL reads target and executes motion

**Use Case**: Passing numerical parameters (positions, forces, tolerances) between Python and KRL.

**Coordination Methods Used**:
- `api.krl.read_param(slot)` - Read from Tech.C (KRL-to-Python channel)
- `api.krl.write_param(slot, value)` - Write to Tech.T (Python-to-KRL channel)
- `api.krl.wait_for_signal(channel, timeout, group=None)`
- `api.krl.signal_complete(channel, group=None)`

### 3. state_machine.src
**Multi-state workflow with complex coordination**

Implements a 5-state machine:
1. **IDLE**: Waiting to start
2. **CALIBRATING**: Python performing calibration
3. **READY**: Calibration complete
4. **EXECUTING**: Robot motion in progress
5. **COMPLETE**: Task finished
6. **ERROR**: Error handling state

**Use Case**: Complex workflows requiring multiple handshakes, error handling, and state tracking.

**Coordination Methods Used**:
- All coordination methods from basic_handshake and parameter_passing
- State variable in Tech.C[11] (KRL-to-Python)
- Command variable in Tech.T[11] (Python-to-KRL)

## Python-KRL Coordination Patterns

### Pattern 1: Simple Handshake

```python
# Python side
# group=None auto-detects the DiL word for the ready signal, and the
# DiO word for the completion signal - the hardcoded defaults
# (group='Digin' / group='Digout') don't exist / aren't writable in
# either shipped config and raise immediately.
api.krl.wait_for_signal(1, group=None)  # Wait for external ready signal (DiL bit 0 / $IN[1])
# Do processing...
api.krl.signal_complete(1, group=None)  # Signal KRL to continue (DiO bit 0)
```

```krl
; KRL side
; Readiness comes from an external device (PLC/sensor) wired to $IN[1] -
; RSIPI cannot read a robot output readback (Digout.o1) as an "input"
; channel, so KRL cannot signal readiness via its own $OUT[] assignment.
; Wait for Python's completion signal on the $OUT bit RSI's DiO word
; maps to via MAP2DIGOUT (e.g. $OUT[20] per RSI_EthernetConfig_Full.xml;
; adjust to match your own .rsi mapping).
WHILE $OUT[20] == FALSE
  WAIT SEC 0.1
ENDWHILE
```

### Pattern 2: Parameter Exchange

```python
# Python side
api.krl.wait_for_signal(1, group=None)

# Read from KRL (Tech.C - KRL writes, Python reads)
value = api.krl.read_param('C11')

# Process and write back (Tech.T - Python writes, KRL reads)
result = process(value)
api.krl.write_param('T11', result)

api.krl.signal_complete(1, group=None)
```

```krl
; KRL side
$TECH.C[11] = some_value
$OUT[1] = TRUE  ; Signal data ready

; Wait for Python
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE

; Read result
result = $TECH.T[11]
```

### Pattern 3: Continuous Monitoring

```python
# Python side - non-blocking monitoring loop
api.start()

while api.is_running():
    state = api.krl.read_param('C11')  # Tech.C - KRL writes, Python reads

    if state == 1:  # Specific state
        # React to state change
        api.krl.write_param('T11', calculated_value)  # Tech.T - Python writes
        api.krl.signal_complete(1, group=None)

    time.sleep(0.1)  # Check every 100ms

api.stop()
```

```krl
; KRL side - updates state continuously
$TECH.C[11] = current_state

; Wait for Python response when needed
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE

calculated = $TECH.T[11]
```

## Tech Variable Conventions

Each declared `DEF_Tech.Cn` / `DEF_Tech.Tn` generator (n = 1-6) expands to
exactly 10 slots, `Cn1..Cn10` / `Tn1..Tn10` (see `config_parser.py`'s
`internal_structure`) - not a continuous `[11-199]` range. A slot only
exists if its generator is declared on the matching side of the RSI XML:
the default `RSI_EthernetConfig.xml` declares only `Tech.C1` (`<SEND>`,
giving `C11..C110`) and `Tech.T2` (`<RECEIVE>`, giving `T21..T210`);
`RSI_EthernetConfig_Full.xml` declares all six generators both ways.

### Tech.C Variables (KRL → Python)
**"Control" variables - KRL writes, Python reads with `read_param()`**

| Slot (generator 1) | Description | Example Usage |
|------|-------------|---------------|
| C11  | Command/state | 0=continue, 1=pause, 2=abort |
| C12-C14 | Position offsets | X, Y, Z corrections |
| C15-C17 | Target position | Calculated target coordinates |
| C18-C110 | Process parameters | Speed, force, tolerance, echoes |
| C21-C210, C31-C310, ... | Additional generators | Only if Tech.C2-C6 are declared |

```krl
; KRL writes
$TECH.C[11] = command
$TECH.C[12] = offset_x
$TECH.C[13] = offset_y
```

```python
# Python reads
command = api.krl.read_param('C11')
offset_x = api.krl.read_param('C12')
offset_y = api.krl.read_param('C13')
```

### Tech.T Variables (Python → KRL)
**"Transfer" variables - Python writes with `write_param()`, KRL reads**

| Slot (generator 2 - the one the default config declares) | Description | Example Usage |
|------|-------------|---------------|
| T21  | Current state / command | State machine state number |
| T22-T24 | Current position | X, Y, Z coordinates |
| T25-T27 | Force/torque | Measured forces |
| T28-T210 | Sensor readings | Application-specific values |
| T11-T110, T31-T310, ... | Additional generators | Only if Tech.T1, T3-T6 are declared |

```python
# Python writes
api.krl.write_param('T21', state)
api.krl.write_param('T22', pos_x)
api.krl.write_param('T23', pos_y)
```

```krl
; KRL reads
state = $TECH.T[21]
pos_x = $TECH.T[22]
pos_y = $TECH.T[23]
```

## I/O Signal Conventions

### Standard I/O Mapping

| Signal | Type | Purpose |
|--------|------|---------|
| $OUT[1] | Output | KRL → Python state/ready signal |
| $IN[1]  | Input  | Python → KRL completion acknowledgement |
| $IN[2]  | Input  | Python → KRL error signal |
| $OUT[2] | Output | KRL → Python auxiliary signal |

```python
# Python I/O methods. group=None is required on both calls - the
# hardcoded defaults (group='Digin' / group='Digout') don't exist /
# aren't writable in either shipped config and raise immediately.
api.krl.wait_for_signal(1, group=None)  # Waits on the DiL word (bit 0)
api.krl.signal_complete(1, group=None)  # Sets the DiO word (bit 0)
api.io.set_output(2, True)  # Control output 2 (DiO word, group=None default)
```

## Error Handling Best Practices

### Timeouts

**Always use timeouts to prevent indefinite blocking:**

```python
# Python
if not api.krl.wait_for_signal(1, timeout=10.0, group=None):
    print("Timeout waiting for KRL!")
    # Handle error
```

```krl
; KRL
INT counter
counter = 0
WHILE ($IN[1] == FALSE) AND (counter < 100)
  WAIT SEC 0.1
  counter = counter + 1
ENDWHILE

IF counter >= 100 THEN
  ; Timeout - handle error
  HALT
ENDIF
```

### Error Signaling

**Use dedicated error channels:**

```python
# Python detects error
if error_condition:
    api.io.set_output(2, True)  # Signal error to KRL
```

```krl
; KRL checks for errors
IF $IN[2] == TRUE THEN
  ; Python signaled error
  HALT
ENDIF
```

## Integration with RSI Motion Control

All coordination patterns work seamlessly with RSI real-time motion corrections:

```python
# Python coordinates with KRL AND sends real-time corrections
api.start()

# Wait for KRL to start motion phase
api.krl.wait_for_signal(1, group=None)

# Send real-time corrections during KRL motion
for i in range(100):
    correction = calculate_correction()
    api.motion.update_cartesian(X=correction)
    time.sleep(0.004)  # 250Hz update rate

# Signal motion phase complete
api.krl.signal_complete(1, group=None)

api.stop()
```

```krl
; KRL executes motion while Python sends corrections
$OUT[1] = TRUE  ; Signal motion start

; Python sends corrections via RSI during this move
LIN target_pos Vel=0.5 m/s CPDAT1 Tool[1] Base[0]

; Wait for Python to finish corrections
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE
```

## Testing Templates

To test these templates:

1. **Upload KRL program to robot controller**
2. **Start Python coordination script**
3. **Execute KRL program on teach pendant**
4. **Monitor coordination in Python logs**

Example Python test script:

```python
from RSIPI import RSIAPI
import time

api = RSIAPI('RSI_EthernetConfig.xml')
api.start()

try:
    print("Waiting for KRL ready signal...")

    if api.krl.wait_for_signal(1, timeout=30.0, group=None):
        print("✅ KRL signaled ready!")

        # Simulate processing
        time.sleep(2.0)
        print("Processing complete")

        # Signal back to KRL
        api.krl.signal_complete(1, group=None)
        print("✅ Signaled KRL to continue")

    else:
        print("❌ Timeout waiting for KRL")

except KeyboardInterrupt:
    print("\n⚠️  Interrupted by user")

finally:
    api.stop()
    print("API stopped")
```

## Troubleshooting

The RSI XML parser (`config_parser.py`) expects `SEND/ELEMENTS` and
`RECEIVE/ELEMENTS` containers with `<ELEMENT TAG="..." TYPE="..." INDX="..."/>`
children - see the shipped `RSI_EthernetConfig.xml` / `RSI_EthernetConfig_Full.xml`
for the real schema. A snippet using a different schema (e.g. `<XML><ELEMENT
Tag="..." .../></XML>`) is silently ignored by the parser.

### Signal Not Received

**Check I/O configuration in RSI XML** (DiL/DiO word notation, as shipped
in both configs - and remember `wait_for_signal()`/`signal_complete()`
need `group=None` to auto-detect these; the hardcoded `group='Digin'` /
`group='Digout'` defaults don't exist / aren't writable):
```xml
<SEND>
  <ELEMENTS>
    <ELEMENT TAG="DiL" TYPE="LONG" INDX="1" />
  </ELEMENTS>
</SEND>
<RECEIVE>
  <ELEMENTS>
    <ELEMENT TAG="DiO" TYPE="LONG" INDX="8" HOLDON="1" />
  </ELEMENTS>
</RECEIVE>
```

### Tech Variable Not Found

**Symptom**: `RSIVariableError: Tech.Cxx not found in send_variables` (from
`read_param()`) or `RSIVariableError: Tech.Txx not found in receive_variables`
(from `write_param()`). This happens if you call `read_param()`/`write_param()`
with the letter reversed (Tech.C lives in `send_variables`, Tech.T lives in
`receive_variables` - never the other way round), or if the slot's generator
isn't declared on that side of the config.

**Ensure the corresponding Tech generator is configured in RSI XML** (each
generator expands to 10 slots, e.g. `C11..C110`):
```xml
<SEND>
  <ELEMENTS>
    <ELEMENT TAG="DEF_Tech.C1" TYPE="DOUBLE" INDX="INTERNAL" />
  </ELEMENTS>
</SEND>
<RECEIVE>
  <ELEMENTS>
    <ELEMENT TAG="DEF_Tech.T2" TYPE="DOUBLE" INDX="INTERNAL" HOLDON="0" />
  </ELEMENTS>
</RECEIVE>
```

### Timing Issues

- **Reduce check_interval for faster response**: `api.krl.wait_for_signal(1, check_interval=0.005, group=None)`
- **Increase timeout for slow operations**: `api.krl.wait_for_signal(1, timeout=60.0, group=None)`
- **Add WAIT SEC delays in KRL for signal propagation**

## Next Steps

After understanding these templates:

1. **Adapt templates** to your specific application
2. **Test coordination** patterns with your robot
3. **Implement error recovery** mechanisms
4. **Document custom** coordination protocols
5. **Create application-specific** state machines

## References

- [RSIPI Documentation](../../README.md)
- [Phase 3 Summary](../../PHASE_3_SUMMARY.md) (when available)
- [KUKA RSI 3.3 Documentation](https://www.kuka.com)

---

**Template Author**: RSIPI Development Team
**Last Updated**: January 17, 2026
**Version**: 1.0
