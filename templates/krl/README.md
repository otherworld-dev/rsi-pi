# KRL Coordination Templates

This directory contains KRL program templates demonstrating common Python-KRL coordination patterns using RSIPI.

## Available Templates

### 1. basic_handshake.src
**Simple I/O handshaking between Python and KRL**

- KRL signals "ready" via digital output
- Python waits for signal, processes data
- Python signals "complete" via input
- KRL waits for completion, then continues

**Use Case**: Basic synchronization, ensuring Python completes processing before KRL continues.

**Coordination Methods Used**:
- `api.krl.wait_for_signal(channel, timeout)`
- `api.krl.signal_complete(channel)`

### 2. parameter_passing.src
**Bidirectional numerical data exchange via Tech variables**

- KRL writes current position to Tech.T variables
- Python reads position data
- Python calculates target and writes to Tech.C variables
- KRL reads target and executes motion

**Use Case**: Passing numerical parameters (positions, forces, tolerances) between Python and KRL.

**Coordination Methods Used**:
- `api.krl.read_param(slot)` - Read from Tech.T
- `api.krl.write_param(slot, value)` - Write to Tech.C
- `api.krl.wait_for_signal(channel, timeout)`
- `api.krl.signal_complete(channel)`

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
- State variable in Tech.T[11]
- Command variable in Tech.C[11]

## Python-KRL Coordination Patterns

### Pattern 1: Simple Handshake

```python
# Python side
api.krl.wait_for_signal(1)  # Wait for KRL ready signal
# Do processing...
api.krl.signal_complete(1)  # Signal KRL to continue
```

```krl
; KRL side
$OUT[1] = TRUE  ; Signal ready to Python
; Wait for Python completion
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE
```

### Pattern 2: Parameter Exchange

```python
# Python side
api.krl.wait_for_signal(1)

# Read from KRL
value = api.krl.read_param('T11')

# Process and write back
result = process(value)
api.krl.write_param('C11', result)

api.krl.signal_complete(1)
```

```krl
; KRL side
$TECH.T[11] = some_value
$OUT[1] = TRUE  ; Signal data ready

; Wait for Python
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE

; Read result
result = $TECH.C[11]
```

### Pattern 3: Continuous Monitoring

```python
# Python side - non-blocking monitoring loop
api.start()

while api.is_running():
    state = api.krl.read_param('T11')

    if state == 1:  # Specific state
        # React to state change
        api.krl.write_param('C11', calculated_value)
        api.krl.signal_complete(1)

    time.sleep(0.1)  # Check every 100ms

api.stop()
```

```krl
; KRL side - updates state continuously
$TECH.T[11] = current_state

; Wait for Python response when needed
WHILE $IN[1] == FALSE
  WAIT SEC 0.1
ENDWHILE

calculated = $TECH.C[11]
```

## Tech Variable Conventions

### Tech.C Variables (Python → KRL)
**"Control" variables - Python writes, KRL reads**

| Slot | Description | Example Usage |
|------|-------------|---------------|
| C11  | Command/state | 0=continue, 1=pause, 2=abort |
| C12-C14 | Position offsets | X, Y, Z corrections |
| C15-C17 | Target position | Calculated target coordinates |
| C18-C20 | Process parameters | Speed, force, tolerance |
| C21+ | Custom parameters | Application-specific data |

```python
# Python writes
api.krl.write_param('C11', 0)  # Command: continue
api.krl.write_param('C12', 5.0)  # X offset
api.krl.write_param('C13', -2.0)  # Y offset
```

```krl
; KRL reads
command = $TECH.C[11]
offset_x = $TECH.C[12]
offset_y = $TECH.C[13]
```

### Tech.T Variables (KRL → Python)
**"Transfer" variables - KRL writes, Python reads**

| Slot | Description | Example Usage |
|------|-------------|---------------|
| T11  | Current state | State machine state number |
| T12-T14 | Current position | X, Y, Z coordinates |
| T15-T17 | Force/torque | Measured forces |
| T18-T20 | Sensor readings | External sensor data |
| T21+ | Custom data | Application-specific values |

```krl
; KRL writes
$TECH.T[11] = current_state
$TECH.T[12] = $POS_ACT.X
$TECH.T[13] = $POS_ACT.Y
$TECH.T[14] = $POS_ACT.Z
```

```python
# Python reads
state = api.krl.read_param('T11')
pos_x = api.krl.read_param('T12')
pos_y = api.krl.read_param('T13')
pos_z = api.krl.read_param('T14')
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
# Python I/O methods
api.krl.wait_for_signal(1)  # Wait for $OUT[1]
api.krl.signal_complete(1)  # Set $IN[1]
api.io.set_output(2, True)  # Control $OUT[2]
```

## Error Handling Best Practices

### Timeouts

**Always use timeouts to prevent indefinite blocking:**

```python
# Python
if not api.krl.wait_for_signal(1, timeout=10.0):
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
api.krl.wait_for_signal(1)

# Send real-time corrections during KRL motion
for i in range(100):
    correction = calculate_correction()
    api.motion.update_cartesian(X=correction)
    time.sleep(0.004)  # 250Hz update rate

# Signal motion phase complete
api.krl.signal_complete(1)

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

    if api.krl.wait_for_signal(1, timeout=30.0):
        print("✅ KRL signaled ready!")

        # Simulate processing
        time.sleep(2.0)
        print("Processing complete")

        # Signal back to KRL
        api.krl.signal_complete(1)
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

### Signal Not Received

**Check I/O configuration in RSI XML:**
```xml
<SEND>
  <XML>
    <ELEMENT Tag="Digin" Type="INT"/>
  </XML>
</SEND>
<RECEIVE>
  <XML>
    <ELEMENT Tag="Digout" Type="INT"/>
  </XML>
</RECEIVE>
```

### Tech Variable Not Found

**Ensure Tech variables are configured in RSI XML:**
```xml
<SEND>
  <XML>
    <ELEMENT Tag="Tech" Type="DOUBLE" Indizes="[1..199]"/>
  </XML>
</SEND>
<RECEIVE>
  <XML>
    <ELEMENT Tag="Tech" Type="DOUBLE" Indizes="[1..199]"/>
  </XML>
</RECEIVE>
```

### Timing Issues

- **Reduce check_interval for faster response**: `api.krl.wait_for_signal(1, check_interval=0.005)`
- **Increase timeout for slow operations**: `api.krl.wait_for_signal(1, timeout=60.0)`
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
