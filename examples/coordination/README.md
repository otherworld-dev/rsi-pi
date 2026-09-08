# Python-KRL Coordination Examples

This directory contains Python examples demonstrating Python-KRL coordination patterns using RSIPI Phase 3 features.

## Prerequisites

- RSIPI library installed (`pip install -e .` from rsi-pi directory)
- KUKA robot controller with RSI 3.3 configured
- RSI_EthernetConfig.xml configured with required I/O and Tech variables
- Corresponding KRL programs uploaded to robot controller (see `controller/Program/`)

## Examples

### 01_basic_handshake.py
**Simple I/O handshaking**

Demonstrates basic bidirectional signaling using digital I/O channels.

**Requires**: `controller/Program/basic_handshake.src` running on robot

**Run**:
```bash
python 01_basic_handshake.py --config path/to/RSI_EthernetConfig.xml
```

**Flow**:
1. Start Python script
2. Execute KRL program on teach pendant
3. Python waits for KRL ready signal
4. Python performs processing
5. Python signals completion
6. KRL continues with motion

**API Features Demonstrated**:
- `api.krl.wait_for_signal(channel, timeout, group=None)`
- `api.krl.signal_complete(channel, group=None)`

---

### 02_parameter_passing.py
**Bidirectional parameter exchange**

Demonstrates numerical data exchange using RSI Tech variables.

**Requires**: `controller/Program/parameter_passing.src` running on robot

**Run**:
```bash
python 02_parameter_passing.py --config path/to/RSI_EthernetConfig.xml
```

**Flow**:
1. KRL writes current position to Tech.C variables
2. Python reads position data
3. Python calculates target position
4. Python writes target to Tech.T variables
5. Python signals completion
6. KRL reads target and executes motion

**API Features Demonstrated**:
- `api.krl.read_param(slot)` - Read Tech.C variables (KRL-to-Python channel)
- `api.krl.write_param(slot, value)` - Write Tech.T variables (Python-to-KRL channel)
- `api.krl.wait_for_signal(channel, timeout, group=None)`
- `api.krl.signal_complete(channel, group=None)`

---

### 03_state_machine.py
**Multi-state workflow coordination**

Demonstrates complex state machine with error handling and calibration.

**Requires**: `controller/Program/state_machine.src` running on robot

**Run**:
```bash
python 03_state_machine.py --config path/to/RSI_EthernetConfig.xml
```

**Flow**:
1. Python monitors state variable (Tech.C11)
2. Calibration state: Python performs calibration routine
3. Python writes calibration offsets to Tech.T
4. Executing state: KRL uses offsets for motion
5. Complete state: Workflow finishes successfully

**States**:
- 0: IDLE - Waiting to start
- 1: CALIBRATING - Python calibration in progress
- 2: READY - Ready for motion
- 3: EXECUTING - Robot in motion
- 4: COMPLETE - Task finished
- 9: ERROR - Error condition

**API Features Demonstrated**:
- All coordination methods from examples 01 and 02
- State monitoring loop
- Error handling with I/O signals
- Real-time state transitions

---

## Configuration Requirements

### RSI XML Configuration

Your `RSI_EthernetConfig.xml` must include the following elements (the
parser expects `SEND/ELEMENTS` and `RECEIVE/ELEMENTS` containers with
`<ELEMENT TAG="..." TYPE="..." INDX="..."/>` children - see
`config_parser.py` and the shipped `RSI_EthernetConfig.xml` /
`RSI_EthernetConfig_Full.xml` for the real schema):

**Digital I/O** (DiL/DiO word notation, as shipped in both configs):
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

**Tech Variables** (each `DEF_Tech.Cn` / `DEF_Tech.Tn` generator expands
to 10 slots, e.g. `C11..C110` / `T11..T110` - see
`RSI_EthernetConfig_Full.xml` for the full C1-C6/T1-T6 pattern):
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

### Network Settings

Ensure your RSI network settings match:
```xml
<IP_NUMBER>192.168.1.100</IP_NUMBER>  <!-- Your PC IP -->
<PORT>49152</PORT>
<SENTYPE>ImFree</SENTYPE>
```

## Running Examples

### Step 1: Start Python Script

```bash
# In one terminal
cd examples/coordination
python 01_basic_handshake.py --config ../../RSI_EthernetConfig.xml
```

### Step 2: Execute KRL Program

1. Upload corresponding KRL program to robot controller
2. Switch to AUTO mode on teach pendant
3. Select and execute the KRL program
4. Monitor coordination in Python terminal

### Step 3: Monitor Output

Python will log:
- State transitions
- I/O signal changes
- Parameter reads/writes
- Errors and warnings

**Example Output**:
```
2026-01-17 14:32:01 - INFO - Starting RSI communication...
2026-01-17 14:32:01 - INFO - ✅ RSI started successfully
2026-01-17 14:32:01 - INFO - Waiting for KRL ready signal...
2026-01-17 14:32:05 - INFO - ✅ KRL signaled ready!
2026-01-17 14:32:05 - INFO - Performing Python-side processing...
2026-01-17 14:32:07 - INFO - ✅ Processing complete
2026-01-17 14:32:07 - INFO - ✅ Signaled KRL to continue
```

## Customizing Examples

### Modify Processing Logic

In `01_basic_handshake.py`:
```python
# Replace simulated processing
time.sleep(2.0)

# With actual processing
sensor_data = read_sensor()
processed_result = analyze_data(sensor_data)
update_database(processed_result)
```

### Change Calculation Logic

In `02_parameter_passing.py`:
```python
# Modify target calculation
target_x = current_x + 100.0  # Original
target_x = calculate_adaptive_target(current_x, sensor_feedback)  # Custom
```

### Extend State Machine

In `03_state_machine.py`:
```python
# Add new states
class State(IntEnum):
    IDLE = 0
    CALIBRATING = 1
    INSPECTING = 2  # NEW STATE
    READY = 3  # Renumber subsequent states
    # ...

# Handle new state
elif current_state == State.INSPECTING:
    inspection_result = perform_inspection()
    api.krl.write_param('T25', inspection_result)  # Tech.T - Python writes
    api.krl.signal_complete(1, group=None)
```

## Troubleshooting

### "Timeout waiting for KRL signal"

**Problem**: Python doesn't receive the expected digital-input signal

**Solutions**:
1. Verify KRL program is running on robot
2. Confirm `wait_for_signal()`/`get_input()` are called with `group=None`
   so IOAPI auto-detects the DiL word - the hardcoded default
   `group='Digin'` does not exist in either shipped config and raises
   `RSIVariableError` immediately instead of waiting
3. Remember digital inputs (DiL / $IN[]) are hardware/PLC-driven -
   RSIPI cannot read a robot output readback (Digout.o1) as an "input"
   channel, so a KRL program's own `$OUT[n] = TRUE` cannot satisfy
   `wait_for_signal()`; use `api.krl.signal_complete(channel, group=None)`
   (DiO) for the Python-to-KRL direction instead
4. Verify network connectivity (ping robot IP)
5. Increase timeout: `api.krl.wait_for_signal(1, timeout=60.0, group=None)`

### "RSIVariableError: Tech.C11/T11 not found in send_variables/receive_variables"

**Problem**: Tech.C variables live in `send_variables` (KRL writes, Python
reads via `read_param()`) and Tech.T variables live in `receive_variables`
(Python writes via `write_param()`, KRL reads) - never the other way
round. Calling `read_param()` on a `T` slot, or `write_param()` on a `C`
slot, raises this error because that slot's group was never declared on
that side of the config.

**Solutions**:
1. Use `read_param('Cxx')` for KRL-to-Python state and `write_param('Txx', ...)`
   for Python-to-KRL commands - not the reverse
2. Add the corresponding `DEF_Tech.Cn` (under `<SEND>`) or `DEF_Tech.Tn`
   (under `<RECEIVE>`) generator to the RSI XML if the slot you need
   isn't declared - each generator provides 10 slots, e.g. `C11..C110`
3. Restart robot controller after XML changes
4. Verify variable configuration: `api.tools.show_variables()`

### "Connection refused"

**Problem**: Cannot connect to robot controller

**Solutions**:
1. Check robot IP address in RSI XML
2. Verify robot is in correct mode (T1/T2/AUTO)
3. Ensure firewall allows UDP port 49152
4. Check RSI is enabled on robot controller

### KRL Program Halts

**Problem**: KRL program stops unexpectedly

**Solutions**:
1. Check KRL timeout values (increase if needed)
2. Verify Python script is running before KRL execution
3. Check error signals ($IN[2] for error condition)
4. Review KRL logs on teach pendant

## Advanced Usage

### Non-Blocking Monitoring

For continuous operation without blocking:

```python
import threading

def monitor_state():
    while running:
        state = api.krl.read_param('C11')  # Tech.C - KRL writes, Python reads
        if state == CRITICAL_STATE:
            handle_critical_state()
        time.sleep(0.1)

# Run monitoring in background thread
monitor_thread = threading.Thread(target=monitor_state, daemon=True)
monitor_thread.start()

# Main thread does other work
perform_other_tasks()
```

### Multiple Coordination Channels

Use different I/O channels for parallel coordination:

```python
# Channel 1: Main workflow
if api.krl.wait_for_signal(1, group=None):
    api.krl.signal_complete(1, group=None)

# Channel 2: Emergency stop
if api.io.get_input(2):  # Emergency input
    logging.error("Emergency stop!")
    api.safety.stop()

# Channel 3: Auxiliary signaling
api.io.pulse(3, duration=0.1)  # Quick pulse signal
```

### Integration with Motion Control

Combine coordination with real-time RSI corrections:

```python
# Wait for motion start
api.krl.wait_for_signal(1, group=None)

# Send real-time corrections during KRL motion
for i in range(100):
    sensor_offset = get_sensor_offset()
    api.motion.update_cartesian(X=sensor_offset)
    time.sleep(0.004)  # 250Hz

# Signal motion complete
api.krl.signal_complete(1, group=None)
```

## Next Steps

1. **Test examples** with your robot controller
2. **Adapt templates** to your specific application
3. **Implement error recovery** mechanisms
4. **Create custom workflows** for your use case
5. **Document coordination** protocols for your team

## References

- [RSIPI API Documentation](../../README.md)
- [KRL Templates](../../controller/Program/README.md)
- [Phase 3 Summary](../../PHASE_3_SUMMARY.md) (when available)
- [KUKA RSI 3.3 Manual](https://www.kuka.com)

---

**Last Updated**: January 17, 2026
**RSIPI Version**: 2.0.0
