# RSI mode and rate limiting

### Absolute vs Relative Mode

The `rsi_mode` parameter must match what your KRL program uses with `RSI_MOVECORR()`:

| Mode | Behavior | Use Case |
|------|----------|----------|
| `"relative"` | Corrections are **added** to the programmed path each cycle. Sending `X=1.0` every cycle moves 1mm/cycle continuously. | Continuous adjustments, sensor feedback |
| `"absolute"` | Corrections specify **total offset** from programmed path. Sending `X=10.0` holds 10mm offset regardless of how many cycles. | Target position offsets |

## Cycle Time

KUKA RSI supports two cycle rates, configured on the robot controller side. RSIPI's network loop is reactive (it responds to whatever the robot sends), but the `cycle_time` parameter ensures diagnostics, health checks, and jitter warnings use the correct baseline:

```python
# 4ms cycle / 250Hz (default)
api = RSIAPI("RSI_EthernetConfig.xml", cycle_time=0.004)

# 12ms cycle / 83Hz
api = RSIAPI("RSI_EthernetConfig.xml", cycle_time=0.012)
```

| Cycle Time | Frequency | Use Case |
|------------|-----------|----------|
| `0.004` (4ms) | 250 Hz | High-frequency corrections, sensor feedback loops |
| `0.012` (12ms) | 83 Hz | Standard motion corrections, less demanding applications |

## Rate Limiting

Rate limiting caps the per-cycle change to prevent sudden jumps:

```python
api = RSIAPI(
    "RSI_EthernetConfig.xml",
    rsi_mode="relative",
    max_cartesian_rate=0.5,   # Max 0.5 mm per cycle
    max_joint_rate=0.1,       # Max 0.1 deg per cycle
    cycle_time=0.004          # 4ms cycle
)
```

Set rates to `0.0` (default) to disable rate limiting. Clamping is applied in the network process right before the response is sent to the robot.
