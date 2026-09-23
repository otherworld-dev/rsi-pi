# Safety

The software E-stop, per-variable correction limits and the override for calibration work.

## Usage

```python
# Emergency stop (software-level, NOT safety-rated)
api.safety.stop()
api.safety.is_stopped()          # True

# Reset E-stop
api.safety.reset()

# Configure correction limits
api.safety.set_limit("RKorr.X", -50.0, 50.0)
api.safety.set_limit("AKorr.A1", -10.0, 10.0)

# View all limits
limits = api.safety.get_limits()
for var, (lo, hi) in limits.items():
    print(f"{var}: [{lo}, {hi}]")

# Full status
status = api.safety.status()
# {"emergency_stop": False, "safety_override": False, "limits": {...}}

# Override limits (use with extreme caution)
api.safety.override(True)
# ... calibration work ...
api.safety.override(False)
```

## Reference

::: RSIPI.safety_api.SafetyAPI
