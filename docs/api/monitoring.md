# Monitoring

Position, IPOC, motor currents, applied corrections, override and STATUS reads, as dicts, NumPy or pandas.

## Usage

```python
# Comprehensive snapshot
data = api.monitoring.get_live_data()
# {"position": {X,Y,Z,A,B,C}, "velocity": {...}, "force": {...}, "ipoc": 123456}

# Individual reads
pos = api.monitoring.get_position()      # {X, Y, Z, A, B, C}
ipoc = api.monitoring.get_ipoc()         # Interrupt point counter

# Motor currents (MACur, an INTERNAL tag: Max and Full contexts). Units are
# not stated by KUKA - treat as relative. Raises RSIVariableError if the
# context has no MACur; get_force() is the older name and returns zeros then.
currents = api.monitoring.get_motor_currents()   # {A1..A6}

# Max context only: what the controller actually applied, and $OV_PRO
api.monitoring.get_applied_correction()          # POSCORRMON {X..C}, {} if not wired
api.monitoring.get_applied_joint_correction()    # AXISCORRMON {A1..A6}
api.monitoring.get_override()                    # $OV_PRO %, None if not wired
api.monitoring.set_override(50)                  # 1-100; ValueError outside that
api.monitoring.get_correction_limit_status()     # POSCORR Stat decoded, if your context
# gets it to the PC (it cannot come back over Ethernet - see docs/rsi-objects.md);
# None on the shipped contexts
api.monitoring.get_robot_status(1, "Mode_Op")    # STATUS object n decoded ("T1", "AUT"...)

# NumPy/Pandas formats
arr = api.monitoring.get_live_data_as_numpy()        # shape (4, 6)
df = api.monitoring.get_live_data_as_dataframe()     # single-row DataFrame

# Console watch (blocking, Ctrl+C to stop)
api.monitoring.watch_network(duration=10, rate=0.2)
```

## Reference

::: RSIPI.monitoring_api.MonitoringAPI
