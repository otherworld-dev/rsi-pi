from RSIPI import rsi_api
import time

rsi = rsi_api.RSIAPI()
rsi.start_rsi()

# Plan simple trajectory
points = [
    {"x": 0, "y": 0, "z": 0},
    {"x": 50, "y": 0, "z": 0},
    {"x": 50, "y": 50, "z": 0},
    {"x": 0, "y": 50, "z": 0},
    {"x": 0, "y": 0, "z": 0}
]

for point in points:
    rsi.update_cartesian(**point)
    time.sleep(0.5)

rsi.stop_rsi()
