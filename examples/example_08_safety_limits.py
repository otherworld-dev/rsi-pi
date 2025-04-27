from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()

# Set X axis soft limits
rsi.set_safety_limit(axis="X", min_value=-500, max_value=500)

rsi.start_rsi()

try:
    while True:
        pass
except KeyboardInterrupt:
    rsi.stop_rsi()