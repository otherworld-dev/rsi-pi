from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.start_rsi()

# Move TCP 50mm along X-axis
rsi.update_cartesian(x=50, y=0, z=0)

rsi.stop_rsi()
