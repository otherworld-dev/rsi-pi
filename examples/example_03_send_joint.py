from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.start_rsi()

# Move Joint A1 by 10 degrees
rsi.update_joints(a1=10)

rsi.stop_rsi()
