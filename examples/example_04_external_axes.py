from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.start_rsi()

# Move external axis E1 by 100mm
rsi.update_external(e1=100)

rsi.stop_rsi()
