from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.enable_graphing()

rsi.start_rsi()

print("Live graphing started. Press Enter to stop.")
input()

rsi.stop_rsi()
