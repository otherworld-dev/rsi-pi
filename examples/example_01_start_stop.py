from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()

rsi.start_rsi()

print("RSI connection started. Press Enter to stop.")
input()

rsi.stop_rsi()
print("RSI connection stopped.")
