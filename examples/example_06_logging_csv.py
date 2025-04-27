from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.enable_csv_logging()

rsi.start_rsi()

print("Logging robot data to CSV. Press Enter to stop.")
input()

rsi.stop_rsi()
