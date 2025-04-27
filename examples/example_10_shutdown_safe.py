from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()

try:
    rsi.start_rsi()
    print("Press Ctrl+C to stop RSI safely.")
    while True:
        pass

except KeyboardInterrupt:
    print("\nEmergency stop triggered.")
    rsi.safety_stop()
    rsi.stop_rsi()
