from RSIPI import rsi_api

rsi = rsi_api.RSIAPI()
rsi.start_rsi()

# Set digital output (e.g., to open gripper)
rsi.update_digital_io(125)  # Example binary pattern

rsi.stop_rsi()
