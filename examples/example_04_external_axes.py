from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # External-axis corrections need the Full context: RSI_EthernetConfig_Full.xml
    # declares EKorr.E1-E6 in RECEIVE, and the controller-side RSIPI_Full.rsi wires
    # them into an AXISCORREXT1 object. The default config (RSI_EthernetConfig.xml)
    # has no EKorr channels, so it must be requested explicitly here.
    api = RSIAPI('RSI_EthernetConfig_Full.xml')
    api.start()

    # Move external axis E1 by 100mm (writes EKorr.E1, applied by AXISCORREXT1)
    api.motion.move_external_axis('E1', 100)

    api.stop()
