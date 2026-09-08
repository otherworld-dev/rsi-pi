from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI('RSI_EthernetConfig.xml')
    api.start()

    # Set digital output (e.g., to open gripper). With the shipped configs this
    # writes bit 0 of the DiO word in RECEIVE. Note: Digout.o1-3 in SEND is a
    # separate, read-only channel - the robot's readback of its own outputs.
    api.io.set_output(1, True)

    api.stop()
