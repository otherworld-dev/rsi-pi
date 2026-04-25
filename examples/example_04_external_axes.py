from RSIPI import RSIAPI

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.start()

    # Move external axis E1 by 100mm
    api.motion.move_external_axis('E1', 100)

    api.stop()
