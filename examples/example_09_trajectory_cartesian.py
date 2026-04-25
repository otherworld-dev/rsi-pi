from RSIPI import RSIAPI
import time

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    api = RSIAPI()
    api.start()

    # Plan simple trajectory
    points = [
        {"X": 0, "Y": 0, "Z": 0},
        {"X": 50, "Y": 0, "Z": 0},
        {"X": 50, "Y": 50, "Z": 0},
        {"X": 0, "Y": 50, "Z": 0},
        {"X": 0, "Y": 0, "Z": 0}
    ]

    for point in points:
        api.motion.update_cartesian(**point)
        time.sleep(0.5)

    api.stop()
