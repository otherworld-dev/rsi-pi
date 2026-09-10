"""Calibrate the gripper gestures to your hand.

Asks for each hand shape in turn, records the fingertip gaps for a few
seconds, derives thresholds that sit midway between the shapes, and writes
them to gestures.json next to this file. hand_input.py loads that file if
it exists, so the calibration applies to the teleop demo automatically.

    .venv-demo\\Scripts\\python examples\\teleop\\gesture_calibrate.py

Shapes: a relaxed open hand (the one you drive with), the four fingers
pressed together (closes the gripper), and the Vulcan salute (opens it).
"""
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hand_input import HandInput, GESTURES_FILE

SECONDS = 3.0
SHAPES = (
    ("relaxed", "a RELAXED open hand, palm to the camera - the shape you drive with"),
    ("pressed", "the four fingers PRESSED together, flat, palm to the camera"),
    ("salute", "the VULCAN SALUTE: index+middle together, ring+little together, a gap between"),
)


def record(h, seconds):
    """Collect (g1, g2, g3) samples while a hand is in view."""
    samples, end = [], time.time() + seconds
    while time.time() < end:
        h.poll()
        s = h._state or {}
        if s.get("gaps") and s.get("state") not in ("no hand", "fist: stopped"):
            samples.append(s["gaps"])
        time.sleep(0.05)
    return samples


def summary(samples):
    cols = list(zip(*samples))
    return [(statistics.median(c), min(c), max(c)) for c in cols]


if __name__ == "__main__":
    h = HandInput()
    data = {}
    try:
        for key, prompt in SHAPES:
            input(f"\nHold {prompt}.\nPress Enter, then hold it for {SECONDS:.0f} s... ")
            for i in range(3, 0, -1):
                print(f"  {i}", end="\r"); time.sleep(0.5)
            samples = record(h, SECONDS)
            if len(samples) < 10:
                print(f"  only {len(samples)} readings - the hand was not seen; try again")
                samples = record(h, SECONDS)
            data[key] = summary(samples)
            print(f"  {key}: {len(samples)} readings; gaps (median, min, max) per finger pair:")
            for name, (m, lo, hi) in zip(("index-middle", "middle-ring", "ring-little"), data[key]):
                print(f"    {name:13} {m:.2f}  [{lo:.2f} .. {hi:.2f}]")
    finally:
        h.close()

    rel, prs, sal = data["relaxed"], data["pressed"], data["salute"]
    # together: index-middle and middle-ring both below a line midway between
    # the pressed maximum and the relaxed minimum
    pressed_top = max(prs[0][2], prs[1][2])
    relaxed_floor = min(rel[0][1], rel[1][1])
    together = round((pressed_top + relaxed_floor) / 2, 2)
    # vulcan: middle-ring above a line midway between the salute minimum and
    # the largest middle-ring anyone else shows; and a ratio to the other gaps
    other_top = max(rel[1][2], prs[1][2])
    vulcan_gap = round((sal[1][1] + other_top) / 2, 2)
    ratio_salute = sal[1][0] / max(sal[0][0], sal[2][0])
    ratio_relaxed = rel[1][0] / max(rel[0][0], rel[2][0])
    vulcan_ratio = round(max(1.2, (ratio_salute + ratio_relaxed) / 2), 2)

    cal = {"TOGETHER_GAP": together, "VULCAN_GAP": vulcan_gap, "VULCAN_RATIO": vulcan_ratio,
           "measured": {k: [list(map(lambda x: round(x, 3), t)) for t in v] for k, v in data.items()}}
    GESTURES_FILE.write_text(json.dumps(cal, indent=2))
    print(f"\nthresholds -> {GESTURES_FILE}")
    print(f"  TOGETHER_GAP {together}  (pressed up to {pressed_top:.2f}, relaxed from {relaxed_floor:.2f})")
    print(f"  VULCAN_GAP   {vulcan_gap}  (salute from {sal[1][1]:.2f}, others up to {other_top:.2f})")
    print(f"  VULCAN_RATIO {vulcan_ratio}  (salute {ratio_salute:.2f}, relaxed {ratio_relaxed:.2f})")
    if pressed_top >= relaxed_floor:
        print("  WARNING: pressed and relaxed overlap - hold them more distinctly and rerun")
    if sal[1][1] <= other_top:
        print("  WARNING: the salute's middle-ring gap overlaps the other shapes - rerun")
