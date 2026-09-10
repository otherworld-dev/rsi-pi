"""Print what the hand tracker sees, 5 times a second, with no robot.

Use it to tune the gesture thresholds in hand_input.py: hold a Vulcan
salute or fingers-together in front of the camera and watch the gesture
column and the fingertip gaps (as fractions of palm width).

    .venv-demo\\Scripts\\python examples\\teleop\\gesture_probe.py
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hand_input import HandInput, VULCAN_GAP, VULCAN_RATIO, TOGETHER_GAP

if __name__ == "__main__":
    print(f"vulcan: middle-ring gap > {VULCAN_GAP} and > {VULCAN_RATIO} x the others; "
          f"together: all gaps < {TOGETHER_GAP}. Ctrl+C to stop.")
    h = HandInput()
    try:
        while True:
            cmd = h.poll()
            s = h._state or {}
            print(f"{s.get('state', '-'):34} gesture={str(s.get('gesture')):9} "
                  f"gaps={s.get('gaps')}  events={sorted(cmd.events)}  {h.fps:.0f} fps")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        h.close()
