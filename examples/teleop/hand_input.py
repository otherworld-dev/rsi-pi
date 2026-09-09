"""Hand tracking as a teleop input: MediaPipe HandLandmarker + an OpenCV camera.

Your hand is a virtual joystick. Hold an OPEN hand in front of the camera:
where it sits relative to the frame centre is the velocity demand -
left/right is Y, up/down is Z, and pushing towards or pulling away from the
camera is X (from the apparent size of the hand against the size it had
when you opened it). Close your fist, or take the hand away, and the demand
is zero: the open hand IS the deadman. Record, replay, E-stop and speed come
from the keyboard, the same keys as --input keys, because a gesture is too
easy to trigger by accident.

The camera and the model run in their own thread at camera rate (~30 fps,
inference 10-30 ms on a CPU) so the 50 Hz control loop never waits on
them; poll() returns the latest filtered result. No hand for more than
DROPOUT_S -> deadman released. A preview window shows the landmarks, the
deadzone and the demand vector.

Needs mediapipe and opencv, which live in the disposable .venv-demo, and
the hand landmarker model, fetched once into examples/teleop/models/:

    .venv-demo\\Scripts\\python examples\\teleop\\teleop.py --input hand
"""
import math
import os
import threading
import time
import urllib.request
from pathlib import Path

from inputs import Command, KeyboardInput, _shape

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")
MODEL_PATH = Path(__file__).resolve().parent / "models" / "hand_landmarker.task"

CAMERA = 0
DROPOUT_S = 0.2        # no hand for this long -> deadman released
DEADZONE = 0.15        # of the half-frame, around the centre
DEPTH_DEADZONE = 0.15  # of the size ratio
DEPTH_GAIN = 4.0       # x demand per unit of (size / size_when_opened - 1)
SMOOTH = 0.5           # EMA weight of the newest frame

WRIST = 0
PALM = (0, 5, 9, 13, 17)                           # wrist + the four MCP knuckles
FINGERS = ((8, 6), (12, 10), (16, 14), (20, 18))   # (tip, pip): index .. little
CONNECTIONS = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
               (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
               (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def hand_state(points):
    """From 21 normalised (x, y) landmarks: (is_open, centre, size).

    A finger counts as extended when its tip is further from the wrist than
    its middle joint; three or more extended is an open hand. Size is the
    wrist-to-middle-knuckle distance, which tracks distance from the camera.
    Pure, so it can be checked without a camera.
    """
    wrist = points[WRIST]
    extended = sum(_d(points[tip], wrist) > _d(points[pip], wrist) for tip, pip in FINGERS)
    centre = (sum(points[i][0] for i in PALM) / len(PALM),
              sum(points[i][1] for i in PALM) / len(PALM))
    return extended >= 3, centre, _d(points[0], points[9])


def demand_from(centre, size, ref_size):
    """Hand centre and size -> (x, y, z) demands in -1..1.

    Image x runs left to right and is already mirrored, so a hand moved to
    the right gives +Y; image y runs top to bottom, so a hand raised gives
    +Z. Depth comes from the size ratio against the size when the hand was
    opened: bigger (closer to the camera) is +X.
    """
    y = _shape((centre[0] - 0.5) * 2.0, DEADZONE)
    z = _shape((0.5 - centre[1]) * 2.0, DEADZONE)
    x = 0.0
    if ref_size:
        ratio = max(-1.0, min(1.0, (size / ref_size - 1.0) * DEPTH_GAIN))
        x = _shape(ratio, DEPTH_DEADZONE)
    return x, y, z


class HandInput:
    NAME = "hand"

    def __init__(self, camera=CAMERA, preview=True):
        self._camera = camera
        self._preview = preview
        self._keys = KeyboardInput()
        self._lock = threading.Lock()
        self._demand = (0.0, 0.0, 0.0)
        self._held = False
        self._seen = 0.0
        self._ref_size = None
        self._error = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self.fps = 0.0
        self._ensure_model()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(15.0) or self._error:
            raise RuntimeError(self._error or "hand tracker did not start")

    # ------------------------------------------------------------- setup

    @staticmethod
    def _ensure_model():
        if MODEL_PATH.exists():
            return
        MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        print(f"fetching the hand landmarker model (about 8 MB) -> {MODEL_PATH}")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)

    # ------------------------------------------------------------ thread

    def _run(self):
        # MediaPipe and TFLite announce themselves on stderr at INFO level;
        # keep the console for the teleop status.
        os.environ.setdefault("GLOG_minloglevel", "2")
        os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions
            from mediapipe.tasks.python.vision import (HandLandmarker, HandLandmarkerOptions,
                                                       RunningMode)
        except ImportError as e:
            self._error = f"{e} - run this with .venv-demo (see examples/teleop/README.md)"
            self._ready.set()
            return

        landmarker = HandLandmarker.create_from_options(HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
            running_mode=RunningMode.VIDEO, num_hands=1))
        cap = cv2.VideoCapture(self._camera, cv2.CAP_DSHOW)
        if not cap.isOpened():
            self._error = f"camera {self._camera} did not open"
            self._ready.set()
            return
        self._ready.set()

        t0 = time.time()
        last_ms = -1
        frames, fps_t = 0, time.time()
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    continue
                frame = cv2.flip(frame, 1)                      # a mirror, like a webcam preview
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ms = max(last_ms + 1, int((time.time() - t0) * 1000))   # must increase
                last_ms = ms
                result = landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ms)

                points = None
                if result.hand_landmarks:
                    points = [(lm.x, lm.y) for lm in result.hand_landmarks[0]]
                self._update(points)

                frames += 1
                if time.time() - fps_t >= 1.0:
                    self.fps = frames / (time.time() - fps_t)
                    frames, fps_t = 0, time.time()
                if self._preview:
                    self._draw(cv2, frame, points)
                    cv2.imshow("RSIPI hand teleop", frame)
                    cv2.waitKey(1)
        finally:
            cap.release()
            if self._preview:
                cv2.destroyAllWindows()

    def _update(self, points):
        now = time.time()
        if points is None:
            return                        # poll() applies the dropout timeout
        is_open, centre, size = hand_state(points)
        with self._lock:
            if is_open and not self._held:
                self._ref_size = size     # depth is relative to where the hand opened
            self._held = is_open
            self._seen = now
            if is_open:
                x, y, z = demand_from(centre, size, self._ref_size)
                px, py, pz = self._demand
                self._demand = (px + SMOOTH * (x - px), py + SMOOTH * (y - py), pz + SMOOTH * (z - pz))
            else:
                self._demand = (0.0, 0.0, 0.0)

    def _draw(self, cv2, frame, points):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.circle(frame, (cx, cy), int(DEADZONE * w / 2), (120, 120, 120), 1)
        with self._lock:
            held, (dx, dy, dz) = self._held, self._demand
        if points:
            px = [(int(p[0] * w), int(p[1] * h)) for p in points]
            colour = (0, 200, 0) if held else (0, 0, 220)
            for a, b in CONNECTIONS:
                cv2.line(frame, px[a], px[b], colour, 2)
            for p in px:
                cv2.circle(frame, p, 3, colour, -1)
            _, centre, _ = hand_state(points)
            hand = (int(centre[0] * w), int(centre[1] * h))
            cv2.arrowedLine(frame, (cx, cy), hand, colour, 2)
        state = "OPEN HAND: driving" if held else "fist / no hand: stopped"
        cv2.putText(frame, state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 200, 0) if held else (0, 0, 220), 2)
        cv2.putText(frame, f"X {dx:+.2f}  Y {dy:+.2f}  Z {dz:+.2f}   {self.fps:.0f} fps",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)

    # -------------------------------------------------------------- poll

    def poll(self) -> Command:
        keys = self._keys.poll()
        with self._lock:
            x, y, z = self._demand
            held, seen = self._held, self._seen
        alive = self._thread.is_alive() and not self._error
        if time.time() - seen > DROPOUT_S:
            held, x, y, z = False, 0.0, 0.0, 0.0
        cmd = Command(x=x, y=y, z=z, deadman=held and alive, connected=alive)
        cmd.events = keys.events
        return cmd

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
