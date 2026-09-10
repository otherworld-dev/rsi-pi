"""Hand tracking as a teleop input: MediaPipe HandLandmarker + an OpenCV camera.

Your hand is a virtual joystick with a spring centre. Hold an OPEN hand in
the centre circle of the camera view for ARM_S and it arms; from then on
where it sits relative to the centre is the velocity demand - left/right is
Y, up/down is Z. Close your fist, take the hand away, or let the tracker
lose it and it disarms within DROPOUT_S; arming again means centring again.
That interlock is what stops the robot setting off the instant a hand is
seen at the edge of the frame. The open hand IS the deadman.

Depth (X, from the apparent size of the hand) is OFF unless asked for: the
wrist-to-knuckle length changes as much when the hand tilts as when it
moves towards the camera, and a tilt read as a velocity sends the robot
off in a direction nobody asked for - which is exactly what happened the
first time this was tried. With --depth it uses a large deadzone and a low
gain, and is relative to the size when the hand armed.

Record, replay, E-stop and speed come from the keyboard, the same keys as
--input keys, because a gesture is too easy to trigger by accident.

The camera and the model run in their own thread at camera rate (~30 fps,
inference 10-30 ms on a CPU) so the 50 Hz control loop never waits on
them; poll() returns the latest result. frame() returns the latest camera
image with the landmarks, the centre circle and the demand vector drawn on
it; the teleop dashboard shows it as its camera panel. (Deliberately no
separate OpenCV window: one driven from a background thread while
matplotlib owns the main thread is unreliable on Windows.)

All of the decision logic is in HandFilter, which needs no camera, so it
can be checked with synthetic landmarks.

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
DEADZONE = 0.2         # radius of the centre circle, in half-frame units (1 = frame edge)
ARM_S = 0.3            # open hand inside the circle for this long -> armed
DROPOUT_S = 0.2        # no usable hand for this long -> disarmed
MAX_SPEED = 3.0        # frame-widths per second; a centre moving faster is not a hand
                       # (a speed, not a per-frame distance, so a camera dropping to
                       # 15 fps in poor light does not turn a brisk move into a "jump")
SLEW = 0.12            # largest change in any demand per frame (~30 fps): full scale in ~0.3 s
DEPTH_DEADZONE = 0.3   # of the size ratio, when depth is enabled
DEPTH_GAIN = 2.0       # x demand per unit of (size / size_when_armed - 1)
CONFIDENCE = 0.6       # detection / presence / tracking thresholds for the landmarker

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
    wrist-to-middle-knuckle distance.
    """
    wrist = points[WRIST]
    extended = sum(_d(points[tip], wrist) > _d(points[pip], wrist) for tip, pip in FINGERS)
    centre = (sum(points[i][0] for i in PALM) / len(PALM),
              sum(points[i][1] for i in PALM) / len(PALM))
    return extended >= 3, centre, _d(points[0], points[9])


def demand_from(centre, size, ref_size):
    """Hand centre (and size, if ref_size is given) -> (x, y, z) in -1..1.

    Image x runs left to right and is already mirrored, so a hand moved to
    the right gives +Y; image y runs top to bottom, so a hand raised gives
    +Z. Depth, when enabled, is the size ratio against the size when the
    hand armed: bigger (closer to the camera) is +X.
    """
    y = _shape((centre[0] - 0.5) * 2.0, DEADZONE)
    z = _shape((0.5 - centre[1]) * 2.0, DEADZONE)
    x = 0.0
    if ref_size:
        ratio = max(-1.0, min(1.0, (size / ref_size - 1.0) * DEPTH_GAIN))
        x = _shape(ratio, DEPTH_DEADZONE)
    return x, y, z


def _centred(centre):
    return math.hypot((centre[0] - 0.5) * 2.0, (centre[1] - 0.5) * 2.0) < DEADZONE


class HandFilter:
    """Turns a stream of landmark frames into (armed, demand). No camera."""

    def __init__(self, depth=False):
        self.depth = depth
        self.armed = False
        self.demand = (0.0, 0.0, 0.0)
        self.seen = 0.0             # last time a usable hand was seen
        self.state = "no hand"      # for the overlay
        self._arm_since = None
        self._ref_size = None
        self._last_centre = None

    def _disarm(self, state):
        self.armed = False
        self._arm_since = None
        self._last_centre = None
        self.demand = (0.0, 0.0, 0.0)
        self.state = state

    def update(self, points, now):
        """One camera frame. `points` is 21 (x, y) landmarks or None."""
        if points is None:
            if now - self.seen > DROPOUT_S:
                self._disarm("no hand")
            return
        is_open, centre, size = hand_state(points)
        if self._last_centre is not None:
            dt = min(max(now - self.seen, 1.0 / 60.0), DROPOUT_S)
            if _d(centre, self._last_centre) > MAX_SPEED * dt:
                self._disarm("lost it (jump)")     # a skeleton teleporting is not a hand
                return
        self._last_centre = centre
        self.seen = now
        if not is_open:
            self._disarm("fist: stopped")
            return
        if not self.armed:
            if _centred(centre):
                self._arm_since = self._arm_since or now
                if now - self._arm_since >= ARM_S:
                    self.armed = True
                    self._ref_size = size
                    self.state = "ARMED: driving"
                else:
                    self.state = "hold still in the circle to arm"
            else:
                self._arm_since = None
                self.state = "open hand: centre it to arm"
            self.demand = (0.0, 0.0, 0.0)
            return
        target = demand_from(centre, size, self._ref_size if self.depth else None)
        self.demand = tuple(p + max(-SLEW, min(SLEW, t - p)) for p, t in zip(self.demand, target))
        self.state = "ARMED: driving"

    def timed_out(self, now):
        """Called from poll(): the camera thread may have stopped delivering."""
        if now - self.seen > DROPOUT_S and self.armed:
            self._disarm("no hand")
        return now - self.seen > DROPOUT_S


class HandInput:
    NAME = "hand"
    START_SPEED_INDEX = 0       # tracking is noisier than a stick: start at 25 %

    def __init__(self, camera=CAMERA, depth=False):
        self._camera = camera
        self._keys = KeyboardInput()
        self._lock = threading.Lock()
        self._filter = HandFilter(depth=depth)
        self._frame = None            # latest annotated RGB frame, for the dashboard
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
            running_mode=RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=CONFIDENCE,
            min_hand_presence_confidence=CONFIDENCE,
            min_tracking_confidence=CONFIDENCE))
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
                with self._lock:
                    self._filter.update(points, time.time())

                frames += 1
                if time.time() - fps_t >= 1.0:
                    self.fps = frames / (time.time() - fps_t)
                    frames, fps_t = 0, time.time()
                self._draw(cv2, frame, points)
                small = cv2.resize(frame, (480, 360))      # cheap to redraw at 10 Hz
                annotated = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                with self._lock:
                    self._frame = annotated
        finally:
            cap.release()

    def _draw(self, cv2, frame, points):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        with self._lock:
            armed, (dx, dy, dz), state = self._filter.armed, self._filter.demand, self._filter.state
        colour = (0, 200, 0) if armed else (0, 0, 220)
        cv2.circle(frame, (cx, cy), int(DEADZONE * w / 2), (200, 200, 200), 2)
        if points:
            px = [(int(p[0] * w), int(p[1] * h)) for p in points]
            for a, b in CONNECTIONS:
                cv2.line(frame, px[a], px[b], colour, 2)
            for p in px:
                cv2.circle(frame, p, 3, colour, -1)
            _, centre, _ = hand_state(points)
            hand = (int(centre[0] * w), int(centre[1] * h))
            cv2.arrowedLine(frame, (cx, cy), hand, colour, 2)
        cv2.putText(frame, state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        cv2.putText(frame, f"X {dx:+.2f}  Y {dy:+.2f}  Z {dz:+.2f}   {self.fps:.0f} fps",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)

    # -------------------------------------------------------------- poll

    def poll(self) -> Command:
        keys = self._keys.poll()
        alive = self._thread.is_alive() and not self._error
        with self._lock:
            dropped = self._filter.timed_out(time.time())
            x, y, z = self._filter.demand
            armed = self._filter.armed
        if dropped or not alive:
            x, y, z, armed = 0.0, 0.0, 0.0, False
        cmd = Command(x=x, y=y, z=z, deadman=armed, connected=alive)
        cmd.events = keys.events
        return cmd

    def frame(self):
        """Latest annotated camera frame as an RGB array, or None."""
        with self._lock:
            return self._frame

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
