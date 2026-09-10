"""Hand tracking as a teleop input: MediaPipe HandLandmarker + an OpenCV camera.

Your hand is a virtual joystick with a spring centre. Hold an OPEN hand in
the centre circle of the camera view for ARM_S and it arms; from then on
where it sits relative to the centre is the velocity demand - left/right is
Y, up/down is Z. Close your fist, take the hand away, or let the tracker
lose it and it disarms within DROPOUT_S; arming again means centring again.
That interlock is what stops the robot setting off the instant a hand is
seen at the edge of the frame. The open hand IS the deadman.

Depth (X) comes from the apparent size of the palm relative to its size
when the hand armed: push towards the camera for +X, pull back for -X. The
first version used one dimension (wrist to middle knuckle) and a tilted
hand read as a size change, which sent the robot off in a direction nobody
asked for. Now the size is the LARGER of two orthogonal palm dimensions -
wrist to middle knuckle, and index to little knuckle: a tilt about either
axis shrinks one of them but not the other, moving closer grows both, and
rolling the hand in the image plane changes neither. A wide deadzone on top.
--no-depth turns X off if it still misbehaves in the room on the day.

Orientation: the tool copies the palm. Roll (the hand turned in the image
plane, wrist-to-knuckle line against vertical) is measured directly and is
solid; pitch (fingers towards or away from the camera) and sideways tilt
come from MediaPipe's per-landmark depth, which is rougher, so all three
have a deadzone and are smoothed. Angles are relative to the hand's
orientation when it armed and go out as TARGETS (Command.orient), which
teleop.py position-controls within its rotation fence - so hold your hand
at an angle and the tool settles at that angle. Which robot axis each one
should drive, and with which sign, depends on where the camera stands
relative to the robot: ORIENT_SIGNS below flips any of them. --no-orient
turns this off.

Two gestures drive the gripper, each held for GESTURE_S so a passing shape
does nothing: a Vulcan salute (index and middle together, ring and little
together, a gap between the pairs) opens it; an open palm with the fingers
together closes it. A splayed hand is neutral and a fist is still "stop".

Record, replay, E-stop and speed come from the keyboard, the same keys as
--input keys, because a gesture is too easy to trigger by accident.

The camera and the model run in their own PROCESS at camera rate (~30 fps,
inference 10-30 ms on a CPU): a thread was starved by matplotlib's 3D
redraw in the main process and its frame rate halved. poll() returns the
latest result it has sent. frame() returns the latest camera
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
import json
import math
import multiprocessing
import os
import queue
import time
import urllib.request
from pathlib import Path

from inputs import Command, KeyboardInput, _shape

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")
MODEL_PATH = Path(__file__).resolve().parent / "models" / "hand_landmarker.task"

CAMERA = 0
DEADZONE = 0.2         # radius of the centre circle, in half-frame units (1 = frame edge)
MM_PER_FRAME = 500.0   # hand moved across the whole frame width = this far, in mm (about 1:1
                       # for a webcam 60 cm away); the teleop fence caps it at +/-40 mm
DEPTH_MM = 300.0       # X per unit of (palm size ratio - 1): 20 % bigger = 60 mm closer
DEPTH_RATIO_DEADZONE = 0.08
POS_SMOOTH = 0.3       # EMA weight of the newest frame on the position target
ARM_S = 0.3            # open hand inside the circle for this long -> armed
DROPOUT_S = 0.2        # no usable hand for this long -> disarmed
MAX_SPEED = 3.0        # frame-widths per second; a centre moving faster is not a hand
                       # (a speed, not a per-frame distance, so a camera dropping to
                       # 15 fps in poor light does not turn a brisk move into a "jump")
SLEW = 0.12            # largest change in any demand per frame (~30 fps): full scale in ~0.3 s
DEPTH_DEADZONE = 0.25  # on the gained size ratio: below a ~14 % size change nothing happens
DEPTH_GAIN = 1.8       # x demand per unit of (size / size_when_armed - 1): +55 % = full ahead
CONFIDENCE = 0.6       # detection / presence / tracking thresholds for the landmarker
ORIENT_DEADZONE_DEG = 12.0  # palm angles inside this of the armed orientation ask for nothing
                            # (pitch/tilt come from MediaPipe depth and wobble several degrees)
ORIENT_MAX_DEG = 30.0       # target clamp (teleop fences at the same figure)
ORIENT_SMOOTH = 0.3         # EMA weight of the newest frame - depth-derived angles are noisy
ORIENT_SIGNS = (1.0, 0.0, 0.0)   # (roll -> A, pitch -> B, tilt -> C): sign, or 0 to disable an axis.
                                 # Pitch and tilt come from MediaPipe depth and drifted when the hand
                                 # rolled - on the robot that nosed the tool forward. Roll only for now.
GESTURE_S = 0.4        # a gripper gesture must be held this long before it counts
VULCAN_GAP = 0.8       # middle-to-ring fingertip gap, as a fraction of palm width (Adam: 1.2)
VULCAN_RATIO = 1.5     # ... and at least this many times the other two gaps (his ring-little: 0.65)
TOGETHER_GAP = 0.5     # index-middle AND middle-ring fingertip gaps below this = fingers together.
                       # The ring-little gap is ignored: the little finger's tip sits lower, so it
                       # reads ~0.65 even when pressed (Adam: pressed 0.44/0.35, relaxed 0.57/0.56)

# Per-person thresholds written by gesture_calibrate.py override the three
# defaults above. Loaded here so the tracker process picks them up too.
GESTURES_FILE = Path(__file__).resolve().parent / "gestures.json"
if GESTURES_FILE.exists():
    try:
        _cal = json.loads(GESTURES_FILE.read_text())
        TOGETHER_GAP = float(_cal.get("TOGETHER_GAP", TOGETHER_GAP))
        VULCAN_GAP = float(_cal.get("VULCAN_GAP", VULCAN_GAP))
        VULCAN_RATIO = float(_cal.get("VULCAN_RATIO", VULCAN_RATIO))
    except (ValueError, OSError):
        pass

WRIST = 0
PALM = (0, 5, 9, 13, 17)                           # wrist + the four MCP knuckles
FINGERS = ((8, 6), (12, 10), (16, 14), (20, 18))   # (tip, pip): index .. little
CONNECTIONS = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
               (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
               (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _square(points, aspect):
    """Landmarks with x scaled by the frame's aspect ratio, so distances are
    isotropic. Without this a palm measured 33 % bigger when it was rolled
    so its width ran vertically (x is normalised to 640 px, y to 480), and
    the depth axis read that as "closer" and drove +X. Seen on the robot."""
    return [(p[0] * aspect, p[1]) + tuple(p[2:]) for p in points]


def hand_state(points, aspect=4.0 / 3.0):
    """From 21 normalised (x, y) landmarks: (is_open, centre, size).

    A finger counts as extended when its tip is further from the wrist than
    its middle joint; three or more extended is an open hand. Size is a
    pair of orthogonal palm dimensions: wrist to middle knuckle (length)
    and index to little knuckle (width), measured isotropically. The centre
    stays in normalised frame units (for the arming circle and the position
    mapping).
    """
    sq = _square(points, aspect)
    wrist = sq[WRIST]
    extended = sum(_d(sq[tip], wrist) > _d(sq[pip], wrist) for tip, pip in FINGERS)
    centre = (sum(points[i][0] for i in PALM) / len(PALM),
              sum(points[i][1] for i in PALM) / len(PALM))
    size = (_d(sq[0], sq[9]), _d(sq[5], sq[17]))
    return extended >= 3, centre, size


def hand_orientation(points, aspect=4.0 / 3.0):
    """Palm roll, pitch and tilt in degrees from 21 (x, y, z) landmarks.

    x and y are normalised to the image width and height, so x is scaled
    by `aspect` first to make the geometry square. MediaPipe's z is depth
    relative to the wrist on roughly x's scale, smaller = closer.

      roll   wrist-to-middle-knuckle line against the image vertical;
             0 with the fingers pointing up, positive turned clockwise
      pitch  positive when the knuckles come towards the camera
      tilt   positive when the little-finger side is further away
    """
    w, m = points[0], points[9]
    dx, dy = (m[0] - w[0]) * aspect, m[1] - w[1]
    roll = math.degrees(math.atan2(dx, -dy))
    length = math.hypot(dx, dy) or 1e-6
    pitch = math.degrees(math.atan2((w[2] - m[2]) * aspect, length))
    i, l = points[5], points[17]
    width = math.hypot((l[0] - i[0]) * aspect, l[1] - i[1]) or 1e-6
    tilt = math.degrees(math.atan2((l[2] - i[2]) * aspect, width))
    return roll, pitch, tilt


def fingertip_gaps(points, aspect=4.0 / 3.0):
    """(index-middle, middle-ring, ring-little) fingertip gaps as fractions of palm width."""
    sq = _square(points, aspect)
    width = _d(sq[5], sq[17]) or 1e-6
    return (_d(sq[8], sq[12]) / width, _d(sq[12], sq[16]) / width, _d(sq[16], sq[20]) / width)


def gripper_gesture(points, aspect=4.0 / 3.0):
    """'vulcan', 'together' or None, from the fingertip gaps."""
    g1, g2, g3 = fingertip_gaps(points, aspect)
    if g2 > VULCAN_GAP and g2 > VULCAN_RATIO * max(g1, g3):
        return "vulcan"
    if max(g1, g2) < TOGETHER_GAP:
        return "together"
    return None


def size_ratio(size, ref):
    """How much bigger the palm looks than when it armed.

    The larger of the two dimension ratios: a tilt about either axis
    foreshortens one dimension but not the other, so it leaves the max
    alone, while a real change of distance moves both.
    """
    return max(size[0] / ref[0], size[1] / ref[1])


def demand_from(centre, ratio=None):
    """Hand centre (and palm size ratio, if given) -> (x, y, z) in -1..1.

    Image x runs left to right and is already mirrored, so a hand moved to
    the right gives +Y; image y runs top to bottom, so a hand raised gives
    +Z. Depth is the size ratio against the size when the hand armed:
    bigger (closer to the camera) is +X.
    """
    y = _shape((centre[0] - 0.5) * 2.0, DEADZONE)
    z = _shape((0.5 - centre[1]) * 2.0, DEADZONE)
    x = 0.0
    if ratio is not None:
        x = _shape(max(-1.0, min(1.0, (ratio - 1.0) * DEPTH_GAIN)), DEPTH_DEADZONE)
    return x, y, z


def _centred(centre):
    return math.hypot((centre[0] - 0.5) * 2.0, (centre[1] - 0.5) * 2.0) < DEADZONE


class HandFilter:
    """Turns a stream of landmark frames into (armed, demand). No camera."""

    def __init__(self, depth=True, orient=False):
        self.depth = depth
        self.orient_on = orient
        self.orient_target = None   # (A, B, C) degrees from the start pose while armed
        self.pos_target = None      # (X, Y, Z) mm since arming while armed
        self._ref_orient = None
        self._ref_centre = None
        self.armed = False
        self.demand = (0.0, 0.0, 0.0)
        self.seen = 0.0             # last time a usable hand was seen
        self.state = "no hand"      # for the overlay
        self.gesture = None         # 'vulcan' / 'together' / None, for the overlay
        self.gaps = None            # last fingertip gaps, for the probe
        self.events = set()         # gripper events for poll() to drain
        self._arm_since = None
        self._ref_size = None
        self._last_centre = None
        self._gesture_since = None
        self._gesture_held = None
        self._gesture_sent = None   # the last gesture that fired, so it fires once per hold

    def _disarm(self, state):
        self.armed = False
        self._arm_since = None
        self._last_centre = None
        self.demand = (0.0, 0.0, 0.0)
        self.orient_target = None
        self.pos_target = None
        self.state = state
        self._gestures(None, 0.0)

    def _gestures(self, gesture, now):
        """Fire grip_open / grip_close once a gesture has been held GESTURE_S."""
        self.gesture = gesture
        if gesture is None:
            self._gesture_since = None
            self._gesture_sent = None   # a neutral hand in between lets the same gesture fire again
            return
        if self._gesture_since is None or gesture != self._gesture_held:
            self._gesture_since, self._gesture_held = now, gesture
        elif now - self._gesture_since >= GESTURE_S and gesture != self._gesture_sent:
            self._gesture_sent = gesture
            self.events.add("grip_open" if gesture == "vulcan" else "grip_close")

    def update(self, points, now, aspect=4.0 / 3.0):
        """One camera frame. `points` is 21 (x, y[, z]) landmarks or None."""
        if points is None:
            if now - self.seen > DROPOUT_S:
                self._disarm("no hand")
            return
        is_open, centre, size = hand_state(points, aspect)
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
        self.gaps = tuple(round(g, 2) for g in fingertip_gaps(points, aspect))
        self._gestures(gripper_gesture(points, aspect), now)
        if not self.armed:
            if _centred(centre):
                self._arm_since = self._arm_since or now
                if now - self._arm_since >= ARM_S:
                    self.armed = True
                    self._ref_size = size
                    self._ref_centre = centre
                    self.pos_target = (0.0, 0.0, 0.0)
                    self._ref_orient = hand_orientation(points, aspect) if len(points[0]) > 2 else None
                    self.orient_target = (0.0, 0.0, 0.0) if self.orient_on and self._ref_orient else None
                    self.state = "ARMED: following"
                else:
                    self.state = "hold still in the circle to arm"
            else:
                self._arm_since = None
                self.state = "open hand: centre it to arm"
            self.demand = (0.0, 0.0, 0.0)
            return
        # Position: the tool follows the hand's displacement since arming.
        # Image x is mirrored so hand-right is +Y; image y runs downward so
        # hand-up is +Z; a bigger palm (closer to the camera) is +X.
        y = (centre[0] - self._ref_centre[0]) * MM_PER_FRAME
        z = (self._ref_centre[1] - centre[1]) * MM_PER_FRAME / aspect   # same mm per pixel as Y
        x = 0.0
        if self.depth:
            r = size_ratio(size, self._ref_size) - 1.0
            if abs(r) > DEPTH_RATIO_DEADZONE:
                x = (r - math.copysign(DEPTH_RATIO_DEADZONE, r)) * DEPTH_MM
        self.pos_target = tuple(p + POS_SMOOTH * (w - p) for p, w in zip(self.pos_target, (x, y, z)))
        self.demand = (0.0, 0.0, 0.0)
        if self.orient_target is not None:
            angles = hand_orientation(points, aspect)
            wanted = []
            for angle, ref, sign in zip(angles, self._ref_orient, ORIENT_SIGNS):
                delta = (angle - ref + 180.0) % 360.0 - 180.0
                # Deadzone that starts from zero rather than stepping.
                delta = 0.0 if abs(delta) < ORIENT_DEADZONE_DEG else delta - math.copysign(ORIENT_DEADZONE_DEG, delta)
                wanted.append(sign * max(-ORIENT_MAX_DEG, min(ORIENT_MAX_DEG, delta)))
            self.orient_target = tuple(p + ORIENT_SMOOTH * (w - p) for p, w in zip(self.orient_target, wanted))
        self.state = "ARMED: following"

    def timed_out(self, now):
        """Called from poll(): the camera thread may have stopped delivering."""
        if now - self.seen > DROPOUT_S and self.armed:
            self._disarm("no hand")
        return now - self.seen > DROPOUT_S


def _draw(cv2, frame, points, filt, fps):
    """Annotate a BGR frame in place: skeleton, centre circle, arrow, text."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    dx, dy, dz = filt.pos_target if filt.pos_target else (0.0, 0.0, 0.0)
    colour = (0, 200, 0) if filt.armed else (0, 0, 220)
    cv2.circle(frame, (cx, cy), int(DEADZONE * w / 2), (200, 200, 200), 2)
    if points:
        px = [(int(p[0] * w), int(p[1] * h)) for p in points]
        for a, b in CONNECTIONS:
            cv2.line(frame, px[a], px[b], colour, 2)
        for p in px:
            cv2.circle(frame, p, 3, colour, -1)
        _, centre, _ = hand_state(points)
        cv2.arrowedLine(frame, (cx, cy), (int(centre[0] * w), int(centre[1] * h)), colour, 2)
    cv2.putText(frame, filt.state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
    if filt.gesture:
        label = {"vulcan": "vulcan salute: gripper OPEN",
                 "together": "fingers together: gripper CLOSE"}[filt.gesture]
        cv2.putText(frame, label, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    if filt.orient_target is not None:
        o = filt.orient_target
        cv2.putText(frame, f"tool A {o[0]:+.0f}  B {o[1]:+.0f}  C {o[2]:+.0f} deg",
                    (10, h - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
    cv2.putText(frame, f"target X {dx:+.0f}  Y {dy:+.0f}  Z {dz:+.0f} mm   {fps:.0f} fps",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)


def _put_latest(q, item):
    """Non-blocking put that drops the item when the queue is full. The
    worker must never wait on the parent: a pipe did, and a parent busy in
    a prompt or a redraw dragged the tracker down to 1 fps."""
    try:
        q.put_nowait(item)
    except queue.Full:
        pass


def _camera_worker(state_q, frame_q, stop, camera, depth, orient, model_path):
    """The tracker process: camera + landmarker + HandFilter.

    Puts ("state", dict) on state_q every frame and an annotated RGB frame
    on frame_q every third, ("error", text) on state_q if it cannot start;
    stops when `stop` is set. Its own process rather than a thread because
    matplotlib's 3D redraw in the main process holds the interpreter lock
    long enough to halve a thread's frame rate - seen on the demo laptop.
    """
    os.environ.setdefault("GLOG_minloglevel", "2")
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
    try:
        import cv2
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions
        from mediapipe.tasks.python.vision import (HandLandmarker, HandLandmarkerOptions,
                                                   RunningMode)
    except ImportError as e:
        state_q.put(("error", f"{e} - run this with .venv-demo (see examples/teleop/README.md)"))
        return
    landmarker = HandLandmarker.create_from_options(HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=RunningMode.VIDEO, num_hands=1,
        min_hand_detection_confidence=CONFIDENCE,
        min_hand_presence_confidence=CONFIDENCE,
        min_tracking_confidence=CONFIDENCE))
    cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        state_q.put(("error", f"camera {camera} did not open"))
        return
    # Pin a cheap mode: 640x480 MJPEG at 30 fps (27 fps as-is, 30 pinned,
    # measured on the demo laptop).
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    filt = HandFilter(depth=depth, orient=orient)
    t0 = time.time()
    last_ms = -1
    frames, fps_t, fps, n = 0, time.time(), 0.0, 0
    try:
        while not stop.is_set():
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
                points = [(lm.x, lm.y, lm.z) for lm in result.hand_landmarks[0]]
            now = time.time()
            filt.update(points, now, aspect=frame.shape[1] / frame.shape[0])

            frames += 1
            if now - fps_t >= 1.0:
                fps, frames, fps_t = frames / (now - fps_t), 0, now
            # Gripper events must not be lost to a full queue: keep them until
            # a state message actually goes out.
            pending = sorted(filt.events)
            try:
                state_q.put_nowait(("state", {
                    "armed": filt.armed, "pos": filt.pos_target, "orient": filt.orient_target,
                    "state": filt.state, "gesture": filt.gesture, "gaps": filt.gaps, "seen": filt.seen,
                    "events": pending, "fps": fps}))
                filt.events = set()
            except queue.Full:
                pass
            n += 1
            if n % 3 == 0:                                  # ~10 Hz is plenty for a preview
                _draw(cv2, frame, points, filt, fps)
                _put_latest(frame_q, cv2.cvtColor(cv2.resize(frame, (480, 360)), cv2.COLOR_BGR2RGB))
    finally:
        cap.release()
        state_q.cancel_join_thread()
        frame_q.cancel_join_thread()


class HandInput:
    NAME = "hand"
    START_SPEED_INDEX = 0       # tracking is noisier than a stick: start at 25 %

    def __init__(self, camera=CAMERA, depth=True, orient=False):
        self._keys = KeyboardInput()
        self._ensure_model()
        self._state = None
        self._frame = None
        self._events = set()
        self._error = None
        self.fps = 0.0
        self._state_q = multiprocessing.Queue(maxsize=8)
        self._frame_q = multiprocessing.Queue(maxsize=1)
        self._stop = multiprocessing.Event()
        self._proc = multiprocessing.Process(
            target=_camera_worker,
            args=(self._state_q, self._frame_q, self._stop, camera, depth, orient, str(MODEL_PATH)),
            daemon=True, name="hand-tracker")
        self._proc.start()
        deadline = time.time() + 30.0                       # the model load takes a moment
        while time.time() < deadline:
            self._drain(block=0.1)
            if self._error:
                raise RuntimeError(self._error)
            if self._state is not None:
                return
        raise RuntimeError("hand tracker did not start")

    @staticmethod
    def _ensure_model():
        if MODEL_PATH.exists():
            return
        MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        print(f"fetching the hand landmarker model (about 8 MB) -> {MODEL_PATH}")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)

    def _drain(self, block=0.0):
        """Take everything the tracker has sent; keep the latest of each kind."""
        try:
            while True:
                kind, payload = self._state_q.get(timeout=block) if block else self._state_q.get_nowait()
                block = 0.0
                if kind == "state":
                    self._state = payload
                    self._events |= set(payload["events"])
                    self.fps = payload["fps"]
                elif kind == "error":
                    self._error = payload
        except queue.Empty:
            pass
        try:
            while True:
                self._frame = self._frame_q.get_nowait()
        except queue.Empty:
            pass

    def poll(self) -> Command:
        keys = self._keys.poll()
        self._drain()
        s = self._state or {}
        alive = self._proc.is_alive() and not self._error
        # The tracker stamps every state with the time it last saw a usable
        # hand; if that is stale (hand gone, or the tracker itself stalled)
        # the deadman is released here, whatever the last state said.
        armed = bool(s.get("armed")) and alive and (time.time() - s.get("seen", 0.0) <= DROPOUT_S)
        cmd = Command(deadman=armed, connected=alive,
                      pos=s.get("pos") if armed else None,
                      orient=s.get("orient") if armed else None)
        cmd.events = keys.events | self._events
        self._events = set()
        return cmd

    def frame(self):
        """Latest annotated camera frame as an RGB array, or None."""
        self._drain()
        return self._frame

    def close(self):
        self._stop.set()
        self._proc.join(timeout=3.0)
        if self._proc.is_alive():
            self._proc.terminate()
