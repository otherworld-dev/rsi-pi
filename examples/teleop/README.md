# Teleop: drive the robot with an Xbox pad, record the path, replay it

RSI's relative mode *is* velocity control — whatever sits in `RKorr` is
re-applied every 4 ms cycle — so a stick deflection mapped to
millimetres-per-cycle is a velocity joystick with no extra maths. That is all
`teleop.py` does, at 50 Hz from a background thread, with record and replay
on top: teach a path by hand, and the trajectory executor plays it back at
the pace it was taught and reports how far it strayed.

```
python examples/teleop/teleop.py                        # Xbox pad (wired, adapter or Bluetooth)
python examples/teleop/teleop.py --input keys           # keyboard
python examples/teleop/teleop.py --replay logs/teleop_20260910_101500.csv
python examples/dry_run.py examples/teleop/teleop.py --context joints
```

Nothing to install: the pad is read through XInput and the keyboard through
`GetAsyncKeyState`, both DLLs that ship with Windows; the dashboard is
matplotlib, already a dependency. Under `dry_run.py` the script drives a
scripted square, records it, replays it and prints PASS/FAIL on the
deviation, so the whole pipeline is checked without a controller.

## Controls

| | Xbox | Keyboard |
|---|---|---|
| X (away / towards) | left stick up/down | W / S |
| Y (left / right) | left stick left/right | A / D |
| Z (up / down) | RT / LT | Q / E |
| A, B rotation | right stick | arrows |
| **Deadman** (hold to move) | **RB** | **SPACE** |
| E-stop | B | ESC |
| Reset E-stop | Y | R |
| Start / stop recording | X | TAB |
| Replay | A | P |
| Speed 25 / 50 / 100 % | D-pad up / down | + / − |
| Quit | Back | BACKSPACE |

Small deflections are squared, so the first half of the stick is fine
control. Speed starts at 50 %: full stick = 50 mm/s, 100 mm/s at 100 %.

The keyboard is read globally, whatever window has focus — including the
deadman. Hold SPACE only when you mean it.

## Dashboard

A 3D plot of the path (blue), the recording (orange) and the replay
(green) inside a wireframe of the ±40 mm fence cube, with the current
position in red; a status panel (mode, deadman, speed, demand, offset from
the start pose, fence state, cycle time and jitter, last replay's
deviation); and, with the hand tracker, the camera panel. Drag the 3D axes
to rotate them. The window closing stops the script cleanly.

## Safety layers, innermost first

1. `max_cartesian_rate` caps every per-cycle correction at 0.4 mm **in the
   network process**, whatever the script asks for.
2. The Joints context's `POSCORR` limits (±50 mm) clamp the total correction
   on the controller itself.
3. A soft fence at ±40 mm from the start pose stops the script pushing
   further out before that clamp engages (coming back is always allowed).
4. The deadman: no motion unless it is held, and a lost controller counts as
   released.
5. E-stop from the pad freezes corrections at zero until reset.
6. The control loop is watched: if it stalls, the dashboard zeros the
   corrections.

And the usual `confirm()` prompt before any of it is enabled.

## Record and replay

Recording samples the actual pose (`RIst`) and the robot's IPOC every
control tick (50 Hz) and saves them to `logs/teleop_<timestamp>.csv` when
stopped. Replay returns the robot to where the recording began (75 mm/s),
then turns the samples into **per-cycle deltas**: the IPOC difference
between two samples is exactly how many robot cycles passed, so each
sample-to-sample move is spread evenly over that many cycles and sent
through `execute_trajectory(points="delta")`, the exactly-once path. The
replay runs in robot-cycle time at the recorded pace, every delta is one
the robot already accepted, and the 0.4 mm per-cycle clamp never engages —
which it silently would if 50 Hz world poses were replayed directly, as
the first version of this did.

The report is the distance from each replayed sample to the nearest
recorded point, so clock drift between the two runs doesn't count as error.
On the emulator it is a fraction of a millimetre; on the robot it is the
figure worth writing down.

One emulator artefact to know about: the echo server sends its next packet
as soon as the reply to the last one arrives, so its cycle rate floats with
reply latency and a replay's wall-clock pace can differ from the recording's
(the geometry is unaffected). A real controller's cycle is fixed, and there
"cycle for cycle" means "at the recorded pace".

## Hand tracking

`--input hand` makes your hand the joystick, through a webcam and
MediaPipe's hand landmarker. Its dependencies live in a separate,
disposable environment so the library, `.venv` and the release never see
them:

```
py -3.14 -m venv .venv-demo
.venv-demo\Scripts\pip install -e . mediapipe opencv-contrib-python
.venv-demo\Scripts\python examples\teleop\teleop.py --input hand
```

The ~8 MB landmarker model is fetched once into `examples/teleop/models/`
(gitignored). Then:

| Hand | Robot |
|---|---|
| **Open hand held in the centre circle for 0.3 s** | armed — the deadman is held |
| Move left / right of the circle | Y |
| Raise / lower | Z |
| Fist, hand out of view, or tracking lost | disarmed within 0.2 s — centre it again to re-arm |
| Push towards / pull from the camera | X (about +40 % palm size = full ahead; `--no-depth` disables) |

The hand is a joystick with a spring centre and an interlock: nothing
moves until an open hand has sat still in the centre circle for 0.3 s, so
a hand appearing at the edge of the frame cannot set the robot off, and
your idea of "centre" is calibrated to the camera's before anything
happens. Demands are slew-limited (full scale in about 0.3 s), the hand
input starts at 25 % speed, and a skeleton that jumps across the frame in
one frame is treated as lost.

Depth (X) is the palm's apparent size relative to its size when the hand
armed. The first version used a single dimension and learnt the hard way
that a *tilted* hand looks smaller too, which sent the robot off in a
direction nobody asked for. It now takes the **larger of two orthogonal
palm dimensions** (wrist→middle knuckle and index→little knuckle): a tilt
about either axis shrinks one but not the other, moving closer grows both,
and rolling the hand in the image plane changes neither. There is a wide
deadzone on top (about 14 % before anything happens), and roughly a
hand-and-a-half closer is full speed ahead. Arm with a flat palm facing
the camera — that is the size everything is measured against. If it feels
too eager or too lazy, `DEPTH_GAIN` and `DEPTH_DEADZONE` at the top of
`hand_input.py` are the knobs; `--no-depth` turns X off.

The dashboard gains a camera panel showing what is being tracked: the
landmarks (green = armed, red = not), the centre circle, the demand vector,
the arming state and the tracker's frame rate. There is deliberately no
separate OpenCV window — one driven from a background thread while
matplotlib owns the main thread is unreliable on Windows, and a stalled
tracker means a dropped deadman. Record, replay, E-stop, reset and speed
are the **keyboard** keys from the table above — a gesture is too easy to
trigger by accident.

What it needs on the day: even light on the hand, a plain-ish background,
and nobody else's hand in the frame (only one is tracked; the first found
wins). Camera + inference run in their own thread at ~30 fps, so the
control loop never waits on them; if the preview's fps drops into the
teens the laptop is struggling and the deadman drop-out will feel abrupt.

## Adding an input

`inputs.py` defines `Command` (axis demands in −1..1, `deadman`,
`connected`, and edge events) and the pad, keyboard and synthetic sources;
`hand_input.py` adds the tracker. A new one needs a `poll()` returning a
`Command` and a `NAME`, optionally a `close()`; nothing else changes.

## Driving the emulator

`dry_run.py` forwards arguments it doesn't recognise, so the same script
runs against the echo server with a real input:

```
python examples/dry_run.py examples/teleop/teleop.py --context joints --input xbox --timeout 3600
.venv-demo\Scripts\python examples\dry_run.py examples\teleop\teleop.py --context joints --input hand --timeout 3600
```

The console is captured by `dry_run.py`; the dashboard (and the camera
preview) are the interface.
