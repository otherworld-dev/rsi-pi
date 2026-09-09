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

## Adding an input

`inputs.py` defines `Command` (axis demands in −1..1, `deadman`,
`connected`, and edge events) and three sources. A new one needs a `poll()`
returning a `Command` and a `NAME`; nothing else changes. A MediaPipe hand
tracker is the intended next one — its dependencies live in a separate,
disposable environment (`.venv-demo`, see the repo root) so the library and
its release never see them.
