"""Input sources for the teleop demo.

Each source is polled by the control loop and returns a Command: axis demands
in -1..1, whether the deadman is held, and the edge-triggered events (E-stop,
record, replay...) that happened since the last poll. Nothing here knows
about the robot - teleop.py turns a Command into a per-cycle correction.

  XboxInput      an Xbox pad over XInput (xinput1_4.dll ships with Windows)
  KeyboardInput  WASD/QE and the arrows, read globally via GetAsyncKeyState
  SynthInput     a scripted square with record and replay, for dry_run.py

Neither the pad nor the keyboard needs anything installed: both are ctypes
calls into DLLs that are part of Windows.
"""
import ctypes
import time
from dataclasses import dataclass, field


@dataclass
class Command:
    x: float = 0.0          # +X demand, -1..1
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0          # rotation demands
    b: float = 0.0
    deadman: bool = False   # motion is only allowed while this is held
    connected: bool = True  # False = the source has gone away: treat as deadman released
    # Edge-triggered, once per press: estop, reset, record, replay,
    # faster, slower, quit
    events: set = field(default_factory=set)


def _shape(value, deadzone=0.2):
    """Deadzone, then a squared curve so small deflections give fine control."""
    if abs(value) < deadzone:
        return 0.0
    v = (abs(value) - deadzone) / (1.0 - deadzone)
    return v * v if value > 0 else -v * v


# ------------------------------------------------------------------ Xbox

class _XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_ushort),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class _XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint),
                ("Gamepad", _XINPUT_GAMEPAD)]


_XBOX_BUTTONS = {
    "dpad_up": 0x0001, "dpad_down": 0x0002, "dpad_left": 0x0004, "dpad_right": 0x0008,
    "start": 0x0010, "back": 0x0020, "lb": 0x0100, "rb": 0x0200,
    "a": 0x1000, "b": 0x2000, "x": 0x4000, "y": 0x8000,
}
_XBOX_EVENTS = {"b": "estop", "y": "reset", "x": "record", "a": "replay",
                "dpad_up": "faster", "dpad_down": "slower", "back": "quit"}


class XboxInput:
    """Xbox pad via XInput. Works for wired, wireless-adapter and Bluetooth pads.

    Left stick: X (push away) / Y (push left).  Triggers: Z (RT up, LT down).
    Right stick: A / B rotation.  RB held: deadman.
    B: E-stop.  Y: reset E-stop.  X: start/stop recording.  A: replay.
    D-pad up/down: speed.  Back: quit.
    """
    NAME = "xbox"

    def __init__(self, index=0):
        if not hasattr(ctypes, "WinDLL"):
            raise RuntimeError("XInput is Windows-only; use --input keys")
        self._dll = None
        for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self._dll = ctypes.WinDLL(name)
                break
            except OSError:
                continue
        if self._dll is None:
            raise RuntimeError("no XInput DLL found; use --input keys")
        self._index = index
        self._state = _XINPUT_STATE()
        self._prev_buttons = 0

    def poll(self) -> Command:
        if self._dll.XInputGetState(self._index, ctypes.byref(self._state)) != 0:
            self._prev_buttons = 0
            return Command(connected=False)
        g = self._state.Gamepad
        cmd = Command(connected=True)
        cmd.x = _shape(g.sThumbLY / 32767.0)
        cmd.y = _shape(-g.sThumbLX / 32767.0)
        cmd.z = (g.bRightTrigger - g.bLeftTrigger) / 255.0
        cmd.a = _shape(g.sThumbRX / 32767.0)
        cmd.b = _shape(g.sThumbRY / 32767.0)
        buttons = g.wButtons
        cmd.deadman = bool(buttons & _XBOX_BUTTONS["rb"])
        pressed = buttons & ~self._prev_buttons
        self._prev_buttons = buttons
        for button, event in _XBOX_EVENTS.items():
            if pressed & _XBOX_BUTTONS[button]:
                cmd.events.add(event)
        return cmd


# -------------------------------------------------------------- Keyboard

_VK = {"W": 0x57, "S": 0x53, "A": 0x41, "D": 0x44, "Q": 0x51, "E": 0x45,
       "LEFT": 0x25, "UP": 0x26, "RIGHT": 0x27, "DOWN": 0x28,
       "SPACE": 0x20, "ESC": 0x1B, "R": 0x52, "TAB": 0x09, "P": 0x50,
       "PLUS": 0xBB, "MINUS": 0xBD, "BACKSPACE": 0x08}
_KEY_EVENTS = {"ESC": "estop", "R": "reset", "TAB": "record", "P": "replay",
               "PLUS": "faster", "MINUS": "slower", "BACKSPACE": "quit"}


class KeyboardInput:
    """Keyboard via GetAsyncKeyState - key state, not key presses, so a held
    key is a held demand and letting go stops.

    W/S: +/-X.  A/D: +/-Y.  Q/E: +/-Z.  Arrows: A/B rotation.
    SPACE held: deadman.  ESC: E-stop.  R: reset.  TAB: record.  P: replay.
    +/-: speed.  BACKSPACE: quit.

    Keys are read globally, whatever window has focus - so is the deadman.
    Hold SPACE only when you mean it.
    """
    NAME = "keys"

    def __init__(self):
        if not hasattr(ctypes, "windll"):
            raise RuntimeError("GetAsyncKeyState is Windows-only")
        self._user32 = ctypes.windll.user32
        self._prev = set()

    def _held(self, key):
        return bool(self._user32.GetAsyncKeyState(_VK[key]) & 0x8000)

    def poll(self) -> Command:
        held = {k for k in _VK if self._held(k)}
        cmd = Command(connected=True)
        cmd.x = float("W" in held) - float("S" in held)
        cmd.y = float("A" in held) - float("D" in held)
        cmd.z = float("Q" in held) - float("E" in held)
        cmd.a = float("RIGHT" in held) - float("LEFT" in held)
        cmd.b = float("UP" in held) - float("DOWN" in held)
        cmd.deadman = "SPACE" in held
        for key, event in _KEY_EVENTS.items():
            if key in held and key not in self._prev:
                cmd.events.add(event)
        self._prev = held
        return cmd


# ------------------------------------------------------------- Synthetic

class SynthInput:
    """A scripted run, for dry_run.py and for checking the whole pipeline
    without a controller: deadman on, start recording, drive a square
    (+X, +Y, -X, -Y), stop recording, replay it, and quit once the replay
    has finished. teleop.py calls notify("replay_done") when it has.
    """
    NAME = "synth"

    def __init__(self, leg_seconds=0.6):
        self._t0 = None
        self._leg = leg_seconds
        self._fired = set()
        self._replay_done = False

    def notify(self, what):
        if what == "replay_done":
            self._replay_done = True

    def _once(self, cmd, latch, event=None):
        """Emit `event` (default: the latch name) the first time only."""
        if latch not in self._fired:
            self._fired.add(latch)
            cmd.events.add(event or latch)

    def poll(self) -> Command:
        if self._t0 is None:
            self._t0 = time.time()
        t = time.time() - self._t0
        L = self._leg
        cmd = Command(connected=True, deadman=True)
        if t < 0.3:
            pass                                    # settle
        elif t < 0.3 + 4 * L:
            self._once(cmd, "record_start", "record")
            leg = int((t - 0.3) / L)
            cmd.x, cmd.y = ((1, 0), (0, 1), (-1, 0), (0, -1))[leg]
        elif t < 0.6 + 4 * L:
            self._once(cmd, "record_stop", "record")
        else:
            cmd.deadman = False
            self._once(cmd, "replay")
            if self._replay_done:
                self._once(cmd, "quit")
        return cmd


def make_input(name):
    return {"xbox": XboxInput, "keys": KeyboardInput, "synth": SynthInput}[name]()
