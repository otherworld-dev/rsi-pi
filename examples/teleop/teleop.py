"""Drive the robot with an Xbox pad, record the path, replay it.

RSI's relative mode IS velocity control: whatever sits in RKorr is re-applied
every 4 ms cycle. So a stick deflection mapped to millimetres-per-cycle gives
a proper velocity joystick with no extra maths - that is all this does, at
50 Hz, from a background thread. Letting go of the stick (or the deadman)
writes zeros and the robot stops within a cycle.

Record and replay: while recording, the actual pose (RIst) and the robot's
IPOC are sampled every control tick. Replay first returns the robot to
where the recording began, then turns the samples into per-cycle deltas -
the IPOC says exactly how many robot cycles passed between two samples, so
each sample-to-sample move is spread evenly over that many cycles - and
plays them through the executor's exactly-once path (points="delta"). The
replay therefore runs in robot-cycle time at the recorded pace, every delta
is one the robot already accepted, and the per-cycle clamp never engages.
Afterwards it reports how far the replayed path strayed from the recording.

Safety layers, innermost first:
  - max_cartesian_rate caps every per-cycle correction at MAX_STEP_MM in
    the network process itself, whatever this script asks for
  - the Joints context's POSCORR limits (+/-50 mm) clamp the total
    correction on the controller
  - a soft fence at FENCE_MM stops this script pushing further out before
    that clamp engages
  - the deadman: no motion unless it is held, and a lost controller counts
    as released
  - E-stop from the pad (B) freezes the corrections at zero until reset (Y)
  - the control loop is watched: if it stalls, the dashboard zeros the
    corrections

Needs context("joints") (the default): Cartesian corrections and RIst only.

    python examples/teleop/teleop.py                        # Xbox pad
    python examples/teleop/teleop.py --input keys           # keyboard
    .venv-demo/Scripts/python examples/teleop/teleop.py --input hand   # webcam, see hand_input.py
    python examples/teleop/teleop.py --replay logs/teleop_....csv
    python examples/dry_run.py examples/teleop/teleop.py --context joints
        # scripted square, recorded and replayed, with a PASS/FAIL
    python examples/dry_run.py examples/teleop/teleop.py --context joints --input xbox --timeout 3600
        # drive the emulator with the pad; the dashboard shows the result

Controls are listed in inputs.py and README.md.
"""
import argparse
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # examples/_confirm.py
from _confirm import assume_yes, confirm
from inputs import Command, make_input

from RSIPI import RSIAPI, context
from RSIPI.exceptions import RSIError

MAX_STEP_MM = 0.4       # mm per cycle at full demand, 100 % speed: 100 mm/s at 4 ms
MAX_ROT_DEG = 0.04      # deg per cycle: 10 deg/s
FENCE_MM = 40.0         # soft fence on X/Y/Z from the start pose (POSCORR clamps at 50)
SPEED_SCALES = (0.25, 0.5, 1.0)
CONTROL_HZ = 50
RETURN_STEP_MM = 0.3    # per-cycle step when returning to a recording's start: 75 mm/s
STALE_S = 0.25          # control loop silent for this long -> corrections zeroed
AXES = ("X", "Y", "Z", "A", "B", "C")
ZERO = {axis: 0.0 for axis in AXES}


def _dist(p, q):
    return math.sqrt(sum((p[k] - q[k]) ** 2 for k in "XYZ"))


class Teleop:
    def __init__(self, api, source):
        self.api = api
        self.source = source
        self.mode = "TELEOP"            # TELEOP, RECORDING, REPLAY, ESTOP
        self.speed_i = getattr(source, "START_SPEED_INDEX", 1)   # the hand tracker starts slower
        self.start_pose = api.motion.get_current_pose()
        self.recording = []             # [(t, pose)] while RECORDING
        self.replay_trace = []          # [(t, pose)] while the executor runs
        self._tracing = False
        self.trail = deque(maxlen=3000)
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.last_tick = time.time()
        self.last_cmd = Command()
        self.fenced = ()
        self.speed_mm_s = 0.0
        self.offset = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.deviation = None           # (mean, max) after the last replay
        self.message = ""
        self._last_written = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._replay_thread = None

    # ------------------------------------------------------------ lifecycle

    def start(self):
        self._thread.start()

    def stop(self):
        self.stop_flag.set()
        self.api.motion.cancel_trajectory()
        self._thread.join(timeout=2.0)
        if self._replay_thread is not None:
            self._replay_thread.join(timeout=5.0)
        self._write(ZERO)
        if hasattr(self.source, "close"):
            self.source.close()          # the hand tracker owns a camera and a window

    # --------------------------------------------------------- control loop

    def _loop(self):
        period = 1.0 / CONTROL_HZ
        prev_pose, prev_t = None, None
        while not self.stop_flag.is_set():
            t = time.time()
            cmd = self.source.poll()
            self._events(cmd)
            pose = self.api.motion.get_current_pose()
            try:
                ipoc = int(self.api.monitoring.get_ipoc())
            except (TypeError, ValueError):
                ipoc = 0
            with self.lock:
                self.offset = {k: pose[k] - self.start_pose[k] for k in "XYZ"}
                self.trail.append((pose["X"], pose["Y"]))
                if self.mode == "RECORDING":
                    self.recording.append((t, ipoc, dict(pose)))
                if self._tracing:
                    self.replay_trace.append((t, ipoc, dict(pose)))
                if prev_pose is not None and t > prev_t:
                    self.speed_mm_s = _dist(pose, prev_pose) / (t - prev_t)
            prev_pose, prev_t = pose, t

            if self.mode in ("TELEOP", "RECORDING"):
                self._drive(cmd)
            self.last_cmd = cmd
            self.last_tick = t
            time.sleep(max(0.0, period - (time.time() - t)))
        self._write(ZERO)

    def _events(self, cmd):
        ev = cmd.events
        if "quit" in ev:
            self.stop_flag.set()
        if "estop" in ev:
            self.api.motion.cancel_trajectory()
            self.api.safety.stop()
            self.mode = "ESTOP"
            self.message = "E-STOP - press reset (Y / R) to continue"
        if "reset" in ev and self.mode == "ESTOP":
            self.api.safety.reset()
            self._last_written = None
            self._write(ZERO)
            self.mode = "TELEOP"
            self.message = "E-stop reset"
        if "faster" in ev:
            self.speed_i = min(self.speed_i + 1, len(SPEED_SCALES) - 1)
        if "slower" in ev:
            self.speed_i = max(self.speed_i - 1, 0)
        if "record" in ev:
            if self.mode == "TELEOP":
                with self.lock:
                    self.recording = []
                self.mode = "RECORDING"
                self.message = "recording"
            elif self.mode == "RECORDING":
                self.mode = "TELEOP"
                self.message = f"recorded {len(self.recording)} samples -> {self._save_recording()}"
        if "replay" in ev and self.mode == "TELEOP":
            if not self.recording:
                self.message = "nothing recorded yet"
            else:
                self._replay_thread = threading.Thread(target=self._replay, daemon=True)
                self._replay_thread.start()

    def _drive(self, cmd):
        if not cmd.connected or not cmd.deadman or self.api.safety.is_stopped():
            self.fenced = ()
            self._write(ZERO)
            return
        scale = SPEED_SCALES[self.speed_i]
        step = {"X": cmd.x, "Y": cmd.y, "Z": cmd.z}
        fenced = []
        for axis in "XYZ":
            v = step[axis] * MAX_STEP_MM * scale
            # Refuse to push further out past the fence; always allow coming back.
            if (self.offset[axis] >= FENCE_MM and v > 0) or (self.offset[axis] <= -FENCE_MM and v < 0):
                v = 0.0
                fenced.append(axis)
            step[axis] = v
        self.fenced = tuple(fenced)
        self._write({"X": step["X"], "Y": step["Y"], "Z": step["Z"],
                     "A": cmd.a * MAX_ROT_DEG * scale, "B": cmd.b * MAX_ROT_DEG * scale, "C": 0.0})

    def _write(self, correction):
        """Hold `correction` in RKorr. Skips the write if nothing changed, so
        a released stick costs nothing per tick; a safety refusal (E-stop
        active) is not an error here - the network process is already
        sending zeros."""
        if correction == self._last_written:
            return
        try:
            self.api.motion.update_cartesian(**correction)
            self._last_written = dict(correction)
        except RSIError:
            self._last_written = None

    # -------------------------------------------------------------- replay

    def _cycle_deltas(self, samples):
        """Per-cycle deltas that retrace `samples` at the pace they were
        recorded. The IPOC difference between two samples is the number of
        robot cycles that passed (IPOC counts milliseconds, one cycle_time
        per cycle), so each sample-to-sample move is spread evenly over that
        many cycles. Falls back to the wall clock if IPOC is unusable."""
        ipoc_per_cycle = self.api.client.cycle_time * 1000.0
        deltas = []
        for (t0, i0, p0), (t1, i1, p1) in zip(samples, samples[1:]):
            if i1 > i0:
                cycles = max(1, int(round((i1 - i0) / ipoc_per_cycle)))
            else:
                cycles = max(1, int(round((t1 - t0) / self.api.client.cycle_time)))
            step = {a: (p1[a] - p0[a]) / cycles for a in AXES}
            # Never ask for more per cycle than teleop itself may: the
            # network process would clamp it silently.
            for a in "XYZ":
                step[a] = max(-MAX_STEP_MM, min(MAX_STEP_MM, step[a]))
            deltas.extend([dict(step)] * cycles)
        return deltas

    @staticmethod
    def _deltas_to(target, here, step_mm):
        """Straight-line per-cycle deltas from `here` to `target`."""
        back = _dist(here, target)
        n = max(1, int(math.ceil(back / step_mm)))
        return [{a: (target[a] - here[a]) / n for a in AXES}] * n

    def _replay(self):
        self.mode = "REPLAY"
        self._write(ZERO)
        with self.lock:
            samples = list(self.recording)
        start = samples[0][2]
        try:
            # 1. Back to where the recording started, at RETURN_STEP_MM per
            #    cycle (75 mm/s), through the same exactly-once path.
            here = self.api.motion.get_current_pose()
            back = _dist(here, start)
            if back > 0.2:
                self.message = f"returning {back:.1f} mm to the start of the recording"
                self.api.motion.execute_trajectory(
                    self._deltas_to(start, here, RETURN_STEP_MM),
                    space="cartesian", cycles_per_step=1, points="delta")
            # 2. The recording itself, cycle for cycle.
            deltas = self._cycle_deltas(samples)
            self.message = f"replaying {len(samples)} samples as {len(deltas)} cycles"
            with self.lock:
                self.replay_trace = []
            self._tracing = True
            self.api.motion.execute_trajectory(
                deltas, space="cartesian", cycles_per_step=1, points="delta")
        except RSIError as e:
            self.message = f"replay stopped: {e}"
        finally:
            self._tracing = False
            self._write(ZERO)
            if self.mode == "REPLAY":
                self.mode = "TELEOP"
            self._report_deviation()
            if hasattr(self.source, "notify"):
                self.source.notify("replay_done")

    def _report_deviation(self):
        """How far each replayed sample strayed from the recorded path -
        distance to the nearest recorded point, so timing drift between the
        two sample clocks does not count as error."""
        with self.lock:
            recorded = [p for *_, p in self.recording]
            trace = [p for *_, p in self.replay_trace]
        if not recorded or not trace:
            self.deviation = None
            return
        errors = [min(_dist(t, r) for r in recorded) for t in trace]
        self.deviation = (sum(errors) / len(errors), max(errors))
        self.message = (f"replay done: {len(trace)} samples, deviation mean "
                        f"{self.deviation[0]:.2f} mm, max {self.deviation[1]:.2f} mm")

    # ----------------------------------------------------------- recordings

    def _save_recording(self):
        os.makedirs("logs", exist_ok=True)
        path = Path("logs") / f"teleop_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with self.lock:
            rows = list(self.recording)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "ipoc", *AXES])
            for t, ipoc, pose in rows:
                w.writerow([f"{t:.4f}", ipoc, *(f"{pose[a]:.3f}" for a in AXES)])
        return str(path)

    def load_recording(self, path):
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        with self.lock:
            self.recording = [(float(r["t"]), int(r.get("ipoc", 0)),
                               {a: float(r[a]) for a in AXES}) for r in rows]
        self.message = f"loaded {len(self.recording)} samples from {path}"

    # -------------------------------------------------------------- status

    def status_text(self):
        cmd = self.last_cmd
        stale = time.time() - self.last_tick > STALE_S
        try:
            stats = self.api.diagnostics.get_stats()
            cycle = f"{stats.get('mean_cycle_time', 0.0) * 1000:.2f} ms  jitter {stats.get('jitter', 0.0) * 1000:.2f} ms"
        except Exception:
            cycle = "n/a"
        scale = SPEED_SCALES[self.speed_i]
        lines = [
            f"mode      {self.mode}{'   (LOOP STALLED)' if stale else ''}",
            f"input     {self.source.NAME}   {'connected' if cmd.connected else 'NOT CONNECTED'}",
            f"deadman   {'HELD' if cmd.deadman else 'released'}",
            f"speed     {int(scale * 100)} %  (full stick {MAX_STEP_MM * scale * 250:.0f} mm/s)",
            f"demand    X {cmd.x:+.2f}  Y {cmd.y:+.2f}  Z {cmd.z:+.2f}",
            f"offset    X {self.offset['X']:+7.1f}  Y {self.offset['Y']:+7.1f}  Z {self.offset['Z']:+7.1f} mm",
            f"moving    {self.speed_mm_s:6.1f} mm/s",
            f"fence     +/-{FENCE_MM:.0f} mm  {('AT LIMIT ' + ' '.join(self.fenced)) if self.fenced else 'clear'}",
            f"recorded  {len(self.recording)} samples",
            f"cycle     {cycle}",
        ]
        if self.deviation:
            lines.append(f"deviation mean {self.deviation[0]:.2f} mm  max {self.deviation[1]:.2f} mm")
        if self.message:
            lines.append("")
            lines.append(self.message)
        return "\n".join(lines)


# ------------------------------------------------------------- dashboard

def run_dashboard(teleop, seconds=None):
    import matplotlib.pyplot as plt
    import numpy as np

    plt.ion()
    camera = hasattr(teleop.source, "frame")      # the hand tracker supplies a live frame
    if camera:
        fig, (ax, cam_ax, panel) = plt.subplots(
            1, 3, figsize=(17, 6), gridspec_kw={"width_ratios": [3, 3, 2]})
        image = cam_ax.imshow(np.zeros((360, 480, 3), dtype=np.uint8))
        cam_ax.set_title("camera: green = driving, red = stopped")
        cam_ax.axis("off")
    else:
        fig, (ax, panel) = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={"width_ratios": [3, 2]})
    fig.canvas.manager.set_window_title("RSIPI teleop")
    end = None if seconds is None else time.time() + seconds
    s = teleop.start_pose
    ax.add_patch(plt.Rectangle((s["X"] - FENCE_MM, s["Y"] - FENCE_MM), 2 * FENCE_MM, 2 * FENCE_MM,
                               fill=False, linestyle="--", color="grey", label="soft fence"))
    trail_line, = ax.plot([], [], color="tab:blue", linewidth=1, label="path")
    rec_line, = ax.plot([], [], color="tab:orange", linewidth=2, label="recording")
    rep_line, = ax.plot([], [], color="tab:green", linewidth=1.5, label="replay")
    here, = ax.plot([s["X"]], [s["Y"]], "o", color="tab:red", markersize=8)
    ax.set_xlim(s["X"] - FENCE_MM - 10, s["X"] + FENCE_MM + 10)
    ax.set_ylim(s["Y"] - FENCE_MM - 10, s["Y"] + FENCE_MM + 10)
    ax.set_aspect("equal")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    panel.axis("off")
    text = panel.text(0.0, 1.0, "", va="top", family="monospace", fontsize=10, transform=panel.transAxes)

    while not teleop.stop_flag.is_set() and plt.fignum_exists(fig.number):
        if end is not None and time.time() > end:
            break
        with teleop.lock:
            trail = list(teleop.trail)
            rec = [p for *_, p in teleop.recording]
            rep = [p for *_, p in teleop.replay_trace]
        if trail:
            trail_line.set_data([p[0] for p in trail], [p[1] for p in trail])
            here.set_data([trail[-1][0]], [trail[-1][1]])
        rec_line.set_data([p["X"] for p in rec], [p["Y"] for p in rec])
        rep_line.set_data([p["X"] for p in rep], [p["Y"] for p in rep])
        if camera:
            frame = teleop.source.frame()
            if frame is not None:
                image.set_data(frame)
        text.set_text(teleop.status_text())
        if time.time() - teleop.last_tick > STALE_S:
            teleop._write(ZERO)          # the control loop has stalled: belt and braces
        fig.canvas.draw_idle()
        plt.pause(0.1)
    teleop.stop_flag.set()
    plt.close(fig)


def run_headless(teleop, seconds):
    end = None if seconds is None else time.time() + seconds
    last = ""
    while not teleop.stop_flag.is_set():
        if end is not None and time.time() > end:
            print("time limit reached")
            break
        line = f"[{teleop.mode:9}] offset X {teleop.offset['X']:+6.1f} Y {teleop.offset['Y']:+6.1f}  {teleop.message}"
        if line != last:
            print(line)
            last = line
        if time.time() - teleop.last_tick > STALE_S:
            teleop._write(ZERO)
        time.sleep(0.2)


# ------------------------------------------------------------------ main

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    parser = argparse.ArgumentParser(description="Xbox-pad teleoperation with record and replay")
    parser.add_argument("config", nargs="?", default=None, help="RSI Ethernet config (default: the joints context)")
    parser.add_argument("--input", choices=("xbox", "keys", "hand", "synth"),
                        default="synth" if assume_yes() else "xbox",
                        help="control source (default: xbox; synth under dry_run; "
                             "hand needs .venv-demo)")
    parser.add_argument("--depth", action="store_true",
                        help="hand input only: drive X from the hand's apparent size (off by default - "
                             "a tilted hand reads as a size change)")
    parser.add_argument("--replay", metavar="CSV", help="load a saved recording instead of teaching one")
    parser.add_argument("--no-dashboard", action="store_true", help="console status instead of the plot window")
    parser.add_argument("--seconds", type=float, default=None, help="stop after this long")
    args = parser.parse_args()

    source = make_input(args.input, depth=args.depth) if args.input == "hand" else make_input(args.input)
    headless = args.no_dashboard or args.input == "synth"

    api = RSIAPI(args.config or context("joints"), rsi_mode="relative", max_cartesian_rate=MAX_STEP_MM)
    api.start()
    if not api.wait_for_connection(10.0):
        print("No packets from the robot in 10 s - is RSI_ON running?")
        api.stop()
        sys.exit(1)

    scale = SPEED_SCALES[getattr(source, "START_SPEED_INDEX", 1)]
    detail = (f"Full stick = {MAX_STEP_MM * scale * 250:.0f} mm/s at the starting speed "
              f"({MAX_STEP_MM * 250:.0f} mm/s at 100 %), soft fence +/-{FENCE_MM:.0f} mm around "
              f"the current pose, per-cycle steps capped at {MAX_STEP_MM} mm in the network "
              f"process. Motion only while the deadman is held.")
    if not confirm(f"Enable {source.NAME} teleoperation of the robot", detail):
        api.stop()
        sys.exit(0)

    teleop = Teleop(api, source)
    if args.replay:
        teleop.load_recording(args.replay)
    teleop.start()
    try:
        if headless:
            run_headless(teleop, args.seconds if args.seconds else (30.0 if args.input == "synth" else None))
        else:
            run_dashboard(teleop, args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()
        api.stop()

    if args.input == "synth":
        n_rec, n_rep = len(teleop.recording), len(teleop.replay_trace)
        # A real square at 50 Hz is ~100+ samples; the replay must have run
        # for most of it; and it must land on the recorded path.
        ok = (n_rec >= 50 and n_rep >= n_rec // 2
              and teleop.deviation is not None and teleop.deviation[1] < 2.0)
        print(f"synthetic square: recorded {n_rec} samples, replayed {n_rep}, "
              f"deviation {teleop.deviation}")
        print("PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
