# Changelog

## 0.2.0 — 2026-09-10

Hardware-verified on a KUKA KR 16-2 (KRC4, KSS 8.3, RSI 3.3) on 2026-08 and
2026-09-10; see `docs/hardware-findings.md`.

### Contexts ship with the package
- `RSIPI.context("joints")` (default), `basic`, `max`, `full`, `onlysend`,
  `stop` — resolved by name, installed as package data. `config_file` is now
  a **required** argument to `RSIAPI`.
- `python -m RSIPI.deploy` copies a context's four files out for the
  controller; `python -m RSIPI.config_builder` generates the Ethernet config
  from a `.rsi.xml` (reproduces all six shipped configs channel for channel).
- **All contexts now carry `POSCORRMON`/`AXISCORRMON`** (500 mm / 180°).
  Without them the controller applies its 6 mm / 6° overall-correction
  default and stops RSI with `KSS29000` — found on the robot.
- `RSIPI_Max`: applied-correction monitors, motor currents, analogue I/O,
  `$SEN_PINT`, program override — verified 11/11 on hardware. Its output word
  is `Index=3` = `$OUT[17..32]`.
- `RSIPI_Stop`: STOP object (Mode = ExitMoveCorr) — verified; the earlier
  build's invented `Channel` parameter was what disabled corrections.

### API
- `monitoring`: `get_motor_currents()` (raises when `MACur` is not wired;
  `MACur` equals `$CURR_ACT`), `get_applied_correction()`,
  `get_applied_joint_correction()`, `get_override()` / `set_override()`
  (1–100, never clamps), `get_correction_limit_status()`,
  `get_robot_status()`.
- `io`: `read_analog()` / `set_analog()`. `krl`: `read_sen_pint()` /
  `write_sen_pint()`.
- Trajectory queue takes `cycles_per_step`; `rate=` is a deprecated alias.
- `cycles_per_step > 1` now spreads each waypoint's delta over its cycles
  instead of sending it in one cycle and holding — the old behaviour
  hammered the robot audibly and was silently shortened by the rate limiter.

### Wire format and safety
- Values are sent as their declared type (BOOL as `1`/`0`, LONG as an
  integer, DOUBLE to 6 dp), and inbound scalars/attributes are coerced to
  their declared type by value.
- `OvProW` defaults to 100 so a context with `MAP2OV_PRO` never commands 0 %
  override at start.
- `EKorr` is covered by E-stop and `zero_corrections()`.
- `RSIAPI.start()` re-raises when the client thread dies instead of
  reporting success.

### Examples
- 18 numbered examples, all gated by a `confirm()` prompt before motion and
  all runnable against the emulator via `examples/dry_run.py`.
- `examples/teleop/`: Xbox pad or hand-tracking teleoperation with record
  and replay, a soft fence, deadman and E-stop, gripper by gesture.

### Docs
- `docs/rsi-objects.md` (all 74 RSI objects and how they reach KRL),
  `docs/hardware-findings.md`, `docs/hardware-test-plan.md`,
  `docs/controller-setup.md`.

## 0.1.1

Initial public version.
