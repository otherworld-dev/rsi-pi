# Changelog

## Unreleased

### Fixed
- The echo server no longer locks one packet behind after a single late
  reply. It read one datagram per cycle, so a reply that missed its window
  stayed queued. From then on every cycle read the previous cycle's reply,
  rejected it, and exhausted `Timeout=100` in under a second. It now reads
  past replies to earlier packets within the cycle and drops them.
- The echo server counts a cycle left unanswered once, when the client is
  next heard from (`no reply in time for N cycle(s)`). Silence before a
  client attaches is still never counted. Neither is silence longer than
  `Timeout` cycles, which is treated as the client restarting.
- After a stall, the client answered every queued robot packet in turn. The
  first of those replies was certain to be rejected, yet it carried, and
  acked, any correction published during the stall, so that step was lost
  without an error. The network process now answers only the newest queued
  packet. `diagnostics.get_stats()["skipped_packets"]` counts the ones it
  passed over.

### Added
- `RSIPI.fault_injection.suspended(pid)` freezes a process, such as RSIPI's
  network process, to create a stall on demand.
- `examples/stall_probe.py` measures two things on the robot. First, whether
  a PC stall keeps the robot moving: in relative mode, `HOLDON=1` may repeat
  the last step on every missed cycle. Second, whether ETHERNET `Timeout`
  counts consecutive late packets or a running total.

### Tests
- `tests/test_echo_server_resync.py`: queued late replies, the client's
  read-ahead, one scripted late reply over loopback, and RSIPI's own client
  frozen past a reply window, including a correction published during the
  freeze.
  `RSIPI_SOAK_SECONDS=<n> pytest -m soak` runs the same stall repeatedly for
  minutes.

## 0.2.0 — 2026-09-14

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
- `RSI_EthernetConfig_Max.xml` no longer declares the `PosCorrStat` SEND
  element (index 22) that the context stopped wiring.

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
- `examples/teleop/` (Xbox pad or hand-tracking teleoperation) has moved to
  the PhD applications repo, history included; RSIPI ships no teleop code.

### Docs
- `docs/rsi-objects.md` (all 74 RSI objects and how they reach KRL),
  `docs/hardware-findings.md`, `docs/hardware-test-plan.md`,
  `docs/controller-setup.md`.

### Packaging
- Published on PyPI: `pip install RSIPI`. Requires Python 3.10+ (the CLI
  and `viz` use `match`).
- `lxml` and `scipy` dropped from the dependencies — nothing imported them.
- `RSIPI.__version__` now matches the released version (it said 2.0.0);
  `tests/test_version.py` keeps the two in step.
- Releases build and upload from GitHub Actions (`publish.yml`) through
  PyPI trusted publishing — no API token is stored anywhere.

## 0.1.1

Initial public version.
