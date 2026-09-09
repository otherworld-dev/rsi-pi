# Hardware test plan — Thursday 2026-09-10

Everything waiting on the robot, in the order to run it. Rig: KUKA KR 16-2,
KRC4, KSS 8.3.

Results go in [hardware-findings.md](hardware-findings.md) afterwards.

## Before you start

- **T1, override ~10%**, hand on the enabling switch and the hardware E-stop.
- **Start the PC side first** in every session except ONLYSEND. The ETHERNET
  object breaks off with `RSIBad` 0.4 s after `RSI_ON` if nothing answers.
- Copy context files as a **set of four**:
  `python -m RSIPI.deploy --context <name> --out C:\deploy`
- Check a context offline before deploying it:
  `python examples/validate_context.py`

---

## Session 0 — Regression check (do this first, ~10 min)

**Why this comes first:** the wire format changed since the last hardware run.
Values are now sent as their declared type rather than all-floats, and scalars
from the robot are parsed to their declared type rather than left as strings.
Both changes touch the 4 ms hot path and are verified only offline. Prove a
known-good context still works before trusting any new result.

**Context:** `RSIPI_Joints` (hardware-verified 2026-08)
**Run:** `python examples/first_contact.py` (use the joints config)

| Check | Pass |
|---|---|
| Connects, IPOC advances ~1000/s | packets flowing, `Delay` 0 |
| Stage 2 move | +5.00 mm, as before |
| Stage 3 E-stop | stops mid-move, no resume on reset |

⚠️ **If this fails, stop.** Something in today's changes broke the wire format,
and nothing below is meaningful until that is understood.

---

## Session 1 — STOP / ExitMoveCorr (card #645)

The blocker: `RSI_MOVECORR()` cannot currently be ended from the PC, so
`motion.exit_movecorr()` does nothing useful.

**Background:** a STOP object once made the controller silently stop applying
*every* correction — RSI ran perfectly, no error, only a pose that never
changed. `Mode=4` is confirmed correct against KUKA's CircleCorr example, and
the `Channel` parameter theory is dead (that parameter is real and optional).
So the cause is still open; the remaining suspect is the input source, since
all three of KUKA's STOP objects are driven by condition objects rather than
directly from an ETHERNET channel.

**Deploy:** `RSIPI_Stop` (4 files) + `controller/Program/RSIPI_Stop.src`
**Run:** `python examples/stop_test.py`

| Step | Pass |
|---|---|
| 1. Corrections still work | **5 mm move measured.** If the robot does not move, STOP has broken corrections again — stop here and report |
| 2. `MoveStop` ends MOVECORR | pendant reaches `MOVECORR ENDED - STOP object works`; PC sees IPOC freeze |

The script tests motion **before** the stop deliberately: last time the
failure was indistinguishable from success at the network level.

**If step 1 fails:** the STOP object is the cause, and the next thing to try is
driving it from a condition object (e.g. a `GREATER` fed from an ETHERNET
channel) rather than the channel directly — matching how KUKA's examples wire
it.

---

## Session 2 — RSIPI_Max (the big one)

Everything `Max` adds over `Joints` is unverified. RSIVisual opened and
re-saved the context without complaint, so the XML is structurally sound —
but only the controller can say whether every object *binds* on this robot.

**Deploy:** `RSIPI_Max` (4 files) + `controller/Program/RSIPI_Max.src`
(nothing to edit on the pendant - it is `RSIPI_Minimal` with the context name
changed).
**Run:** `python examples/max_test.py`

| # | Check | Pass |
|---|---|---|
| 0 | **Context loads** | no `RSIBad`, no `RSI_CREATE` error at `RSI_ON` |
| 1 | **Override** | `$OV_PRO` reads back non-zero, and **the pendant override is NOT 0%** |
| 2 | Applied-correction monitors | `PosCorrMon` matches the measured move within 0.5 mm |
| 3 | Motor currents | `MACur` non-zero (costs no channel — `INTERNAL`) |
| 4 | Analogue I/O | `$ANIN[1]` readable; `$ANOUT[1]` writable (pendant: Display > Analog I/O) |
| 5 | `$SEN_PINT` | write 7 from Python, read 7 back; confirm on pendant (Display > Variable > Single) |
| 6 | **Correction clamping** | drive X past the ±50 mm `POSCORR` limit; `get_correction_limit_status()` should report `at_limit: ['upper X']` instead of the move silently stopping |
| 7 | 16-bit output word | write 300 to `DiO`, `DoutW` reads 300 — needs bit 8, so this proves the `Word` fix |

**Check 1 matters most.** `MAP2OV_PRO` writes `$OV_PRO` every cycle, so
whatever RSIPI holds *is* the robot's speed dial. The default was 0, which
would have commanded 0% override the instant RSI started — found offline, now
defaulted to 100. **Glance at the pendant override immediately after
`RSI_ON`.** It should be exactly what you set it to.

**Open question worth settling here:** at 0% override, do RSI corrections still
apply, or does everything stop? Not documented and not tested. If check 1 ever
misbehaves, that answer explains what you would see.

**If the context fails to load,** note *which object* the pendant names, then
fall back to `context("joints")`. Likely suspects are the objects with no real
export to copy from: `ANIN`, `SEN_PINT`, `OV_PRO`, and the three `MAP2*`.

---

## Session 3 — Opportunistic, if time allows

### Velocity and acceleration (card #635)
Implemented and offline-verified, never checked against a real robot.
Run a slow known move and confirm `monitoring.get_velocity()` matches
distance/time. Currently derived from the IPOC clock, so real jitter is the
thing to look for.

### ETHERNET `Precision` (card #640)
Default is 1 decimal place in *sent* data, which is why an absolute move
landed at 4.90 mm rather than 5.00. Raise `Precision` on the ETHERNET object
(RSIVisual, save, redeploy) and re-measure the same move. Received data is
always parsed at full precision, so this only affects robot → PC.

### `$FLAG[1]` — comms-loss signal (free, never tested)

Every shipped context sets the ETHERNET object's `Flag=1`, so `$FLAG[1]`
should go **TRUE when the connection is interrupted** — a fault flag, not a
health flag. Nothing has ever confirmed it fires.

Cheap test, and useful if it works: with RSI running, watch `$FLAG[1]` on the
pendant (Display > Variable > Single), then stop the Python side. The flag
should go TRUE before the `Timeout` break-off. It gives a KRL program a way
to notice the PC vanished and bring the cell to a safe state itself.

### `Word_U` for the read-back
`DIGOUT4` is signed `Word`, so a `DoutW` above 32767 reads back negative.
Switching it to `Word_U` in RSIVisual costs nothing. Only worth doing if
session 2 check 7 passes.

---

## Not testable on this cell

- **`RSIPI_Full` / external axes** (card #641) — the KR 16-2 has none, so
  `AXISCORREXT`, `AxisCorrMon.E1-E6`, `MECur`, `MOTORCURRENTEXT` and
  `GEARTORQUEEXT` cannot be bound. Needs a different cell.
- **`STATUS`** — deliberately absent from every context. Its `Type` is an enum
  whose ordinal is not documented, and guessing one is what cost a session on
  the STOP object. Add it in **RSIVisual** (which writes the correct value),
  then it can be tested.

## What to record

For each session: pass/fail, the numbers, and any pendant message **verbatim**
— the exact wording is what identifies which object failed. Anything
surprising goes in `hardware-findings.md` with symptom → cause → fix.
