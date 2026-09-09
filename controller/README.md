# Controller deployment files

What to copy onto a KRC4 controller, and where (as the **Expert** user
group). Then see [docs/controller-setup.md](../docs/controller-setup.md) for
network setup and the staged first-run procedure, and
[docs/rsi-objects.md](../docs/rsi-objects.md) for what each RSI object does,
which KRL system variable it touches and whether RSIPI wraps it.

| Repo folder | Copy to (on the controller) |
|---|---|
| [`src/RSIPI/contexts/`](../src/RSIPI/contexts/) | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `Program/` (this folder) | `C:\KRC\ROBOTER\KRC\R1\Program\` (`KRC:\R1\Program` in the Navigator) |

The RSI contexts live **inside the Python package**, not here, so that they
ship with `pip install` — an installed RSIPI would otherwise have no config
to run against. That also means the PC and the controller load the very same
files. Ask Python where they are:

```python
>>> from RSIPI import context, context_files
>>> context("joints")          # path to pass to RSIAPI(...)
>>> context_files("joints")    # the four files to copy to the controller
```

## Contexts (`src/RSIPI/contexts/`)

Six contexts ship. **Never use `RSIPI_Full` unless your cell has
external axes** — it wires `AXISCORREXT` and external-axis (E1-E6) monitor
channels, which a plain 6-axis robot cannot bind, so its ETHERNET object
reports `RSIBad` at `RSI_ON` and the robot stops.

| Context (+ its `RSI_EthernetConfig_*.xml`) | Contents | Use when |
|---|---|---|
| `RSIPI_Basic` | KUKA's `RSI_Ethernet` example, structurally unchanged: RKorr, DiO, `$SEN_PREA`, Tech C1/T2 | Cartesian corrections only. The most conservative choice |
| `RSIPI_Joints` | Basic **+** `AXISCORR` (AKorr.A1-A6), joint feedback (`AIPos`/`ASPos`), and a `DoutW` output read-back | **Default.** Any 6-axis robot wanting joint control |
| `RSIPI_Full` | **+** `AXISCORREXT` (EKorr E1-E6), correction monitors, motor currents | Cells with configured external axes only |
| `RSIPI_OnlySend` | `RSIPI_Basic` with `<ONLYSEND>TRUE</ONLYSEND>` in its config — the only difference | Data logging: the robot streams, the PC never replies |
| `RSIPI_Stop` | `RSIPI_Basic` **+** a `STOP` object (`Mode=ExitMoveCorr`) fed from a `MoveStop` BOOL on RECEIVE 12 | **Under test, not yet verified** — see below. Lets the PC end `RSI_MOVECORR()` |
| `RSIPI_Max` | `RSIPI_Joints` **+** applied-correction monitors, motor currents, analogue I/O, `$SEN_PINT`, program override and `POSCORR` `Stat` | **Not yet verified.** Everything a 6-axis robot can bind |

### `RSIPI_Max` — everything that works without external axes

The point of `Max` is that "maximal" and "portable" are not opposites, as long
as you leave out the external-axis objects. It is `RSIPI_Joints` plus:

| Addition | Object | Channels | API |
|---|---|---|---|
| Applied Cartesian correction | `POSCORRMON1` | SEND 7-12 | `monitoring.get_applied_correction()` |
| Applied joint correction | `AXISCORRMON1` (A1-A6 only) | SEND 13-18 | `monitoring.get_applied_joint_correction()` |
| Motor currents | *none* — `DEF_MACur` is `INTERNAL` | **0** | `monitoring.get_force()` |
| Analogue input `$ANIN[1]` | `ANIN1` | SEND 19 | `io.read_analog()` |
| `$SEN_PINT[1]` | `SEN_PINT1` | SEND 20 | `krl.read_sen_pint()` |
| Program override `$OV_PRO` | `OV_PRO1` | SEND 21 | `monitoring.get_override()` |
| Analogue output `$ANOUT[1]` | `MAP2ANOUT1` | RECEIVE 18 | `io.set_analog()` |
| Write `$SEN_PINT[1]` | `MAP2SEN_PINT1` | RECEIVE 19 | `krl.write_sen_pint()` |
| Write `$OV_PRO` | `MAP2OV_PRO1` | RECEIVE 20 | `monitoring.set_override()` |
| **Correction clamping** | `POSCORR1`'s `Stat` output | SEND 22 | `monitoring.get_correction_limit_status()` |

**The monitors are the interesting ones.** They report what the controller
*actually applied*, which is the question the commanded value cannot answer —
and precisely the blind spot that let a STOP object silently disable every
correction for a whole session while RSI looked perfect.

**Deliberately excluded:** `AXISCORREXT`, `AxisCorrMon.E1-E6`, `MECur`,
`MOTORCURRENTEXT`, `GEARTORQUEEXT` — all external-axis only, and all reasons
`RSIPI_Full` reports `RSIBad` on a 6-axis robot.

**`STATUS` is deliberately absent.** Its only parameter is an enum (`Type`)
whose ordinal is not in `RsiElements.chm`, has no documented default, and
appears in no real export. Guessing an enum ordinal is exactly what broke the
STOP object silently, so if you want robot status, add the object in
RSIVisual — it writes the correct ordinal — wire it to a spare SEND channel
and declare a matching `ELEMENT` in the config.

`RSIPI_Basic` and `RSIPI_Joints` are both verified on hardware (KR 16-2,
KSS 8.3); `RSIPI_Full` is not — see
[docs/hardware-findings.md](../docs/hardware-findings.md).

Each context's **four** files form a unit — the three context files *and*
its config. Always copy them together, and never rename them: the `.rsi`
names its config file, and the KRL programs name the `.rsi` in
`RSI_CREATE`. Copying a mismatched pair produces
`RSI_CREATE: Invalid index - signal output`; check first with:

```
python examples/validate_context.py
```

The PC side must load **the same Ethernet config file** the controller
does; a mismatch means the two ends disagree about the telegram structure:

```
python examples/first_contact.py src/RSIPI/contexts/RSI_EthernetConfig_Basic.xml
```

## Program/

Copy only what you need — each is a standalone KRL program.

| File | Purpose | PC-side partner |
|---|---|---|
| `RSIPI_Minimal.src` | First contact: `RSI_CREATE` → HALT → `RSI_ON` → `RSI_MOVECORR`. No Tech, no I/O. **Start here.** | `examples/first_contact.py` |
| `RSIPI_Test.src` | Full acceptance test: corrections, Tech C/T handshake, `$SEN_PREA`, digital I/O | `examples/rsipi_test.py` |
| `RSIPI_OnlySend.src` | ONLYSEND check: streams for 60 s with **no** `RSI_MOVECORR` and no HALT | `examples/onlysend_monitor.py` |
| `RSIPI_Stop.src` | STOP check: whether `RSI_MOVECORR()` can be ended from the PC | `examples/stop_test.py` |
| `RSIPI_Max.src` | Loads `RSIPI_Max` — monitors, analogue I/O, `$SEN_PINT`, override | `examples/max_test.py` |
| `basic_handshake.src` | Template: wait for a PC signal, signal back | `examples/coordination/01_basic_handshake.py` |
| `parameter_passing.src` | Template: exchange values over Tech C/T | `examples/coordination/02_parameter_passing.py` |
| `state_machine.src` | Template: KRL state machine driven by the PC | `examples/coordination/03_state_machine.py` |

`README.md` in this folder documents the KRL side of those templates.

## Digital I/O: where the DiO word actually lands

`MAP2DIGOUT1` ships with `Index=20`, `DataSize=Word`. The RSI object
reference is explicit: with any `DataSize` other than `Bit`, **`Index` is a
byte index, not an output number**. So the DiO word does *not* drive
`$OUT[20]`:

| DiO bit | Robot output |
|---|---|
| 0 (`api.io.set_output(1, …)`) | `$OUT[161]` |
| 1 (`set_output(2, …)`) | `$OUT[162]` |
| … | … up to `$OUT[176]` (byte 20-21, 16 outputs) |

The same rule applies to `DIGIN1` (`Index=1`, `DataSize=Byte`) on the read
side: byte 1 is `$IN[9]`–`$IN[16]`, so `DiL` bit 0 is `$IN[9]`.

To use different outputs, change `MAP2DIGOUT1`'s `Index` (a byte number) in
both the `.rsi` and `.rsi.xml`, or set `DataSize=Bit` to address a single
output by its actual number.

## Correction limits (why a move can stop short)

`POSCORR1` caps the **total** Cartesian correction, and in relative mode
corrections accumulate — so a move simply stops at the cap, with no error
anywhere. KUKA's example ships ±5 mm, which silently truncates anything
larger; the shipped contexts raise it to a usable working range:

| Object | Limit | Change it in |
|---|---|---|
| `POSCORR1` | ±50 mm (X/Y/Z), 45° `MaxRotAngle` | `LowerLim*`/`UpperLim*` in the `.rsi` **and** `.rsi.xml` |
| `AXISCORR1` (Joints) | ±10° per axis | `LowerLimA*`/`UpperLimA*` |

These are the robot's own guard rails. Keep the PC-side guards on too —
`RSIAPI(max_cartesian_rate=...)` bounds per-cycle motion, and
`api.safety.set_limit()` bounds each correction before it is sent.

## The STOP object (ending `RSI_MOVECORR` from the PC)

`RSI_MOVECORR()` blocks the KRL program forever: the robot is driven purely
by corrections and never reaches an end point, so normally only an operator
cancelling the program ends it. A `STOP` object with `Mode=ExitMoveCorr`
should let the PC end it instead — `api.motion.exit_movecorr()`.

An earlier attempt at this **silently disabled every correction**. RSI kept
running perfectly — 263 packets/s, `Delay` 0, no error, no log entry — but
the reported pose never changed. It was caught only by noticing the position
was byte-identical across two runs.

The cause has since been found by diffing against KUKA's own working STOP
objects in the `CircleCorr` and `DistanceCtrl` examples. Those ship with the
RSI option package under `DOC\Examples\` (also on the controller at
`D:\KUKA_OPT\RSI\DOC\Examples`) — they are the authoritative reference for
any RSI object question, and no RSIVisual licence is needed to read them:

| | KUKA's STOP | Ours (broken) |
|---|---|---|
| `ObjTypeID` | 18 | 18 ✓ |
| `Mode` | `4` / `ExitMoveCorr` | `4` ✓ — **the value was right all along** |
| Second parameter | *none* | `Channel` (`ParamID=2`) — **invented; no such parameter exists** |
| Input source | a condition object (`TIMER1`, `NOT1`, `GREATER1`) | an `ETHERNET1` output channel |

`RSIPI_Stop` reproduces KUKA's element exactly — one parameter, `Mode` —
and changes nothing else. It is **not yet hardware-verified**: run
`examples/stop_test.py` with `RSIPI_Stop.src`, which measures a 5 mm move
*before* testing the stop, so a recurrence of the silent-disabling failure
is caught immediately rather than mistaken for success.

Until that passes, no other shipped context includes a STOP object.

## Order of operations (this bites everyone once)

The ETHERNET object starts its 4 ms exchange the instant `RSI_ON` runs and
reports `RSIBad` after `Timeout` unanswered cycles — 0.4 s with the shipped
contexts. **Start the Python side first**, then let the KRL program past
its HALT. `RSIPI_Minimal.src` pauses for exactly this reason.

`RSIPI_OnlySend` is the one exception: no reply is expected, so there are no
unanswered cycles to count and start order does not matter. That contrast is
what makes it testable — surviving a full 60 s of PC silence is only
possible if `ONLYSEND=TRUE` really reached the controller.

## After changing files

Re-copy them to the controller — it reads what is on its own disk, not this
repo. Context changes need no reboot; reselecting the KRL program reloads
them. If you have RSIVisual, opening the `.rsi` validates the object wiring
before the robot ever loads it.
