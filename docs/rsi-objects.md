# RSI Objects: what they are and how they reach KRL

A reference for the objects you wire together in RSIVisual, written from the
RSIPI side: what each one does, which KRL system variable it touches, and
whether RSIPI has a method for it.

*Source: the official `RsiElements.chm` object reference shipped with the RSI
option package, cross-checked against KUKA's own example contexts and against
what this project has run on a real KR 16-2. Descriptions here are written
fresh rather than copied — consult the CHM itself for the authoritative
wording, full parameter ranges and defaults.*

---

## 1. How the pieces fit together

An **RSI context** is a dataflow graph. Objects are nodes with typed input
and output ports; you connect them in RSIVisual and the controller evaluates
the whole graph once per interpolation cycle — every 4 ms in `#IPO_FAST`,
12 ms in `#IPO`.

Three file types, and it matters which is which:

| File | Who reads it |
|---|---|
| `.rsi` | RSIVisual. Stores parameters **by name** (`value="ExitMoveCorr"`) |
| `.rsi.xml` | **The controller.** Stores the same parameters **by ordinal** (`ParamValue="4"`) |
| `.rsi.diagram` | RSIVisual canvas layout. Nothing else ever reads it |

That name/ordinal split is the single most dangerous thing in this format.
A wrong ordinal is still valid XML, still loads, and produces no error — it
just silently means a different thing. Two real cases from this project are
called out below.

Each object type has an **ObjTypeID** — `ETHERNET` is 64, `POSCORR` is 27 —
and the `.rsi.xml` identifies objects by that number. Note that some IDs are
shared by a main-axis and external-axis pair (`AXISCORR`/`AXISCORREXT` are
both 33), distinguished by an input or output *offset* instead.

### Where KRL comes in

**Most objects never appear in KRL at all.** This surprises people. The graph
runs on the controller and is wired in RSIVisual; a KRL program does not
reference `POSCORR1` or `SUM3` by name. What KRL does is:

1. **Load, start and stop the context:**

   ```krl
   ret = RSI_CREATE("RSIPI_Joints.rsi", CONTID, TRUE)
   ret = RSI_ON(#RELATIVE)        ; or #ABSOLUTE
   RSI_MOVECORR()                 ; sensor-guided motion
   ret = RSI_OFF()
   ret = RSI_DELETE(CONTID)
   ```

2. **Exchange data through KRL system variables.** This is the real answer to
   "how does it appear in KRL". An object like `SEN_PREA` doesn't appear as an
   object — it appears as `$SEN_PREA[1]`, an ordinary KRL variable your
   program reads or writes. The objects are the bridge between those
   variables and the signal graph:

   | KRL variable | Read into the graph by | Written from the graph by |
   |---|---|---|
   | `$IN[n]` | `DIGIN` | — |
   | `$OUT[n]` | `DIGOUT` | `MAP2DIGOUT`, `SETDIGOUT`, `RESETDIGOUT` |
   | `$ANIN[n]` | `ANIN` | — |
   | `$ANOUT[n]` | `ANOUT` | `MAP2ANOUT` |
   | `$SEN_PREA[n]` | `SEN_PREA` | `MAP2SEN_PREA` |
   | `$SEN_PINT[n]` | `SEN_PINT` | `MAP2SEN_PINT` |
   | `$OV_PRO` | `OV_PRO` | `MAP2OV_PRO` |
   | `$TECHPAR_C[fg,i]` | *(config `DEF_Tech.C`)* | — |
   | `$TECHPAR[fg,i]` | — | *(config `DEF_Tech.T`)* |

   The pattern is consistent: a bare name **reads** a KRL variable into the
   graph, and `MAP2<name>` **writes** the graph's value back out to it.

3. **Reset or retune objects at runtime.** Parameters not marked "cannot be
   changed at runtime" — the `Reset` flags on `I`, `PID`, `TIMER`,
   `IIRFILTER`, `GENCTRL` especially — are settable from KRL. See the RSI
   manual for the parameter-access functions; RSIPI does not wrap them.

Objects with no KRL variable at all (`SUM`, `PID`, `LIMIT`, the whole maths
and logic set) exist purely inside the graph. They need no support from
anything — you wire them and they work.

### Where RSIPI comes in

`ETHERNET` is the object that makes RSIPI possible. It carries up to 64
inputs and 64 outputs over UDP, named by the `RSI_EthernetConfig` file. So:

- Anything wired **into** `ETHERNET` arrives in `api.client.send_variables`.
- Anything wired **out of** `ETHERNET` is written from
  `api.client.receive_variables`.

**RSIPI is object-agnostic.** Wire any object's output to an ETHERNET channel
and it appears as a tag automatically — no library change needed. Named
methods (`motion.update_cartesian()`, `io.set_output()`) exist only for the
common cases, and everything else is reachable via
`api.tools.update_variable()` and the raw dictionaries.

---

## 2. Motion correction

The objects that actually move the robot. All of them apply a correction
*on top of* the programmed path; none of them command an absolute position.

### POSCORR — ID 27

Cartesian correction of the TCP with limiting. Inputs `CorrX`…`CorrC`
(mm and degrees); output `Stat` reports 0 = no motion active, 1 = correcting,
>1 = **being limited**.

Parameters `LowerLimX/Y/Z`, `UpperLimX/Y/Z` and `MaxRotAngle` cap the
**cumulative** correction, not the per-cycle delta. **KUKA's default is ±5 mm**,
which is why a move can stop dead at exactly 5.00 mm with no error anywhere —
the correction has hit the cap and simply stops growing. RSIPI's shipped
contexts raise this to ±50 mm / 45°.

**RSIPI:** `motion.update_cartesian()`, `move_cartesian_trajectory()`, and the
trajectory engine. Wired in every shipped context as `RKorr.X`…`RKorr.C`.

### AXISCORR — ID 33

The same idea per axis: `CorrA1`…`CorrA6` in degrees, with `LowerLimA*` /
`UpperLimA*` (default ±5°) and a `Stat` output.

**RSIPI:** `motion.update_joints()`. In `RSIPI_Joints` and above as
`AKorr.A1`…`A6`.

### AXISCORREXT — ID 33, input offset 7

External axes `E1`–`E6`. Shares its ObjTypeID with `AXISCORR`; what
distinguishes it is that its ports and parameters start at index 7
(`LowerLimE1` is index 7, not 1).

⚠️ **This object cannot bind on a robot with no external axes.** The ETHERNET
object then reports `RSIBad` at `RSI_ON` and the robot stops. This is the
single reason `RSIPI_Full` does not run on a standard 6-axis cell.

**RSIPI:** `motion.move_external_axis()`. `RSIPI_Full` only.

### POSCORRMON — ID 81 · AXISCORRMON — ID 82

Despite the name, these are **not passive monitors**. Each defines a ceiling
for the *overall* correction, and **if it is exceeded the robot program must
be reset**. Their outputs report the correction applied so far — `X`…`C` for
`POSCORRMON`, `A1`–`A6` then `E1`–`E6` for `AXISCORRMON`.

⚠️ Both default to **6.0**, which is small. Add one with defaults to a context
that corrects by more than 6 mm and you will trip it. RSIPI's `Max` context
sets 500 mm / 180°.

⚠️ `AXISCORRMON` outputs 7–12 are the external axes, so wire only outputs 1–6
on a 6-axis robot.

**RSIPI:** `monitoring.get_applied_correction()` and
`get_applied_joint_correction()`, in `RSIPI_Max`. These answer *"did the robot
actually do what I asked?"*, which a commanded value cannot — see the STOP
entry below for why that matters.

---

## 3. Reading the robot

All of these are sources: no inputs, just outputs.

| Object | ID | Returns | Notes |
|---|---|---|---|
| `POSACT` | 24 | Cartesian position X,Y,Z,A,B,C plus Status/Turn | `Type` selects measured / interpolated / filtered / drive setpoint |
| `AXISACT` | 32 | Axis angles A1–A6 | Same `Type` choice |
| `AXISACTEXT` | 32, offset 7 | External axis positions E1–E6 | External axes only |
| `MOTORCURRENT` | 47 | Motor current per axis \[A\] | No parameters |
| `MOTORCURRENTEXT` | 47, offset 7 | External axis motor currents | External axes only |
| `GEARTORQUE` | 46 | Gear torque per axis \[Nm\] | `Type`: setpoint / precontrol / motor-side |
| `GEARTORQUEEXT` | 46, offset 7 | External axis gear torques | External axes only |
| `STATUS` | 62 | One controller status value | See below |
| `OV_PRO` | 63 | Program override `$OV_PRO` | No parameters |

**You usually don't need `POSACT` or `AXISACT`.** The Ethernet config can
declare `DEF_RIst`, `DEF_RSol`, `DEF_AIPos`, `DEF_ASPos`, `DEF_MACur` with
`INDX="INTERNAL"`, which RSI reads straight from the system variables at **no
channel cost and with no object**. Every shipped RSIPI context does this, as
do KUKA's own examples.

**A trap worth knowing:** under RSI correction the *setpoint* never changes —
only the actual position moves. Read `RIst` and `AIPos`, not `RSol`/`ASPos`,
or corrections look like they aren't working. RSIPI's
`motion.get_current_joints()` prefers `AIPos` for exactly this reason.

### STATUS in detail

`Type` selects what the single `Stat` output means: interpolator state,
submit/robot interpreter state, program mode, operating mode (T1/T2/AUT/EXT),
interpolator mode, or sensor-interface state. `IPO_State` is a **bit field**
(1 ACTIVE, 2 CONTINUE, 4 STOP, 8 FSTOP, 16 GSTOP, 32 GSTOP_MOV, 64 CP,
128 SMOOTH).

⚠️ **Not shipped in any RSIPI context, deliberately.** `Type` is an enum, the
CHM documents no default, and no real export we have uses it — so the ordinal
that goes in the `.rsi.xml` cannot be established without guessing. Add it in
RSIVisual, which writes the correct value.

---

## 4. Digital and analogue I/O

| Object | ID | Direction | KRL variable |
|---|---|---|---|
| `DIGIN` | 29 | read | `$IN[n]` |
| `DIGOUT` | 43 | read | `$OUT[n]` — reads back what is set |
| `MAP2DIGOUT` | 14 | write | `$OUT[n]` |
| `SETDIGOUT` | 12 | write | `$OUT[n]`, sets on a rising edge and **latches** |
| `RESETDIGOUT` | 13 | write | `$OUT[n]`, clears on a rising edge and latches |
| `ANIN` | 30 | read | `$ANIN[n]` (index 1–32) |
| `ANOUT` | 44 | read | `$ANOUT[n]` — reads back what is set |
| `MAP2ANOUT` | 15 | write | `$ANOUT[n]` (index 1–32) |

**RSIPI:** `io.set_output()`, `io.get_input()`, `io.pulse()` for digital;
`io.read_analog()` / `io.set_analog()` for analogue (`RSIPI_Max`).

### Types: three different ones, easily conflated

Take `DIGIN` reading `$IN[n]`. Three separate types are involved:

| What | Type | Range |
|---|---|---|
| The `Index` **parameter** (the `n`) | `Int` | 1 – 4096, default 1 |
| The object's `Out1` **value** | `Int` | Width set by `DataSize` |
| `$IN[n]` **in KRL** | `BOOL`, indexed by an `INT` | as configured on the controller |

Two things people trip over:

- **The output is an `Int` even for `DataSize=Bit`.** A single input arrives
  as integer 0 or 1, not a Bool. That is why RSIPI's configs declare `DiL` as
  `TYPE="LONG"` rather than `BOOL`. (`$IN[n]` in KRL *is* a `BOOL` — the
  object converts.)
- **`Index` is `Int` in both cases, but counts different things.** With
  `DataSize=Bit` it is the `$IN`/`$OUT` number directly. With any wider size
  it is a **byte** number, and the first signal covered is:

      $IN[Index * 8 + 1]      … through Index * 8 + 8, 16 or 32

  So `Index=20, DataSize=Word` starts at `$OUT[20 * 8 + 1]` = `$OUT[161]`.

  (KUKA's reference states the byte index may be ≥ 0 while also giving the
  parameter a minimum of 1 — the two disagree, so treat 0 as untested.)

The same applies to `ANIN`/`ANOUT` (`Index` 1–32, no `DataSize`) and to
`SEN_PREA`/`SEN_PINT` (`Index` 1–20), which are always single values.

### ⚠️ The `Index` / `DataSize` trap

`DataSize` decides how many consecutive signals an object covers, and it
**changes what `Index` means**:

> For `DataSize` **Bit**, `Index` is a bit index (≥1). For anything wider it
> is a **byte index** (≥0).

So `MAP2DIGOUT` with `Index=20, DataSize=Word` does **not** drive `$OUT[20]`.
Byte 20 begins at output 161, so it drives `$OUT[161]`–`$OUT[176]`. Outputs
appearing "dead" is almost always this.

**And there are two different DataSize enums**, which is a live hazard when
hand-editing a `.rsi.xml`:

| Enum | Used by | Ordinals |
|---|---|---|
| `RSI_DataSize` | `DIGIN`, `DIGOUT` | Bit=0, Byte_U=1, Byte=2, Word_U=3, Word=4, DWord=5 |
| `RSI_DataSizeX` | `MAP2DIGOUT` | Bit=0, Byte=1, **Word=2** |

`DataSize="2"` therefore means **Word** on a `MAP2DIGOUT` but **Byte** on a
`DIGOUT`. Get it wrong and the file still loads, still runs, and quietly moves
half as many bits as you think.

*(Known issue in this repo: `DIGOUT4`, the `DoutW` read-back in
`RSIPI_Joints`/`Max`, says `Word` in its `.rsi` but encodes `2` — signed Byte —
in its `.rsi.xml`. The paired `MAP2DIGOUT1` writes a full 16-bit word, so the
read-back currently sees only the low 8 bits, signed.)*

---

## 5. Talking to the KRL program

| Object | ID | Direction | KRL variable |
|---|---|---|---|
| `SEN_PREA` | 58 | read | `$SEN_PREA[1..20]` — real values |
| `MAP2SEN_PREA` | 17 | write | `$SEN_PREA[1..20]` |
| `SEN_PINT` | 57 | read | `$SEN_PINT[1..20]` — integers |
| `MAP2SEN_PINT` | 16 | write | `$SEN_PINT[1..20]` |
| `MAP2OV_PRO` | 25 | write | `$OV_PRO` — program override |

These are the general-purpose channels between a KRL program and the sensor
system. A KRL program writes `$SEN_PREA[1] = 42.0`; a `SEN_PREA` object with
`Index=1` picks it up and can pass it to `ETHERNET`, and Python sees it.

The technology parameters `$TECHPAR_C[fg,idx]` (main run, config tag
`Tech.Cnm`) and `$TECHPAR[fg,idx]` (advance run, `Tech.Tnm`) work differently:
they need **no object at all**, just an `INDX="INTERNAL"` declaration in the
Ethernet config.

⚠️ Function generator **1 is reserved for RSI's own use** (`RSITECHIDX`).
Declaring `DEF_Tech.T1` makes the ETHERNET object report `RSIBad`. Use
generator 2 for PC→KRL commands, as KUKA's own example does.

**RSIPI:** `krl.read_param()` / `write_param()` for Tech; `krl.read_sen_pint()`
/ `write_sen_pint()` for `$SEN_PINT`; `monitoring.get_override()` /
`set_override()` for override. `$SEN_PREA` is wired as `SenP1`–`SenP3` in
every shipped context.

⚠️ Give `MAP2SEN_PREA` its **own** ETHERNET channels. KUKA's example shares
channels 1–3 with `POSCORR`, which means the values get rate-limited along
with the corrections — sending 42.0 and reading back 0.5.

---

## 6. Control flow and safety

### STOP — ID 18

Stops a motion on a **rising edge** of its Bool input. `Mode` chooses the
reaction:

| Mode | Effect |
|---|---|
| `InfoMessage` | message only |
| `PathNormal` | path-maintaining stop |
| `Velocity` | maximum braking |
| `PathFast` | fast path-maintaining stop |
| `ExitMoveCorr` | **ends an `RSI_MOVECORR()` motion** — the default |

A second, optional parameter `Channel` exists to tell several STOP objects
apart.

This is the only way to end a purely sensor-guided `RSI_MOVECORR()` from
outside; without it the KRL program blocks until an operator cancels it.

⚠️ **Unresolved in this project.** A STOP object added to a working context
made the controller silently stop applying *every* correction — RSI kept
running perfectly at full packet rate with zero late packets and no error,
and the only symptom was a pose that never changed. Removing it restored
motion exactly. `Mode=4` is confirmed correct against KUKA's CircleCorr
example, so the cause is still open. `RSIPI_Stop` exists to test it, and
`examples/stop_test.py` measures a real move *before* testing the stop,
because that failure was indistinguishable from success at the network level.

### MONITOR — ID 55

Streams up to 24 connected signals over Ethernet for visualisation (the
SmartHMI RSIMonitor plugin). `Refresh` sends only every nth cycle. Separate
from the `ETHERNET` object and not used by RSIPI.

### TIMER — ID 40 · DELAY — ID 50 · SIGNALSWITCH — ID 59 · LIMIT — ID 39 · MINMAX — ID 49

Graph-internal control: a timer that raises a Bool when its time expires
(KUKA's CircleCorr drives a STOP from one), a fixed signal delay, a
two-path switch controlled by a Bool, a clamp between two limits, and
running min/max across up to 10 inputs.

---

## 7. The Ethernet interface

### ETHERNET — ID 64

Up to 64 inputs and 64 outputs exchanged as XML over UDP, once per cycle.

| Parameter | Meaning |
|---|---|
| `ConfigFile` | The `RSI_EthernetConfig` file, which **must** live in `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` |
| `Timeout` | Cycles tolerated without an answer. **KUKA's default is 10**; RSIPI ships 100 (0.4 s at 4 ms) |
| `Flag` | Index of a `$FLAG` reporting the link state. **`$FLAG[Index] == TRUE` means "connection interrupted"** — it is a fault flag, not a health flag. Default −1 (disabled); RSIPI's contexts set 1 |
| `Precision` | Decimal places in **sent** data (default 1, max 32). Received data is always parsed at full precision, so the PC may reply with as many decimals as it likes |

⚠️ The exchange starts the instant `RSI_ON` runs, so **the PC must already be
listening**. Otherwise the object breaks off with `RSIBad` after `Timeout`
cycles and the robot stops. RSIPI's `RSIPI_Minimal.src` HALTs before `RSI_ON`
for this reason.

⚠️ A `.rsi` and a config from different builds gives
`RSI_CREATE: Invalid index - signal output` — the graph references a channel
the config never declares. Copy all four files together, and check first with
`examples/validate_context.py`.

RSIPI can also **generate** the config from a context, which removes the
commonest source of that mismatch:

```
python -m RSIPI.config_builder MyContext.rsi.xml --ip 10.10.10.10 --port 64000
```

---

## 8. Signal processing

None of these touch KRL or RSIPI. They are wired between objects inside the
graph, and if you route one's output to an `ETHERNET` channel, RSIPI reads it
as an ordinary number.

**Sources and maths**

| Object | ID | Does |
|---|---|---|
| `SOURCE` | 45 | Signal generator (sine, etc.) — useful for testing motion with no sensor |
| `SUM` | 31 | Adds up to 5 signals plus a constant |
| `MULTI` | 42 | Multiplies two signals |
| `ABS` `POW` `EXP` `LOG` `NORM` | 48, 68, 76, 77, 83 | Absolute value, power, e^x, ln, vector norm |
| `ROUND` `CEIL` `FLOOR` | 80, 78, 79 | Rounding to n places, up, down |
| `SIN` `COS` `TAN` `ASIN` `ACOS` `ATAN` `ATAN2` | 69–75 | Trigonometry, in degrees |

**Controllers and filters** — the classical control blocks, all discretised
against the sensor cycle:

| Object | ID | Does |
|---|---|---|
| `P` | 28 | Gain |
| `I` | 34 | Integrator, with anti-windup limits and a KRL-settable `Reset` |
| `D` | 67 | Differentiator |
| `PD` | 35 | Proportional-differential |
| `PID` | 36 | Full PID |
| `PT1` `PT2` | 37, 38 | First and second order lag |
| `IIRFILTER` | 60 | Bessel / Butterworth / Chebyshev, high or low pass |
| `GENCTRL` | 56 | Generic filter to 8th order, coefficients A1–A8/B0–B8 |

**Logic, comparison and bitwise** — up to 10 inputs each where it makes sense:

| Object | ID | Does |
|---|---|---|
| `AND` `OR` `XOR` `NOT` | 8, 9, 11, 10 | Boolean logic |
| `GREATER` `LESS` | 6, 5 | Compare against a constant or another signal, with hysteresis |
| `EQUAL` | 7 | Compare with a tolerance |
| `BAND` `BOR` `BCOMPL` | 52, 53, 54 | Bitwise and / or / complement on integers |

**Transformations**

| Object | ID | Does |
|---|---|---|
| `TRAFO_USERFRAME` | 65 | Transforms a 3-vector into a frame with a given offset and rotation |
| `TRAFO_ROBFRAME` | 66 | Transforms between robot reference frames (Base, TCP, …) |

---

## 9. Rules learned the hard way

1. **A `.rsi` and its config are one unit.** Always copy `.rsi`, `.rsi.xml`,
   `.rsi.diagram` and the config together.
2. **Never trust an enum ordinal you inferred.** The `.rsi` stores names, the
   `.rsi.xml` stores numbers, and a wrong number is silent. Let RSIVisual
   write them.
3. **Verify a context change by measuring real motion.** An object can load
   perfectly and disable exactly the thing you care about, with the network
   still looking flawless.
4. **Add one object at a time.** Every addition beyond KUKA's proven example
   broke something during this project's bring-up.
5. **External-axis objects need external axes.** They are the most common
   cause of `RSIBad` on a standard 6-axis robot.

## See also

- [controller-setup.md](controller-setup.md) — installing and configuring
- [hardware-findings.md](hardware-findings.md) — what has actually been run,
  with a symptom → cause → fix table
- [../controller/README.md](../controller/README.md) — the shipped contexts
