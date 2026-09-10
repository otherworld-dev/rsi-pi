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

   That table is the **complete** set of KRL variables reachable through an
   object — every `$` variable named anywhere in the object reference — plus
   the `STATUS` object, which exposes four more read-only:

   | Via `STATUS` `Type=` | KRL variable |
   |---|---|
   | `ProState_S` / `ProState_R` | `$PRO_STATE` (submit / robot interpreter) |
   | `Pro_Mode_S` / `Pro_Mode_R` | `$PRO_MODE` |
   | `Mode_Op` | `$MODE_OP` — T1 / T2 / AUT / EXT |
   | `IPO_Mode` / `IPO_Mode_C` | `$IPO_MODE` |

   `IPO_State` and `Sensor` return interpolator and sensor-interface state
   that have no direct KRL variable.

3. **Reset or retune objects at runtime.** Parameters not marked "cannot be
   changed at runtime" — the `Reset` flags on `I`, `PID`, `TIMER`,
   `IIRFILTER`, `GENCTRL` especially — are settable from KRL. See the RSI
   manual for the parameter-access functions; RSIPI does not wrap them.

Objects with no KRL variable at all (`SUM`, `PID`, `LIMIT`, the whole maths
and logic set) exist purely inside the graph. They need no support from
anything — you wire them and they work.

### `$FLAG` — free, already wired, and useful

The `ETHERNET` object's `Flag` parameter needs no channel and no object of its
own. Every shipped RSIPI context sets `Flag=1`, so **`$FLAG[1]` is live on
your controller right now**:

```krl
IF $FLAG[1] THEN
  ; TRUE means the RSI connection is INTERRUPTED - it is a fault flag,
  ; not a health flag. Bring the cell to a safe state here.
ENDIF
```

This is the cleanest way for a KRL program to notice the PC has gone away
without waiting for the `Timeout` break-off.

### The third class: `INTERNAL` declarations

Neither an object nor a channel — just a line in the Ethernet config. RSI
reads the system variable directly and the value costs nothing:

```xml
<ELEMENT TAG="DEF_RIst" TYPE="DOUBLE" INDX="INTERNAL" />
```

| Declared | Carries |
|---|---|
| `DEF_RIst` / `DEF_RSol` | Cartesian actual / setpoint position |
| `DEF_AIPos` / `DEF_ASPos` | Axis actual / setpoint position |
| `DEF_EIPos` / `DEF_ESPos` | External axis actual / setpoint |
| `DEF_MACur` / `DEF_MECur` | Motor currents, main / external axes |
| `DEF_Delay` | Late-packet counter |
| `DEF_EStr` | Message string to the controller |
| `DEF_Tech.C1-C6` / `DEF_Tech.T1-T6` | `$TECHPAR_C` / `$TECHPAR` generators |

RSIPI's parser also understands `ELPos`, `BMode`, `IPOSTAT` and `IPOC`, which
no shipped config declares. `IPOC` is handled specially (the timestamp); the
other three are available if you declare them, though their exact semantics
are in the RSI manual rather than the object reference, so confirm before
relying on them.

⚠️ `DEF_Tech.T1` is the one to avoid: **generator 1 is reserved for RSI's own
corrections** (`RSITECHIDX`), and declaring it makes the ETHERNET object
report `RSIBad`. Use generator 2 for PC→KRL commands.

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

Parameters `LowerLimX/Y/Z`, `UpperLimX/Y/Z` and `MaxRotAngle` cap this
object's **cumulative** correction, not the per-cycle delta. **KUKA's default
is ±5 mm**, which is why a move can stop dead at exactly 5.00 mm with no error
anywhere — the correction has hit the cap and simply stops growing. RSIPI's
shipped contexts raise this to ±50 mm / 45°.

The reference is explicit about the silence: *"if an input exceeds the valid
range, the corresponding maximum value is used."* The controller clamps and
reports nothing. `AXISCORR` behaves identically.

**There is a second, separate limit, and it is not silent.** The RSI manual:
*object-specific corrections are limited by default to ±5 mm / 5° and
clamped; the* **overall** *correction is limited to ±6 mm / 6°, and if that
is exceeded, signal processing is stopped.* The overall limit is set by a
`POSCORRMON` / `AXISCORRMON` object — and **applies at its 6 mm default when
the context has none**. On the KR 16-2 (2026-09-10) a context with ±50 mm
`POSCORR` limits and no monitor stopped every move past ~5 mm with
`KSS29000 POSCORR Permissible overall correction exceeded: RSI is stopped`.
Every shipped context now carries a monitor at 500 mm / 180°.

#### `Stat` says exactly when — and where — it is clamping

`Stat` is not the 0/1/>1 summary it looks like. Above 1 it is **bit-coded**:

| Bit | Meaning |
|---|---|
| B0 | always 1 |
| B1 / B2 / B3 | reached **lower** limit in X / Y / Z |
| B4 / B5 / B6 | reached **upper** limit in X / Y / Z |
| B7 | reached `MaxRotAngle` |

So `Stat = 17` (`0b00010001`) means "correcting, and clamped at the upper X
limit". Wire `Stat` to a spare ETHERNET channel and the silent truncation
becomes visible from Python — arguably the most useful diagnostic in this
whole reference, given how much time that failure mode cost during bring-up.

⚠️ **It cannot be sent back over Ethernet.** `Stat` was wired to a spare
ETHERNET input in `RSIPI_Max` and the controller refused the context with
`RSI_CREATE: Circular linking` (2026-09-10): anything derived from an
ETHERNET *output* (`RKorr` → `POSCORR` → `Stat`) must not feed an ETHERNET
*input*. KUKA's own examples route `Stat` into `GREATER` → `STOP` instead,
which is the pattern to copy. `monitoring.get_correction_limit_status()`
decodes the bits if you do get the value to the PC some other way, and
returns `None` on every shipped context.

`AXISCORR` has the same output if you want the joint equivalent.

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

Despite the name, these are **not passive monitors**. Each *sets* the
ceiling for the **overall** correction — and that ceiling exists whether or
not the object does: **with no monitor in the context the system default of
6 mm / 6° applies**, and exceeding it stops RSI with `KSS29000`. Their outputs
report the correction applied so far — `X`…`C` for `POSCORRMON`, `A1`–`A6`
then `E1`–`E6` for `AXISCORRMON`.

⚠️ Confirmed on the KR 16-2, 2026-09-10: `RSIPI_Basic` (±50 mm `POSCORR`, no
monitor) stopped every move at ~5 mm; adding `POSCORRMON1` at 500 mm fixed
it. **Every shipped context now has one** (Joints and Max also
`AXISCORRMON1`), and `config_builder` warns when a context lacks them. The
objects need no wiring — parameters only.

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
| `POSACT` | 24 | Cartesian position X,Y,Z,A,B,C plus Status/Turn | `Type`: `Measured` (default), `IPO`, `IPO_FLT`, `CF` — see the timing note below |
| `AXISACT` | 32 | Axis angles A1–A6 | Same `Type` choice |
| `AXISACTEXT` | 32, offset 7 | External axis positions E1–E6 | External axes only |
| `MOTORCURRENT` | 47 | Motor current per axis \[A\] | No parameters |
| `MOTORCURRENTEXT` | 47, offset 7 | External axis motor currents | External axes only |
| `GEARTORQUE` | 46 | Gear torque per axis \[Nm\] | `Type`: setpoint / precontrol / motor-side |
| `GEARTORQUEEXT` | 46, offset 7 | External axis gear torques | External axes only |
| `STATUS` | 62 | One controller status value | See below |
| `OV_PRO` | 63 | Program override `$OV_PRO` | No parameters |

⚠️ **The `Type` choice is a time shift, not just a source.** `IPO` is about
**100 ms in the future** relative to the drive interface and refreshed only
every 12 ms, and is valid only for CP motions; `IPO_FLT` is about 40 ms ahead.
`Measured` is the default and the one to use unless you specifically want a
look-ahead.

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

One object gives **one** status value, chosen by its `Type` parameter
(`RSI_StatusType`, default `IPO_State`). Want both the operating mode and the
interface state? That is two `STATUS` objects on two channels.

| `Type` | Returns |
|---|---|
| `IPO_State` | Interpolator state — a **bit field**, see below |
| `ProState_S` / `ProState_R` | `$PRO_STATE` of the submit / robot interpreter |
| `Pro_Mode_S` / `Pro_Mode_R` | `$PRO_MODE` of the submit / robot interpreter |
| `Mode_Op` | `$MODE_OP` — 1 T1, 2 T2, 3 AUT, 4 EXT |
| `IPO_Mode` / `IPO_Mode_C` | `$IPO_MODE` — 1 Base, 2 TCP (advance / main run) |
| `Sensor` | **Sensor-interface state — the controller's own view of RSI** |

**`Sensor` is the one to reach for.** It reports the RSI interface's health
directly, which nothing else exposes:

| | | | | |
|---|---|---|---|---|
| 0 OFF | 1 PRE_INIT | 2 INIT | 3 CYCLE | 4 FREEZE |
| 5 FREEZE_IGNORE | 6 TERMINATE | 7 TERMINATE_IGNORE | 8 CLEAR_OFFSETS | **9 ERROR** |

`CYCLE` is the healthy running state. `ERROR` on the wire would be the
earliest, clearest signal that RSI is unhappy — better than inferring it from
packet timing.

`$PRO_STATE`: 1 FREE, 2 RESET, 3 ACTIVE, 4 STOP, 5 END.
`$PRO_MODE`: 1 ISTEP, 2 MSTEP, 3 PSTEP, 4 CSTEP, 5 BSTEP, 6 GO.

`IPO_State` bit flags: 1 ACTIVE, 2 CONTINUE, 4 STOP, 8 FSTOP, 16 GSTOP,
32 GSTOP_MOV, 64 CP (current move is Cartesian), 128 SMOOTH.

⚠️ **Not shipped in any RSIPI context.** Not because it lacks a default — it
defaults to `IPO_State` — but because the `.rsi.xml` stores `Type` as a
*number*, and the reference only ever prints names. Following the pattern
confirmed twice elsewhere (list order, counting from 0) would make
`Mode_Op` = 5 and `Sensor` = 8, but that is a prediction, not a fact, and a
wrong enum ordinal is silent. **Add the object in RSIVisual and read the
number it writes** — the same one-minute method that settled `Word` = 4.

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

### How to actually use this

**First, the one that catches everybody: `DIGOUT` does not switch an output
on.** It *reads back* the current state of one. Writing is `MAP2DIGOUT`. The
naming rule is consistent across RSI — a bare name **reads** a KRL signal
into the graph, `MAP2<name>` **writes** it back out. (`ANOUT` reads too;
`MAP2ANOUT` writes.)

#### One signal, or several?

That single decision drives everything else.

| You want | `DataSize` | `Index` means | Value on the wire |
|---|---|---|---|
| **One** output/input | `Bit` | the signal number itself | 0 or 1 |
| **8 / 16 / 32** at once | `Byte` / `Word` / `DWord` | a **byte** number | a bitmask integer |

For a single gripper on `$OUT[5]`, use `Bit` with `Index=5`. No arithmetic, no
bitmask, and `io.set_output(5, True)` addresses it by its real number.

#### Byte and word addressing

With anything wider than `Bit`, `Index` counts **bytes**, and byte *b* covers:

    $OUT[b * 8 + 1]  …  through 8, 16 or 32 signals

| `Index` | Covers (`Byte`) |
|---|---|
| 1 | `$OUT[1]`–`$OUT[8]` |
| 2 | `$OUT[9]`–`$OUT[16]` |
| 3 | `$OUT[17]`–`$OUT[24]` (`Word`: –`$OUT[32]`) |
| 20 | `$OUT[153]`–`$OUT[160]` (`Word`: –`$OUT[168]`) |

Measured on the KR 16-2 (2026-09-10): `MAP2DIGOUT` with `Index=2, Word` put
bit 1 on `$OUT[10]`, so the first signal is `(Index − 1) × 8 + 1`. This is
why `Index=3` does **not** touch `$OUT[3]`, and why outputs so often look
"dead". (The August bring-up read `DIGIN1` `Index=1, Byte` as `$IN[9]`–`[16]`,
i.e. `Index × 8 + 1` — the two objects may count differently, or that
reading was wrong; re-measure `DIGIN` before relying on either.)

#### Reading the bitmask — right to left

One integer carries all eight states. **The lowest signal number is the
rightmost (least significant) bit.** With `Index=2`:

| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|---|
| Output | `$OUT[24]` | `[23]` | `[22]` | `[21]` | `[20]` | `[19]` | `[18]` | `$OUT[17]` |

So the **first three on** is `0b00000111` = **7**:

```python
api.io.set_output(1, True)   #   1  0b00000001   $OUT[17]
api.io.set_output(2, True)   #   3  0b00000011   $OUT[17],[18]
api.io.set_output(3, True)   #   7  0b00000111   $OUT[17],[18],[19]
```

`0b11100000` = 224 would be the *top* three — `$OUT[22]`, `[23]`, `[24]`.

Note the channel number in `set_output()` is the **bit position**, not the
`$OUT` number, whenever the object is byte- or word-addressed. Each call is a
read-modify-write of the whole word, so all eight signals travel together in
one channel and land in the same 4 ms cycle. That is the real reason to use
byte/word addressing instead of eight separate `Bit` objects.

#### Signed vs unsigned

`Byte_U` is unsigned (0–255); `Byte` is signed (−128–127). With signed, any
value with the top bit set comes back **negative** — all eight on reads as
**−1**, and `$OUT[24]` alone as **−128**.

So **read with `Byte_U` / `Word_U`**. You cannot do the same when writing:
`MAP2DIGOUT` offers only `Bit`, `Byte`, `Word`. Mixing them is fine — the bits
on the wire are identical, only the interpretation of the returned number
differs — so write `Word` and read back `Word_U`.

#### Analogue is simpler

`ANIN` / `MAP2ANOUT` have no `DataSize`: just `Index`, 1–32, addressing
`$ANIN[n]` / `$ANOUT[n]` directly. One value per object.

```python
api.io.read_analog()        # $ANIN[1]
api.io.set_analog(0.75)     # $ANOUT[1]
```

#### Confirm what you sent actually landed

Pair a writer with a reader on the same range — `MAP2DIGOUT` to drive, a
`DIGOUT` at the same `Index` to read back — and the robot tells you the real
state of its own outputs. That is what `DoutW` does in `RSIPI_Joints`, and it
is the only way to check without walking to the pendant.

#### Latching, and why a gripper may want it

`MAP2DIGOUT` is level-driven: send FALSE and the output drops. `SETDIGOUT`
and `RESETDIGOUT` act only on a rising edge and **latch**, so the output holds
regardless of what happens afterwards. Point both at the same `Index` and you
have a set/reset pair:

```
ETHERNET Out7 ──→ SETDIGOUT   (Index 5)   close
ETHERNET Out9 ──→ RESETDIGOUT (Index 5)   open
```

```python
api.io.pulse(7, duration=0.05)   # close - latches ON
api.io.pulse(9, duration=0.05)   # open  - latches OFF
```

⚠️ **This matters for holding a workpiece.** With `MAP2DIGOUT` and
`HOLDON="0"`, losing the PC resets the output and the gripper opens
mid-cycle. `HOLDON="1"` holds the last value, and a latch holds it no matter
what — the robot keeps hold of the part and stops. Choose deliberately.

⚠️ **Do not let both ends drive the same output.** RSI writes it every 4 ms,
so a KRL assignment to the same `$OUT` appears to do nothing. If the output
is safety-interlocked, leave it to KRL and send intent instead
(`krl.write_sen_pint()`), so the output keeps a single owner.

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

      $OUT[(Index - 1) * 8 + 1]      … through + 8, 16 or 32

  So `Index=3, DataSize=Word` starts at `$OUT[17]` and covers `$OUT[17]`–`[32]`
  (measured: `Index=2` put bit 1 on `$OUT[10]`, 2026-09-10). `Index` counts
  bytes from 1, matching the parameter's minimum of 1.

The same applies to `ANIN`/`ANOUT` (`Index` 1–32, no `DataSize`) and to
`SEN_PREA`/`SEN_PINT` (`Index` 1–20), which are always single values.

### ⚠️ The `Index` / `DataSize` trap

`DataSize` decides how many consecutive signals an object covers, and it
**changes what `Index` means**:

> For `DataSize` **Bit**, `Index` is a bit index (≥1). For anything wider it
> is a **byte index** (≥0).

So `MAP2DIGOUT` with `Index=3, DataSize=Word` does **not** drive `$OUT[3]`.
Byte 3 begins at output 17, so it drives `$OUT[17]`–`$OUT[32]` (`RSIPI_Max`
ships this way; a gripper on `$OUT[18]` is `set_output(2)`). Outputs
appearing "dead" is almost always this.

**And there are two different DataSize enums**, which is a live hazard when
hand-editing a `.rsi.xml`:

| Enum | Used by | Values, in the order the reference lists them |
|---|---|---|
| `RSI_DataSize` | `DIGIN`, `DIGOUT` | Bit, Byte_U, Byte, Word_U, **Word**, DWord |
| `RSI_DataSizeX` | `MAP2DIGOUT` | Bit, Byte, **Word** |

The lists are the ordinals, counting from Bit=0, so `Word` is **4** in
`RSI_DataSize` and **2** in `RSI_DataSizeX`.

**Confirmed against RSIVisual (2026-09-09):** setting a `DIGOUT` to `Word`
and saving makes RSIVisual write `ParamValue="4"`. The reference itself only
ever prints names, so this is worth knowing rather than deducing — and it is
the cheap way to settle any other enum: set it in RSIVisual, save, read the
number out of the `.rsi.xml`.

`DataSize="2"` therefore means **Word** on a `MAP2DIGOUT` but **Byte** on a
`DIGOUT`. Get it wrong and the file still loads, still runs, and quietly moves
half as many bits as you think.

*(Fixed 2026-09-09: `DIGOUT4`, the `DoutW` read-back in `RSIPI_Joints` and
`RSIPI_Max`, said `Word` in its `.rsi` but encoded `2` — a signed Byte — in
its `.rsi.xml`, so it read back only the low 8 bits of the 16-bit word
`MAP2DIGOUT1` writes. The hardware test that passed used values 3, 5 and 0,
all inside a byte, so it could not have caught it. Now `4` in both contexts.*

*This is also the neatest way to find that class of error: **open the context
in RSIVisual and save it.** RSIVisual regenerates the `.rsi.xml` from the
name-based `.rsi`, so every ordinal is rewritten correctly and any hand-edited
mistake simply disappears — it repaired this one on its own.)*

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

A second, optional parameter `Channel` (default 0) tells several STOP objects
apart: *"when a stop object is triggered, its channel value is stored as a
global parameter, which can be used to tell the reason for stopping from
KRL."* So with two STOP objects on different channels, the KRL program can
find out which one fired.

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
SmartHMI RSIMonitor plugin). `Refresh` sends only every nth cycle, and
`Channel` (1–8) groups signals logically or separates synchronously running
containers. Separate from the `ETHERNET` object and not used by RSIPI.

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

### The config file: tags are just variable names

An `ELEMENT` in the Ethernet config is a channel given a name:

```xml
<ELEMENT TAG="DiO" TYPE="LONG" INDX="8" HOLDON="1" />
```

**`TAG` is an arbitrary name you choose.** It becomes the literal XML element
in the telegram — that line produces `<DiO>3</DiO>` on the wire. `DiO` is not
an RSI keyword; it is simply what KUKA called it in their example, and RSIPI
inherited the name. Rename it to `Gripper` and everything still works, because
both ends read the *same config file*.

What gives the channel meaning is the **wiring in the context**: `ETHERNET1`
output 8 → `MAP2DIGOUT1` → `$OUT[17]` (Max). The name carries no meaning to the
controller.

**The exception is `INDX="INTERNAL"`.** Those tags *are* reserved RSI names:

```xml
<ELEMENT TAG="DEF_RIst" TYPE="DOUBLE" INDX="INTERNAL" />
```

`DEF_RIst` means "Cartesian actual position", `DEF_Delay` means "late packet
counter". RSI reads the system variable directly, so they need no object and
no channel — but you cannot rename them.

> **Rule:** numbered `INDX` → the name is yours. `INDX="INTERNAL"` → the name
> is RSI's.

#### Dotted tags group into one element

A dot makes the value an **attribute** of a shared element:

| Config | On the wire |
|---|---|
| `TAG="DiO"` | `<DiO>3</DiO>` |
| `TAG="DiO.A"`, `.B`, `.C` | `<DiO A="1" B="0" C="1" />` |

RSIPI parses the second as one variable with three values:
`{"DiO": {"A": …, "B": …, "C": …}}`. That is the same mechanism behind
`<RKorr X="…" Y="…" Z="…" />`.

Grouping is naming only — **each sub-value is still its own RSI signal and
needs its own `INDX` and its own wired object.** To pack several signals into
*one* channel, use `DataSize` (byte/word), not dotted tags.

Useful convention: name a group's members `o1`, `o2`, `o3` and
`io.set_output(1, True)` finds them automatically, since it looks for
`o<channel>` in any RECEIVE group. Names like `.A`/`.B` work equally well on
the wire but need `io.toggle('DiO', 'A', True)`.

#### `TYPE` decides how the value is written

`TYPE` is carried through to the wire, so a value goes out as what it was
declared to be:

| `TYPE` | Python | On the wire |
|---|---|---|
| `BOOL` | `bool` | `1` / `0` |
| `LONG` | `int` | `7` |
| `DOUBLE` | `float` | `1.500000` (6 dp) |
| `STRING` | `str` | as-is |

This holds for grouped values too. Until 2026-09-09 it did not: every
attribute was forced through `float()`, so a grouped `BOOL` went out as
`A="1.000000"`. None of the shipped configs declare a grouped BOOL or LONG on
the RECEIVE side, so nothing was affected in practice — but a hand-built
config would have been the first to hit it, and a silently mistyped signal is
the hardest kind of fault to find on a robot.

### Startup and failure modes

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
| `SOURCE` | 45 | Signal generator: `Const`, `Sin`, `Cos`, `Square`, `Sawtooth` (default `Const`), with `Offset`, `Amplitude` and `Period`. Drives motion with no sensor attached |
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
| `GREATER` `LESS` | 6, 5 | Compare against a constant or another signal. `Hysteresis` sets the minimum difference before the output flips back, which stops a noisy signal chattering |
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
   write them — opening a context and saving it regenerates every ordinal
   from the names, which is both the fix and the check.
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
