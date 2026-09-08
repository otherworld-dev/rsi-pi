# Controller deployment files

This folder mirrors the KRC4 controller's directory layout — copy each
subfolder's contents to the matching destination (as the **Expert** user
group), then see [docs/controller-setup.md](../docs/controller-setup.md)
for network setup and the staged first-run procedure.

| Repo folder | Copy to (on the controller) |
|---|---|
| `SensorInterface/` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `Program/` | `C:\KRC\ROBOTER\KRC\R1\Program\` (`KRC:\R1\Program` in the Navigator) |

## SensorInterface/

Four contexts ship here. **Never use `RSIPI_Full` unless your cell has
external axes** — it wires `AXISCORREXT` and external-axis (E1-E6) monitor
channels, which a plain 6-axis robot cannot bind, so its ETHERNET object
reports `RSIBad` at `RSI_ON` and the robot stops.

| Context (+ its `RSI_EthernetConfig_*.xml`) | Contents | Use when |
|---|---|---|
| `RSIPI_Basic` | KUKA's `RSI_Ethernet` example, structurally unchanged: RKorr, DiO, `$SEN_PREA`, Tech C1/T2 | Cartesian corrections only. The most conservative choice |
| `RSIPI_Joints` | Basic **+** `AXISCORR` (AKorr.A1-A6), joint feedback (`AIPos`/`ASPos`), and a `DoutW` output read-back | **Default.** Any 6-axis robot wanting joint control |
| `RSIPI_Full` | **+** `AXISCORREXT` (EKorr E1-E6), correction monitors, motor currents | Cells with configured external axes only |
| `RSIPI_OnlySend` | `RSIPI_Basic` with `<ONLYSEND>TRUE</ONLYSEND>` in its config — the only difference | Data logging: the robot streams, the PC never replies |

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
python examples/first_contact.py controller/SensorInterface/RSI_EthernetConfig_Basic.xml
```

## Program/

Copy only what you need — each is a standalone KRL program.

| File | Purpose | PC-side partner |
|---|---|---|
| `RSIPI_Minimal.src` | First contact: `RSI_CREATE` → HALT → `RSI_ON` → `RSI_MOVECORR`. No Tech, no I/O. **Start here.** | `examples/first_contact.py` |
| `RSIPI_Test.src` | Full acceptance test: corrections, Tech C/T handshake, `$SEN_PREA`, digital I/O | `examples/rsipi_test.py` |
| `RSIPI_OnlySend.src` | ONLYSEND check: streams for 60 s with **no** `RSI_MOVECORR` and no HALT | `examples/onlysend_monitor.py` |
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
