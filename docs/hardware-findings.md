# Hardware Bring-Up Findings

Facts learned bringing RSIPI up on a real robot: a KUKA KR 16-2 on a KRC4
controller, KSS 8.3, RSI 3.x. This is a findings log, not a tutorial — for
installation steps and the staged first-run procedure, see
[controller-setup.md](controller-setup.md) and
[controller/README.md](../controller/README.md). Everything here is
hardware-verified unless explicitly marked otherwise.

## 1. Status: what is proven on hardware

RSIPI achieved sustained bidirectional RSI communication with a real KR 16-2:

| Metric | Result |
|---|---|
| Packet rate | 263 packets/s each way at the 4 ms `IPO_FAST` cycle |
| IPOC advance | exactly +1000/s |
| Late-packet counter (`DEF_Delay`) | 0 throughout |
| Mean cycle time | 4.00 ms |
| Jitter | 0.29-0.54 ms |
| Packet loss | 0% |

Verified working end to end:

- Connection and diagnostics
- Cartesian corrections and real motion (20 mm out-and-back, accurate to 0.1 mm)
- Velocity-profiled moves
- Safety limits — write-time rejection and send-time clamp
- Software E-stop — stops mid-move, does **not** resume on reset (measured
  drift after reset: 0.10 mm)
- CSV logging (1180 rows, correct `Send.*` columns) and report generation
- Reconnect
- Digital outputs and digital inputs
- Joint corrections

## 2. Deploying: file layout and startup order

### Copy-as-a-unit rule

Each RSI context is **one unit**: the `.rsi`, `.rsi.xml`, `.rsi.diagram`,
and its `RSI_EthernetConfig_*.xml` must always be copied to the controller
together, from the same build. A `.rsi` copied without its matching config
(or vice versa) produces `RSI_CREATE: Invalid index - signal output` — the
object graph references an ETHERNET output index the config's `RECEIVE`
section never declares. `examples/validate_context.py` catches this offline
before you ever load the controller:

```
python examples/validate_context.py controller/SensorInterface/RSIPI_Basic.rsi.xml
```

File destinations on the controller (see
[controller/README.md](../controller/README.md) for the full table):

| Repo folder | Controller destination |
|---|---|
| `controller/SensorInterface/` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `controller/Program/` | `C:\KRC\ROBOTER\KRC\R1\Program\` |

### Startup order (this bites everyone once)

The ETHERNET object begins its 4 ms exchange the instant `RSI_ON` runs, and
reports `Signal flow (running): Object ETHERNET1 returns error RSIBad`
(KSS29002) after `Timeout` unanswered cycles — 100 x 4 ms = 0.4 s with the
shipped contexts. **The PC must already be listening before `RSI_ON`
executes.** `controller/Program/RSIPI_Minimal.src` HALTs before `RSI_ON` for
exactly this reason: start the Python sender first, then press Start on the
pendant to let the program past the HALT.

## 3. Protocol facts and failure modes

### Facts (each cost real debugging time)

1. **IPOC echo.** The reply must echo the robot's IPOC value **unchanged**.
   RSIPI previously sent IPOC+4, which a real controller rejects. IPOC is a
   millisecond timestamp that the robot itself advances, at ~1000/s at the
   4 ms cycle.

2. **`DataSize` on `MAP2DIGOUT` / `DIGIN` / `DIGOUT` is an enum (Bit / Byte
   / Word), not a count.** Per the official object reference
   (`Manuals/RsiElements.chm`), with any `DataSize` other than `Bit`,
   `Index` is a **byte** index. The shipped `MAP2DIGOUT1` uses `Index=20`,
   `DataSize=Word`, so the DiO word drives `$OUT[161]`-`$OUT[176]`, **not**
   `$OUT[20]`. Likewise `DIGIN1` (`Index=1`, `DataSize=Byte`) reads
   `$IN[9]`-`$IN[16]`, so `DiL` bit 0 is `$IN[9]`. Hours were lost watching
   the wrong outputs on the pendant. (This is documented in
   [controller/README.md](../controller/README.md) as well — see it there
   for the full bit map.)

3. **`POSCORR` limits cap the total correction, not the per-cycle delta.**
   KUKA's example ships ±5 mm / 5°; in relative mode corrections
   accumulate, so every move silently stopped at exactly 5.00 mm with no
   error anywhere. This is the single most confusing failure mode on a
   first bring-up — nothing on the pendant or in the RSI reply indicates a
   limit was hit, motion just stops. The shipped contexts raise this to
   ±50 mm / 45° (`AXISCORR` to ±10°/axis).

4. **`AIPos` vs `ASPos`.** While RSI corrections are applied, `ASPos` (axis
   setpoint) holds the *programmed* value and never reflects the
   correction; only `AIPos` (axis actual) moves. Measured directly: holding
   an `AKorr` correction moved `AIPos.A6` from 2.0 to 11.7 deg while
   `ASPos.A6` stayed at 2.0. The Cartesian side has the same relationship
   (`RIst` actual vs `RSol` setpoint). RSIPI's `get_current_joints()` used
   to read `ASPos`; it now prefers `AIPos` (see `src/RSIPI/motion_api.py`).

5. **Technology parameter syntax.** `$TECH.C[11]` / `$TECH.T[11]` is
   invented syntax that does not exist — it produces "variable not
   declared" on every line. `$TECH[n]` is the function-generator
   *configuration* struct (mode/class/fct), not parameter storage. The
   correct KSS arrays are `$TECHPAR_C[fg,idx]` (main run, wire tag
   `Tech.Cnm`) and `$TECHPAR[fg,idx]` (advance run, wire tag `Tech.Tnm`),
   where `fg` is the function generator (1-6) and `idx` the parameter
   (1-10). So wire attribute `C110` is `$TECHPAR_C[1,10]`. **These names
   were confirmed to display on the pendant (Display > Variable > Single)
   but have not yet been exercised in a running handshake** — see
   [Section 6](#6-open--unverified-items).

6. **Function generator 1 is reserved for RSI's own corrections**
   (`RSITECHIDX` defaults to 1). Declaring `DEF_Tech.T1` in `RECEIVE`
   caused `RSIBad`. KUKA's own example uses `DEF_Tech.T2` for this reason;
   RSIPI's shipped contexts follow suit — `Tech.T2` / `$TECHPAR[2,n]` is
   the PC->KRL command channel, `Tech.C1` / `$TECHPAR_C[1,n]` is the
   KRL->PC state channel.

7. **External-axis objects need external axes to exist.** `AXISCORREXT`
   (and `AXISCORRMON`'s E1-E6 channels) cause `ETHERNET1 returns RSIBad` on
   a robot with no external axes configured. They belong only in cells
   that actually have them (`RSIPI_Full`).

8. **KRL syntax gotchas** found loading the shipped programs:
   - A bare `INI` line is **not** a statement — it parses as a call and
     errors with `"(" expected`. Initialisation must be the `;FOLD INI` /
     `BAS (#INITMOV,0 )` block, exactly as in KUKA's example (see
     `controller/Program/RSIPI_Minimal.src`).
   - `&H NOBOUNDSCHECK` is not a valid header.
   - Pendant INLINE-FORM motion (`PTP HOME Vel=100 % DEFAULT`,
     `LIN target_pos Vel=0.5 m/s CPDAT1 Tool[1] Base[0]`) only compiles
     inside a pendant-generated FOLD carrying its `CPDAT`/`PDAT`
     declarations. In plain KRL, use `$VEL.CP = 0.5` + `LIN target_pos`,
     and `BAS(#VEL_PTP, 100)` + `PTP <stored E6POS>`.

9. **`Precision`** (ETHERNET parameter, default `1` in all shipped
   contexts) sets the decimal places in data the *robot* sends. With
   `Precision=1`, `RIst` is reported to 0.1 mm, so a 2.0 mm move can
   measure as 1.9. The controller always parses all decimals it
   *receives* regardless of this setting. Raising `Precision` (e.g. to 4)
   should improve feedback resolution — **untested on hardware**.

### Symptom -> cause -> fix

| Symptom | Cause | Fix |
|---|---|---|
| `Signal flow (running): Object ETHERNET1 returns error RSIBad` right after `RSI_ON` | Nothing answered within `Timeout` cycles (100 x 4 ms = 0.4 s) — PC wasn't listening yet | Start the Python side first; let the program past the HALT in `RSIPI_Minimal.src` |
| `RSI_CREATE: Invalid index - signal output` | `.rsi` and `RSI_EthernetConfig_*.xml` are from different builds / don't agree on channels | Copy the `.rsi` + `.rsi.xml` + `.rsi.diagram` + config as one unit; run `examples/validate_context.py` offline first |
| Move stops at exactly 5 mm (or 5°) with no error anywhere | `POSCORR`/`AXISCORR` limits cap the *total* correction, and relative-mode corrections accumulate | Raise the `LowerLim*`/`UpperLim*` params in the `.rsi` **and** `.rsi.xml` (shipped contexts already raise these to ±50 mm/45°) |
| Outputs seem dead — nothing happens on the expected `$OUT` number | `DataSize=Word`/`Byte` makes `Index` a *byte* index, not an output number | Watch the correct range (shipped `MAP2DIGOUT1`: `$OUT[161]`-`$OUT[176]`) or set `DataSize=Bit` to address a single output directly |
| Joints appear not to move even though corrections are being sent | Reading `ASPos` (setpoint), which never reflects an applied RSI correction | Read `AIPos` (actual) instead — RSIPI's `get_current_joints()` does this now |
| `"variable not declared"` on `$TECH...` lines | Invented `$TECH.C[11]`/`$TECH.T[11]` syntax, or `$TECHPAR`/`$TECHPAR_C` not available under that name on this KSS version | Use `$TECHPAR[fg,idx]` / `$TECHPAR_C[fg,idx]`; confirm the names exist via Display > Variable > Single before relying on them |
| `"(" expected` when loading a KRL program | A bare `INI` line outside the `;FOLD INI` block | Use the `;FOLD INI` / `BAS (#INITMOV,0 )` block as shipped |
| No packets reach the PC at all | Windows Firewall blocking the venv's exact interpreter path, or KUKA `TestServer.exe` still holding the port | Add an inbound UDP rule for the port (see [Section 4](#4-pc-side-gotchas)); confirm nothing else owns the port |

## 4. PC-side gotchas

- **Firewall rules match the exact interpreter path.** A rule scoped to a
  system Python install does not cover a virtualenv's `python.exe`. The
  RSI adapter is usually classified "Public" (no gateway), where inbound
  traffic is blocked by default. Allow the UDP port instead (elevated
  PowerShell):

  ```powershell
  New-NetFirewallRule -DisplayName "RSIPI UDP 64000" -Direction Inbound `
    -Protocol UDP -LocalPort 64000 -Action Allow -Profile Any
  ```

- **KUKA's `TestServer.exe` left running holds the UDP port.** RSIPI used
  to set `SO_REUSEADDR`, so a second bind "succeeded" and then silently
  received nothing. RSIPI now claims the port exclusively
  (`src/RSIPI/network_handler.py`) and fails loudly instead of going quiet.

- **RSIPI binds the config's `IP_NUMBER`.** Once the RSI adapter has that
  static IP, loopback testing against the echo server needs the server to
  target the same address — handled automatically.

- **UDP receive buffers are 64 KB.** The Full config's telegrams exceed
  1 KB; a 1024-byte buffer raised `WinError 10040` on Windows and made the
  Full config unusable. `src/RSIPI/network_handler.py` and
  `src/RSIPI/rsi_echo_server.py` now both call `recvfrom(65535)`.

## 5. Shipped contexts (`controller/SensorInterface/`)

| Context | Contents | Status |
|---|---|---|
| `RSIPI_Basic` | KUKA's `RSI_Ethernet` example, structurally unchanged apart from the `ConfigFile` name and raised `POSCORR` limits. `RKorr`, `DiO`, `$SEN_PREA`, `Tech.C1`/`Tech.T2`. | **Hardware-verified.** The default. |
| `RSIPI_Joints` | `RSIPI_Basic` + `AXISCORR` (`AKorr.A1`-`A6` on RECEIVE channels 9-14) + joint feedback (`DEF_AIPos`/`DEF_ASPos`) + a `DoutW` read-back channel (a `DIGOUT` over the same byte range `MAP2DIGOUT1` writes, so Python can confirm outputs without the pendant). | **Hardware-verified**: joints move, outputs read back correctly. |
| `RSIPI_Full` | Adds external-axis correction (`EKorr`/`AXISCORREXT`) and monitors. | Known to fail with `RSIBad` on a 6-axis robot; for external-axis cells only. **Not hardware-verified.** |

`RSIPI_Joints` is the recommended default for a 6-axis cell: it is verified
on hardware and adds joint control plus the output read-back channel, at the
cost of one more object than `RSIPI_Basic`. Use `RSIPI_Basic` when you want
the most conservative option — it is structurally KUKA's own example.

## 6. Test tooling (`examples/`)

All scripts run with the repo venv: `.venv\Scripts\python.exe examples\<script>.py <config>`.

| Script | Purpose |
|---|---|
| `kuka_example_server.py` | Drop-in replacement for KUKA's `TestServer.exe`; monitors packets, optional `--jog` (2 mm Cartesian move) and `--joints` (joint diagnostic printing `AIPos`/`ASPos` and holding a raw `AKorr`), plus an automatic DiO read-back check. |
| `feature_test.py` | Staged session covering diagnostics, digital I/O, trajectory accuracy, velocity profiles, safety limits, an E-stop drill, CSV logging + report, and reconnect. Prints a PASS/FAIL summary. Last hardware run: 22/24 (the 2 failures were the `$OUT` mix-up, since explained above, and a deliberately skipped reconnect stage). |
| `first_contact.py` | Gated three-stage first-contact script (connect-only, a 5 mm move, an E-stop drill). |
| `udp_probe.py` | Raw UDP listener — proves packets reach the PC independently of RSIPI. |
| `validate_context.py` | Offline check that a context and its config agree (catches the failure in [Section 2](#2-deploying-file-layout-and-startup-order)). |

## 7. Open / unverified items

- **`$TECHPAR`/`$TECHPAR_C` handshake not yet exercised in a running
  program.** The names were confirmed to display correctly on the pendant
  (Display > Variable > Single: `$TECHPAR[2,1]`, `$TECHPAR_C[1,1]`), but no
  hardware run has yet driven a live Tech.C/Tech.T exchange end to end.
- **`RSIPI_Full` (external-axis context) is not hardware-verified.** It has
  only been confirmed to fail predictably (`RSIBad`) on a 6-axis robot with
  no external axes, which is expected — it needs a cell with configured
  external axes to test properly.
- **Raising `Precision` above the default of 1 is untested.** Expected to
  improve `RIst` feedback resolution beyond 0.1 mm but not yet measured.
- **External axes are untestable on this cell** — the KR 16-2 used for this
  bring-up has no external axes, so `AXISCORREXT`/E1-E6 monitoring remains
  unverified for lack of hardware to test against.
