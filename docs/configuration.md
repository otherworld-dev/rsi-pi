# Configuration

RSIPI reads an RSI Ethernet config to determine network settings and which variables are exchanged with the robot. Six ship with the package, resolved by name with `context()`; `available_contexts()` lists them and `describe_contexts()` explains each:

| Context | What it wires | On hardware |
|---------|---------------|-------------|
| `basic` | Cartesian corrections, digital I/O, `$SEN_PREA`, Tech parameters — KUKA's own example, unchanged | verified |
| `joints` (default) | `basic` + joint corrections, joint feedback, digital-output read-back | verified |
| `max` | `joints` + applied-correction monitors, motor currents, analogue I/O, `$SEN_PINT`, program override — everything a 6-axis robot can bind | verified 2026-09-10 (11/11) |
| `full` | `joints` + external axes; reports `RSIBad` at `RSI_ON` on a robot without them | not yet |
| `onlysend` | the robot streams and the PC never replies — logging only, no corrections | verified |
| `stop` | `basic` + a STOP object, to end `RSI_MOVECORR()` from the PC | verified 2026-09-10 |

Every shipped context carries `POSCORRMON`/`AXISCORRMON` objects at 500 mm / 180°: without them the controller applies its own **6 mm / 6° overall-correction default** and stops RSI with `KSS29000` — the first thing that bit on the robot. `config_builder` warns if your own context lacks them.

A context is four files that only work as a set (`.rsi`, `.rsi.xml`, `.rsi.diagram`, `RSI_EthernetConfig_*.xml`), and the same set must be on the controller. Two commands cover the round trip:

```bash
# Copy a shipped context out of the package, ready for the controller
python -m RSIPI.deploy --context joints --out C:\deploy

# Generate the matching Ethernet config for a context you built in RSIVisual
python -m RSIPI.config_builder MyContext.rsi.xml --ip 10.10.10.10 --port 64000
```

`config_builder` derives every channel number, tag and type from what the RSIVisual context already records, so the config cannot disagree with the context — the mismatch behind `RSI_CREATE: Invalid index - signal output`. It reproduces all six shipped configs channel for channel.

**What to put on the controller** (KRC4, KSS 8.3, RSI 3.x — log in as *Expert*):

| What | To (on the controller) |
|---|---|
| The context's **four files, together** — e.g. `RSIPI_Basic.rsi`, `RSIPI_Basic.rsi.xml`, `RSIPI_Basic.rsi.diagram`, `RSI_EthernetConfig_Basic.xml` from `deploy --context basic` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| The KRL program from [`controller/Program/`](https://github.com/otherworld-dev/rsi-pi/tree/main/controller/Program) that creates that context: `RSIPI_Minimal.src` → Basic, `RSIPI_Max.src` → Max, `RSIPI_Stop.src` → Stop, `RSIPI_OnlySend.src` → OnlySend (each names its context in its `RSI_CREATE` line; edit that line to use another, e.g. Joints) | `C:\KRC\ROBOTER\KRC\R1\Program\` |

The `.rsi.xml` carries the object parameters the controller reads, the `.rsi` and `.rsi.diagram` are RSIVisual's view of the same context, and the Ethernet config names the channels — a partial or mixed set fails at `RSI_CREATE`. The PC's RSI interface must be on the IP in the config (`10.10.10.10` as shipped). Then, on the pendant: select the program, run it to its HALT, start the Python side, and press Start — the PC must be listening **before** `RSI_ON`, or the ETHERNET object reports `RSIBad` after 0.4 s of silence. Full walkthrough, network settings and troubleshooting: [Controller setup](controller-setup.md); what each RSI object does and how it reaches KRL: [RSI objects](rsi-objects.md).

```xml
<ROOT>
   <CONFIG>
      <IP_NUMBER>10.10.10.10</IP_NUMBER>   <!-- External PC IP -->
      <PORT>64000</PORT>                    <!-- UDP port -->
      <SENTYPE>ImFree</SENTYPE>             <!-- XML root element name -->
      <ONLYSEND>FALSE</ONLYSEND>            <!-- FALSE = bidirectional -->
   </CONFIG>

   <!-- SEND: What the robot sends TO us (read-only from Python) -->
   <SEND>
      <ELEMENTS>
         <ELEMENT TAG="DEF_RIst" TYPE="DOUBLE" INDX="INTERNAL" />  <!-- TCP position -->
         <ELEMENT TAG="DEF_RSol" TYPE="DOUBLE" INDX="INTERNAL" />  <!-- Commanded position -->
         <ELEMENT TAG="DEF_Delay" TYPE="LONG" INDX="INTERNAL" />   <!-- Packet delay count -->
         <ELEMENT TAG="Digout.o1" TYPE="BOOL" INDX="2" />          <!-- Digital output state -->
      </ELEMENTS>
   </SEND>

   <!-- RECEIVE: What the robot receives FROM us (writable from Python) -->
   <RECEIVE>
      <ELEMENTS>
         <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" HOLDON="1" />  <!-- Cartesian correction -->
         <ELEMENT TAG="RKorr.Y" TYPE="DOUBLE" INDX="2" HOLDON="1" />
         <ELEMENT TAG="RKorr.Z" TYPE="DOUBLE" INDX="3" HOLDON="1" />
         <ELEMENT TAG="RKorr.A" TYPE="DOUBLE" INDX="4" HOLDON="1" />
         <ELEMENT TAG="RKorr.B" TYPE="DOUBLE" INDX="5" HOLDON="1" />
         <ELEMENT TAG="RKorr.C" TYPE="DOUBLE" INDX="6" HOLDON="1" />
         <ELEMENT TAG="DiO" TYPE="LONG" INDX="8" HOLDON="1" />        <!-- Digital I/O -->
      </ELEMENTS>
   </RECEIVE>
</ROOT>
```

*(This is a trimmed illustrative excerpt, not a literal copy of the shipped file --
`RSI_EthernetConfig.xml` also declares `DiL`, `Tech.C1`, `Digout.o2`/`o3`, and `Source1`
in SEND, and `EStr`, `Tech.T2`, and `FREE` in RECEIVE.)*

Key points:
- `DEF_` prefixed tags are expanded internally (e.g., `DEF_RIst` becomes `RIst: {X, Y, Z, A, B, C}`).
- `HOLDON="1"` means the last value is held if no new value is sent.
- SEND variables are read via `api.monitoring` (position, motor currents, IPOC), `api.krl.read_param()` (Tech.C), and `api.io.get_input()` (digital inputs); RECEIVE variables are written via `api.motion`, `api.io`, and `api.krl.write_param()`.
- The config must match the RSI object configuration on the KUKA controller.
