# KUKA Controller Setup Guide

Getting RSIPI talking to a real robot: installing the config files on a KRC4
controller, configuring the network, and troubleshooting the common failure
modes.

Primary source: the official **KUKA.RobotSensorInterface 3.3 manual**
(KST RSI 3.3 V5, issued 2016-10-17, for KSS 8.3/8.4), available from KUKA
via [my.kuka.com](https://my.kuka.com) or your KUKA support contact (it is
KUKA-copyrighted, so it is not redistributed with this repo). Section/page
references below (e.g. "manual §8.1.1, p. 56") point into that document. Claims not covered by the
manual are marked *(practical experience)* and sourced at the bottom.

## Prerequisites

- **KRC4 controller** with **KSS 8.3 or 8.4** and the
  **KUKA.RobotSensorInterface 3.x** option package installed
  (manual §4.1, p. 25). Check **Help > Info > Options** on the pendant.
  RSI provides `RSI_CREATE`, `RSI_ON`, `RSI_MOVECORR`, `RSI_OFF`,
  `RSI_DELETE` (manual §7.1, p. 39).
- RSI must **not** be installed together with KUKA.ConveyorTech or
  KUKA.ServoGun TC (manual §4.1, p. 25).
- For corrections in `#IPO` sensor mode, **function generator 1 must be
  free** (manual §4.1, p. 25; configurable via `RSITECHIDX` in
  `KRC:\R1\TP\RSI\RSI.DAT`, manual §5.2, p. 30).
- PC side: Python 3.10+ with RSIPI installed (`pip install -e .`), on a
  network interface reachable from the controller. KUKA specifies a
  "real-time-capable operating system and real-time-capable network card
  with 100 Mbit in full duplex mode" (manual §4.1, p. 25) — a normal
  Windows/Linux PC works for testing but is not real-time capable, so
  expect occasional late packets (see [Timing](#timing-requirements)).

## 1. Install the files

The `controller/` folder mirrors the controller's layout: everything in
`controller/SensorInterface/` goes to the SensorInterface directory, and
everything in `controller/Program/` goes to the KRL program directory. This
split is specified in manual §8.1.1, p. 56:

> 2. Copy KRL programs into the directory `C:\KRC\ROBOTER\KRC\R1\Program`
>    of the robot controller.
> 3. Copy the sample configurations and the XML file for the Ethernet
>    connection to the directory
>    `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` of the robot
>    controller.

| Repo folder | Destination on controller |
|---|---|
| `controller/SensorInterface/*` (the context `.rsi` + `.rsi.xml` + `.rsi.diagram` and its `RSI_EthernetConfig_*.xml`) | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `controller/Program/*.src` (`RSIPI_Minimal`, `RSIPI_Test`, `RSIPI_OnlySend`, and the optional coordination templates) | `C:\KRC\ROBOTER\KRC\R1\Program\` (= `KRC:\R1\Program` in the Navigator) |

Four contexts ship: **`RSIPI_Joints`** (default — any 6-axis robot,
Cartesian + joint corrections), **`RSIPI_Basic`** (Cartesian only, the most
conservative), **`RSIPI_Full`** (adds external-axis corrections; only
for cells that have them), and **`RSIPI_OnlySend`** (data logging — the
robot streams and the PC never replies, so no corrections are possible).
Copy the four files of the one you need — the
three context files plus its config. See
[controller/README.md](../controller/README.md) for the comparison and
[docs/hardware-findings.md](hardware-findings.md) for what is verified.

Watch the paths carefully — they are confusingly similar:

- KRL programs go under `...\ROBOTER\KRC\R1\Program` (**with** `\KRC\`).
- SensorInterface files go under `...\ROBOTER\Config\User\Common\...`
  (**without** a second `\KRC\`). Putting them in
  `C:\KRC\ROBOTER\KRC\Config\User\Common\SensorInterface` is a common
  mistake; `RSI_CREATE` will not find them there.
- A `.src` file placed in the SensorInterface folder never appears as a
  program, and a program that isn't loaded through `KRC:\R1\Program` throws
  "variable not defined" on every RSI keyword.

Notes:

- The RSI, DIAGRAM and XML files "form a unit and must be transferred to
  the robot controller together" (manual §8.1, p. 55).
- **Do not rename the files.** Each `.rsi` names its own
  `RSI_EthernetConfig_*.xml` in its ETHERNET object, and the `.src`
  programs name the `.rsi` in `RSI_CREATE` (both ship pointing at
  `RSIPI_Basic.rsi`).
- Copy `.src` files as the **Expert** user group (log on via
  Configuration > User group).

### What the shipped context contains

`RSIPI_Full.rsi` wires 16 RSI objects around one `ETHERNET` object
(`Timeout=100`, `ConfigFile=RSI_EthernetConfig_Full.xml`). The channel
numbering is documented in full in the header comment of
`RSI_EthernetConfig_Full.xml`; in short:

| Direction | Channels | Objects |
|---|---|---|
| SEND (robot → PC) | 1-29 | `DIGIN1` (1), `DIGOUT1-3` (2-4), `SOURCE1` (5), `MOTORCURRENT1` (6-11), `POSCORRMON1` (12-17), `AXISCORRMON1` (18-29) |
| RECEIVE (PC → robot) | 1-19 | `POSCORR1` (1-6), `AXISCORR1` (7-12), `AXISCORREXT1` (13-18), `MAP2DIGOUT1` (19); `MAP2SEN_PREA1-3` also read channels 1-3 |

Robot pose, axis positions and motor currents are **not** read through
numbered channels: `RSI_EthernetConfig_Full.xml` declares them as
`DEF_RIst`, `DEF_RSol`, `DEF_AIPos`, `DEF_ASPos`, `DEF_EIPos`, `DEF_ESPos`,
`DEF_MACur`, `DEF_MECur` with `INDX="INTERNAL"`, which RSI reads straight
from the system variables and which cost no channels (manual §8.1.2.1,
p. 58). That is also how KUKA's own examples and real production
configurations do it, so no `POSACT`/`AXISACT` source objects are needed.

## 2. Configure the network

RSI needs "its own Ethernet sensor network which is independent of other
KLI subnetworks", physically connected via the KLI ports X66 or X67.1-3 on
the controller cabinet (manual §5.1, p. 29).

### Controller side

Two documented ways to set the robot's RSI IP address (manual §5.1.1 and
§5.1.2, pp. 29–30):

**A. KLI network configuration on the pendant** (requires KSS 8.3.15+,
Expert group, T1/T2, no program selected):

1. Main menu > **Start-up > Network configuration** > **Advanced...**
2. **Add interface**; name it (e.g. "Ethernet sensor network").
3. Set **Address type: Mixed IP address** — this auto-creates the
   real-time UDP receiving tasks RSI needs.
4. Enter the robot's IP and subnet mask, **Save**, and **reboot the
   controller**.

**B. RSI-Network tool in Windows** (Expert group): minimize the HMI
(Start-up > Service > Minimize HMI), run **All Programs > RSI-Network**,
select the entry **New** under *RSI Ethernet* in the tree, press **Edit**,
enter the robot controller's IP, confirm, then **cold restart** the
controller.

IP address rules (manual §5.1.1, p. 29):

- PC and controller must be "located in the same network segment", i.e.
  only the last of the 4 octets may differ.
- "The IP address range **192.168.0.x is blocked**" and the address "must
  not be in the address range of another KLI subnet". The manual's example
  uses robot `192.168.1.2` / mask `255.255.255.0` with the sensor (PC) at
  `192.168.1.1`. This repo's default config uses `10.10.10.x`, which
  safely avoids all KUKA-internal ranges *(practical experience — see
  Sources)*.

### "Address already in use" when adding the interface

This error is not documented in the manual *(practical experience)*: it
appears when the address you enter collides with an interface entry that
already exists — a leftover RSI Ethernet entry, or another KLI subnet
containing the same address. Fix: in the interface tree, **edit or delete
the existing entry instead of adding a second one** (note the documented
RSI-Network procedure edits the existing "New" entry rather than creating
another), and make sure the subnet doesn't overlap any other configured
interface. Reboot (cold restart) afterwards — network changes only take
effect after a reboot (manual §5.1.1 step 8 / §5.1.2 step 5, p. 30).

### PC side

Give the PC a **static IP** on the RSI link and put that address in the
Ethernet config. The Python side loads the very same file out of
`controller/SensorInterface/`, so there is only one copy to edit — but
remember to re-copy it to the controller after any change, or the two ends
will disagree about the telegram structure:

```xml
<CONFIG>
   <IP_NUMBER>10.10.10.10</IP_NUMBER>   <!-- your PC's IP -->
   <PORT>64000</PORT>
   <SENTYPE>ImFree</SENTYPE>
   <ONLYSEND>FALSE</ONLYSEND>
</CONFIG>
```

Pick a free UDP port "not assigned a standard service" (manual §8.1.2.1,
p. 58). Allow that UDP port through the PC firewall, or disable the
firewall on the dedicated RSI interface *(practical experience)* — a
blocked firewall shows up as "Python receives nothing" while a packet
capture still sees traffic arriving.

On Windows the RSI adapter is usually classified **Public** (it has no
gateway), where inbound traffic is blocked by default, and firewall rules
match the **exact interpreter path** — a rule for a system Python does not
cover a virtualenv's `python.exe`. Allow the port itself instead, in an
**elevated** PowerShell *(practical experience)*:

```powershell
New-NetFirewallRule -DisplayName "RSIPI UDP 64000" -Direction Inbound `
  -Protocol UDP -LocalPort 64000 -Action Allow -Profile Any
```

Verify the path end-to-end before blaming RSIPI: `examples/udp_probe.py`
prints every datagram that reaches the PC (it does not reply, so the robot
will stop with `RSIBad` — expected). Ping also helps: the controller
answers on its RSI address, so `ping 10.10.10.1` proves link and routing
while saying nothing about UDP delivery.

### Timing requirements

The robot controller initiates the exchange and sends a packet every
sensor cycle; "a data packet received by the sensor system must be
answered within the sensor cycle rate. Packets that arrive too late are
rejected. When the maximum number of data packets for which a response has
been sent too late has been exceeded, the robot stops." (manual §2.3.2,
p. 15). The sensor cycle is **4 ms** in `#IPO_FAST` (default) or **12 ms**
in `#IPO` mode (manual §7.1.4, p. 41). The allowed number of late packets
is the `Timeout` parameter of the ETHERNET object — set to 100 in
`RSIPI_Full.rsi`.

## 3. First run: minimal test

`RSIPI_Minimal.src` mirrors KUKA's official `RSI_Ethernet.src` example
(manual §8.1.3, p. 59: `RSI_CREATE` → `RSI_ON(#RELATIVE)` →
`RSI_MOVECORR()` → `RSI_OFF`) and uses no `$TECHPAR` variables, no I/O and no
`$SEN_PREA` — so it isolates "does RSI work at all" from everything else.

**Order matters.** The ETHERNET object starts the 4 ms exchange the moment
`RSI_ON` runs and reports `RSIBad` (KSS29002, robot stops) after `Timeout`
unanswered cycles — 100 × 4 ms = 0.4 s with the shipped context. The PC
must therefore be listening *before* `RSI_ON`; `RSIPI_Minimal` HALTs to let
you do that:

1. On the pendant (T1 mode, low override): select `RSIPI_Minimal` and press
   Start. It does a BCO run to the current position and HALTs with
   *"Start the Python sender, then press Start"*.
2. On the PC, from the repo root, start the sender — e.g.
   `examples/first_contact.py`, or the snippet below. It binds the UDP port
   and waits (up to 30 s) for the robot's first packet.
3. On the pendant, press Start again. `RSI_ON` runs, the exchange begins, and
   the program reports *"RSI active - corrections from Python are live"*. The
   robot holds position, applying whatever corrections Python sends.

Minimal sender:

   ```python
   from RSIPI import RSIAPI

   # max_cartesian_rate clamps every cycle to 0.1 mm even if something
   # writes a large correction - keep it on for all first-contact testing.
   api = RSIAPI("controller/SensorInterface/RSI_EthernetConfig_Full.xml", max_cartesian_rate=0.1)
   api.safety.set_limit("RKorr.X", -6.0, 6.0)   # write-time + send-time clamp
   api.start()
   if not api.wait_for_connection(timeout=30.0):
       raise SystemExit("No packets from robot - see troubleshooting table")

   pose = api.motion.get_current_pose()
   print("Connected:", pose)

   # Move 5mm in X, paced one delta per robot cycle (~0.1 mm/cycle = 25 mm/s)
   # NEVER hold a raw update_cartesian() value in relative mode - a held
   # X=5.0 would be re-applied EVERY 4ms cycle (1250 mm/s), not once.
   api.motion.move_cartesian_trajectory({"X": pose["X"] + 5.0}, steps=50)

   print("Done:", api.motion.get_current_pose())
   input("Press Enter to stop...")
   api.stop()
   ```

3. Stop by cancelling the program on the pendant. Stopping the Python side
   instead also works: after `Timeout` late packets the robot stops and
   reports an RSI communication error — expected and safe (manual §2.3.2,
   p. 15). (A clean stop signal would require a STOP object in the signal
   flow, manual §7.1.6, p. 42 — `RSIPI_Full.rsi` doesn't include one.)

Keep first corrections small. The signal flow limits Cartesian corrections
to ±500 mm (`POSCORR1` in `RSIPI_Full.rsi`), which is generous — tighten it
in RSI Visual and/or set software limits on the Python side via
`api.safety.set_limit()` before doing anything real.

Independent cross-check: KUKA ships a Windows test server (`TestServer.exe`)
with the RSI option package for exactly this purpose (manual §8.1.2, p. 56)
— find it on the controller under
`D:\KUKA_OPT\RSI\DOC\Examples\Ethernet\Server` (manual §8.1, p. 55). If the
robot exchanges packets with TestServer but not with RSIPI, the problem is
on the Python/firewall side, not the robot.

## 4. Full test

`RSIPI_Test.src` + `examples/rsipi_test.py` exercise the whole feature
set: corrections, `$SEN_PREA` (via MAP2SEN_PREA objects, manual §11.2.10,
p. 81), digital I/O, and a handshake over the `Tech.C` / `Tech.T` channels.
Start `rsipi_test.py` **first** (it waits up to 30 s for the KRL state
handshake), then start `RSIPI_Test` on the pendant — for the same 0.4 s
`RSIBad` reason as in §3, the PC must be answering before `RSI_ON`.
Those channels are "technology parameters in the main run / advance run
(function generators 1 to 6)" (manual §7.4.5–7.4.6, pp. 52–53): on the
wire, `DEF_Tech.C1` carries the ten parameters of function generator 1 as
attributes `C11…C110`.

On the KRL side these are the KSS technology-parameter arrays: `Tech.Cnm`
(main run) is `$TECHPAR_C[n,m]` and `Tech.Tnm` (advance run) is
`$TECHPAR[n,m]`, with `n` the function generator (1-6) and `m` the
parameter (1-10) — so the wire attribute `C110` is `$TECHPAR_C[1,10]`.

Earlier versions of the test program used an invented `$TECH.C[11]` syntax.
`$TECH[n]` is the function-generator *configuration* structure (mode,
class, fct), not parameter storage, so every such line failed to load with
"variable not declared"
([issue #1](https://github.com/otherworld-dev/rsi-pi/issues/1)); that is
the root cause, not a missing technology package.

**Confirm the names on your controller before relying on the Tech
handshake**: on the smartHMI, Display > Variable > Single, enter
`$TECHPAR[2,1]` and `$TECHPAR_C[1,1]` — each should display a value. If
either is reported as unknown, run `RSIPI_Minimal.src` instead (everything
except the Tech handshake works without them) and please open an issue with
your KSS version.

## External axis corrections

The shipped `RSIPI_Full.rsi` context wires the RECEIVE channels
`EKorr.E1`-`EKorr.E6` (channels 13-18 in `RSI_EthernetConfig_Full.xml`)
into an `AXISCORREXT1` object, giving Python-side correction of external
axes E1-E6 (`api.motion.move_external_axis('E1', ...)`) the same way
`AXISCORR1` already handles A1-A6. Default correction limits are ±5 (mm or
deg, per axis configuration) — tighten these in RSIVisual for your cell.

This only works on cells with **configured external axes** (e.g. a linear
track or positioner) — `AXISCORREXT` has nothing to act on otherwise.
**Cells without external axes must delete `AXISCORREXT1` from the context**
(or use `RSIPI_Minimal`/a config without the object) before loading it;
leaving an unusable object wired in is at best inert and at worst a load
error, depending on controller configuration.

`AXISCORREXT` shares `ObjTypeID` **`33`** with `AXISCORR` — confirmed
against a real RSIVisual export from a KRC4/RSI 3.x installation AND the
official object reference (`Manuals/RsiElements.chm`, whose pages encode
every type ID, port and parameter — the authoritative source for any
future context work; note ETHERNET's `Timeout` official default is 10
cycles, this project ships 100). The two objects are distinguished by their offsets: `AXISCORREXT`
carries `inputOffset="7"` / `outputOffset="8"` in the `.rsi` model, takes
`InIdx` 7-12 in the `.rsi.xml`, and names its parameters `LowerLimE1`-`E6`
(ParamID 7-12) and `UpperLimE1`-`E6` (ParamID 19-24), continuing
`AXISCORR`'s `LowerLimA1`-`A6` (1-6) / `UpperLimA1`-`A6` (13-18). Keep
those offsets and ParamIDs intact if you edit the files by hand.

**After editing any of the `.rsi`/`.rsi.xml`/`.diagram`/`RSI_EthernetConfig_Full.xml`
files, re-open the context in RSIVisual to validate it** — this surfaces a
wrong `ObjTypeID` or any other structural mistake immediately. Re-copy all
four files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` after any
edit; the controller reads whatever is on disk there, not what's in this
repo.

## 5. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "Variable not defined" on **every** line / `RSI_CREATE`, `RSI_ON`, `RSIOK` unknown | `.src` file not in `KRC:\R1\Program`, or RSI option not installed | Move the `.src` (see §1); check RSI appears under Help > Info > Options |
| "Variable not declared" only on `$TECHPAR…` lines | Technology-parameter arrays not available under that name on this KSS version | Check `$TECHPAR[2,1]` in Display > Variable > Single; use `RSIPI_Minimal.src` meanwhile (see §4) |
| "Address already in use" when creating the RSI Ethernet entry | An interface entry with that address/subnet already exists | Edit or delete the existing entry instead of adding a new one; reboot (see §2) |
| `RSI_CREATE` returns `RSIFILENOTFOUND` / `RSIINVFILE` | Config files missing from `Config\User\Common\SensorInterface` (check for the extra-`\KRC\` path mistake), renamed, or `.rsi`/`.rsi.xml`/`.diagram` set incomplete | Re-copy all four config files together, keep original names (manual §7.1.2, p. 40 for return codes) |
| `RSI_ON` fails / RSI communication error immediately | IP in `RSI_EthernetConfig_Full.xml` doesn't match the PC, or subnet mismatch (only last octet may differ), or PC in `192.168.0.x` | Fix `<IP_NUMBER>`; follow the IP rules in §2 |
| Robot program runs but Python sees no packets | PC firewall blocking the UDP port (rules match the exact interpreter path — a venv `python.exe` needs its own rule, or allow the port), or RSIPI bound to the wrong interface | Add the inbound UDP rule from §2; confirm with `examples/udp_probe.py`, then KUKA's `TestServer.exe` (see §3) |
| `ETHERNET1 returns error RSIBad` right after `RSI_ON` | Nothing answered within `Timeout` cycles (100 × 4 ms = 0.4 s) — the PC must already be listening when `RSI_ON` runs | Start the Python side first, then press Start past the HALT in `RSIPI_Minimal` (see §3) |
| Robot stops mid-motion with RSI error after running fine | Response packets late: Python loop blocked, or non-dedicated network | Keep the correction loop free of blocking work; use a dedicated interface; watch the `Delay` variable (count of late packets, manual §7.4.5, p. 52) |
| Corrections apply but motion is jerky | Late/rejected packets | Same as above — dedicated network, faster loop |

Still stuck? Open an issue with your KSS version, RSI version (Help > Info >
Options), and the exact pendant error message.

## Sources

- **KUKA.RobotSensorInterface 3.3** manual, KST RSI 3.3 V5, KUKA Roboter
  GmbH, 2016-10-17 — from KUKA ([my.kuka.com](https://my.kuka.com)).
  All "manual §…" references above.
- [Michael Sobrepera, "KUKA Setup Guide"](https://michaelsobrepera.com/guides/kuka.html)
  — practical KRC4 + RSI walkthrough (network interface creation, firewall,
  file copying).
- Robot-Forum: [KRC4 RSI Ethernet](https://www.robot-forum.com/robotforum/thread/12240-krc4-rsi-ethernet/)
  and [IP addresses in network configuration of KRC4](https://www.robot-forum.com/robotforum/thread/16990-ip-addresses-in-network-configuration-of-krc4/)
  — reserved KRC4 subnets (`192.168.0.x` shared-memory driver) and RSI
  network setup experience.
