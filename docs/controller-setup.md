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

The files in `rsi_config/` go to **two different places**. This split is
specified in manual §8.1.1, p. 56:

> 2. Copy KRL programs into the directory `C:\KRC\ROBOTER\KRC\R1\Program`
>    of the robot controller.
> 3. Copy the sample configurations and the XML file for the Ethernet
>    connection to the directory
>    `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` of the robot
>    controller.

| File | Destination on controller |
|---|---|
| `RSIPI_Full.rsi` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `RSIPI_Full.rsi.diagram` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `RSIPI_Full.rsi.xml` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `RSI_EthernetConfig_Full.xml` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `RSIPI_Minimal.src` | `C:\KRC\ROBOTER\KRC\R1\Program\` (= `KRC:\R1\Program` in the Navigator) |
| `RSIPI_Test.src` | `C:\KRC\ROBOTER\KRC\R1\Program\` (= `KRC:\R1\Program` in the Navigator) |

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
- **Do not rename the files.** `RSIPI_Full.rsi` references
  `RSI_EthernetConfig_Full.xml` by name in its ETHERNET object, and the
  `.src` programs load `"RSIPI_Full.rsi"` by name in `RSI_CREATE`.
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

Give the PC a **static IP** on the RSI link and put that address in
`RSI_EthernetConfig_Full.xml` (both the copy on the controller and the copy
the Python side loads — they must match):

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
`RSI_MOVECORR()` → `RSI_OFF`) and uses no `$TECH` variables, no I/O and no
`$SEN_PREA` — so it isolates "does RSI work at all" from everything else.

1. On the pendant (T1 mode, low override): select `RSIPI_Minimal` and start
   it. It does a BCO run to the current position, activates RSI, and reports
   *"RSI active - start the Python sender now"*. The robot holds position,
   waiting for corrections.
2. On the PC, from the repo root:

   ```python
   from RSIPI import RSIAPI

   api = RSIAPI("RSI_EthernetConfig_Full.xml")
   api.start()
   if not api.wait_for_connection(timeout=30.0):
       raise SystemExit("No packets from robot - see troubleshooting table")

   print("Connected:", api.motion.get_current_pose())
   api.motion.update_cartesian(X=5.0)   # small 5mm correction
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

`RSIPI_Test.src` + `rsi_config/rsipi_test.py` exercise the whole feature
set: corrections, `$SEN_PREA` (via MAP2SEN_PREA objects, manual §11.2.10,
p. 81), digital I/O, and a handshake over the `Tech.C` / `Tech.T` channels.
Those channels are "technology parameters in the main run / advance run
(function generators 1 to 6)" (manual §7.4.5–7.4.6, pp. 52–53): on the
wire, `DEF_Tech.C1` carries the ten parameters of function generator 1 as
attributes `C11…C110`.

On the KRL side the test program reads/writes these as `$TECH.C[11]` etc.
On at least one user's KSS 8.3.38 installation these `$TECH` references
fail to load with "variable not defined"
([issue #1](https://github.com/otherworld-dev/rsi-pi/issues/1)) — the
variables appear tied to a technology-package installation, and the exact
availability conditions haven't been pinned down yet. **If only the
`$TECH.…` lines error, run `RSIPI_Minimal.src` instead** — everything
except the Tech handshake works without them.

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
| "Variable not defined" only on `$TECH.…` lines | `$TECH` technology parameters not available on this installation | Use `RSIPI_Minimal.src` (see §4) |
| "Address already in use" when creating the RSI Ethernet entry | An interface entry with that address/subnet already exists | Edit or delete the existing entry instead of adding a new one; reboot (see §2) |
| `RSI_CREATE` returns `RSIFILENOTFOUND` / `RSIINVFILE` | Config files missing from `Config\User\Common\SensorInterface` (check for the extra-`\KRC\` path mistake), renamed, or `.rsi`/`.rsi.xml`/`.diagram` set incomplete | Re-copy all four config files together, keep original names (manual §7.1.2, p. 40 for return codes) |
| `RSI_ON` fails / RSI communication error immediately | IP in `RSI_EthernetConfig_Full.xml` doesn't match the PC, or subnet mismatch (only last octet may differ), or PC in `192.168.0.x` | Fix `<IP_NUMBER>`; follow the IP rules in §2 |
| Robot program runs but Python sees no packets | PC firewall blocking the UDP port, or RSIPI bound to the wrong interface | Allow the port / disable firewall on the RSI interface; verify with KUKA's `TestServer.exe` (see §3) |
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
