# The RSI context files moved

They are now in **[`src/RSIPI/contexts/`](../../src/RSIPI/contexts/)**.

## Why

They are data the library needs, not just deployment artefacts. Kept here,
they were not part of the installed package — `pip install RSIPI` gave you 29
`.py` files and no config at all, so an installed copy had nothing to run
against. Inside the package they ship with it, and the PC and the controller
load the very same files.

## What still goes on the controller

Exactly the same as before — copy **all four** files of one context to
`C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`:

```
RSIPI_<Name>.rsi
RSIPI_<Name>.rsi.xml
RSIPI_<Name>.rsi.diagram
RSI_EthernetConfig_<Name>.xml
```

A `.rsi` copied without its matching config gives
`RSI_CREATE: Invalid index - signal output` on the controller.

Get them out of the package with one command, rather than hunting through
`site-packages`:

```
python -m RSIPI.deploy --context joints --out C:\deploy
```

That copies all four files into `C:\deploy` ready to hand to the controller.
`--list` shows the available contexts. Or ask for the paths directly:

```python
>>> from RSIPI import context_files
>>> for f in context_files("joints"):
...     print(f)
```

## Built your own context in RSIVisual?

You do not have to hand-write the matching `RSI_EthernetConfig` — writing it
by hand is what produces `RSI_CREATE: Invalid index - signal output`. The
context already records which object feeds every ETHERNET channel, so RSIPI
can derive the config from it:

```
python -m RSIPI.config_builder MyContext.rsi.xml --ip 10.10.10.10 --port 64000 --out RSI_EthernetConfig_Mine.xml
```

Options: `--sentype`, `--onlysend`, and `--internal DEF_MACur` for extra
`INTERNAL` declarations. Tag names follow the conventions RSIPI's API expects
(`RKorr`, `AKorr`, `DiO`, `SenP1-3`…), so the named methods keep working.

The generator is checked against all six shipped configs — it reproduces every
one of them channel for channel, and those were hand-written and run against a
real controller.

And to open one in RSIVisual:

```
C:\Git\RSI-PI\src\RSIPI\contexts\RSIPI_Joints.rsi
```

## Which context

See [../README.md](../README.md) for the comparison table, or:

```python
>>> from RSIPI import describe_contexts
>>> print(describe_contexts())
```

`joints` is the default and is hardware-verified. The KRL programs are still
in [`../Program/`](../Program/) — those did not move.
