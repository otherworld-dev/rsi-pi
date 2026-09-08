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

Ask Python for the exact paths rather than typing them:

```python
>>> from RSIPI import context_files
>>> for f in context_files("joints"):
...     print(f)
```

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
