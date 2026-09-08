"""Locate the RSI context bundles that ship with RSIPI.

A "context" is one unit of four files that must always travel together:

    RSIPI_<Name>.rsi          RSIVisual's signal-flow file
    RSIPI_<Name>.rsi.xml      the file the controller actually reads
    RSIPI_<Name>.rsi.diagram  RSIVisual's canvas layout (cosmetic)
    RSI_EthernetConfig_<Name>.xml   the telegram structure, named by the .rsi

BOTH ENDS MUST LOAD THE SAME PAIR. Passing ``context("joints")`` on the PC
only works if the controller has that context's files in
``C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\``. A config that
disagrees with the controller's means the two ends describe different
telegrams, which is not a clean failure - see controller/README.md.

    >>> from RSIPI import RSIAPI, context
    >>> api = RSIAPI(context("joints"))

There is deliberately no "everything wired up" context. The maximal one
(``full``) is the *least* portable: its AXISCORREXT and external-axis
monitor channels cannot bind on a robot without external axes, and the
ETHERNET object reports RSIBad at RSI_ON. ``joints`` is the most capable
context that works on any 6-axis robot, which is why it is the default.
"""
from pathlib import Path
from typing import Dict, List

from .exceptions import RSIConfigError

CONTEXT_DIR = Path(__file__).resolve().parent / "contexts"

DEFAULT_CONTEXT = "joints"

#: name -> (context stem, one-line description)
CONTEXTS: Dict[str, tuple] = {
    "basic": ("RSIPI_Basic",
              "Cartesian corrections, digital I/O, $SEN_PREA, Tech. "
              "KUKA's own example, structurally unchanged. Hardware-verified."),
    "joints": ("RSIPI_Joints",
               "basic + joint corrections (AXISCORR), joint feedback and a "
               "digital-output read-back. Hardware-verified. The default."),
    "full": ("RSIPI_Full",
             "joints + external axes (AXISCORREXT), correction monitors and "
             "motor currents. External-axis cells ONLY - reports RSIBad on a "
             "robot without them. Not hardware-verified."),
    "onlysend": ("RSIPI_OnlySend",
                 "basic with ONLYSEND=TRUE: the robot streams and the PC never "
                 "replies, so no corrections are possible. Hardware-verified."),
    "stop": ("RSIPI_Stop",
             "basic + a STOP object, to end RSI_MOVECORR() from the PC. "
             "Not hardware-verified."),
}


def available_contexts() -> List[str]:
    """Names accepted by :func:`context`, in rough order of capability."""
    return list(CONTEXTS)


def describe_contexts() -> str:
    """Human-readable listing of the shipped contexts."""
    width = max(len(n) for n in CONTEXTS)
    lines = []
    for name, (stem, desc) in CONTEXTS.items():
        marker = " (default)" if name == DEFAULT_CONTEXT else ""
        lines.append(f"  {name:<{width}}  {stem}{marker}\n"
                     f"  {'':<{width}}  {desc}")
    return "\n".join(lines)


def context(name: str = DEFAULT_CONTEXT) -> str:
    """Path to a shipped context's Ethernet config, for passing to RSIAPI.

    Args:
        name: one of :func:`available_contexts` (default ``"joints"``).

    Returns:
        Absolute path to ``RSI_EthernetConfig_<Name>.xml``.

    Raises:
        RSIConfigError: if the name is unknown, or the packaged file is
            missing (an installation that dropped its data files).

    Example:
        >>> from RSIPI import RSIAPI, context
        >>> api = RSIAPI(context())            # joints
        >>> api = RSIAPI(context("basic"))
    """
    key = str(name).strip().lower()
    if key not in CONTEXTS:
        raise RSIConfigError(
            f"Unknown context {name!r}. Available:\n{describe_contexts()}"
        )

    stem = CONTEXTS[key][0]
    # stem is 'RSIPI_<Suffix>'; split rather than removeprefix, which is 3.9+.
    suffix = stem.split("_", 1)[1]
    path = CONTEXT_DIR / f"RSI_EthernetConfig_{suffix}.xml"
    if not path.is_file():
        raise RSIConfigError(
            f"Context {key!r} resolves to {path}, which does not exist. The "
            f"packaged context files are missing from this installation."
        )
    return str(path)


def context_files(name: str = DEFAULT_CONTEXT) -> List[Path]:
    """The four files of a context - copy ALL of them to the controller.

    A .rsi copied without its matching config produces
    ``RSI_CREATE: Invalid index - signal output`` on the controller.
    """
    key = str(name).strip().lower()
    if key not in CONTEXTS:
        raise RSIConfigError(
            f"Unknown context {name!r}. Available:\n{describe_contexts()}"
        )
    stem = CONTEXTS[key][0]
    suffix = stem.split("_", 1)[1]
    files = [
        CONTEXT_DIR / f"{stem}.rsi",
        CONTEXT_DIR / f"{stem}.rsi.xml",
        CONTEXT_DIR / f"{stem}.rsi.diagram",
        CONTEXT_DIR / f"RSI_EthernetConfig_{suffix}.xml",
    ]
    missing = [str(f) for f in files if not f.is_file()]
    if missing:
        raise RSIConfigError(f"Context {key!r} is incomplete; missing: {missing}")
    return files
