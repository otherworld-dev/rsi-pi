"""Copy the files a controller needs out of the installed package.

The RSI contexts ship inside RSIPI so that ``pip install`` provides a config
to run against, which means they live somewhere under site-packages. Nobody
should have to go digging in there to put four files on a robot.

    python -m RSIPI.deploy --context joints --out C:\\deploy

Then copy the contents of that folder to
``C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\`` on the
controller, as the Expert user group.

All four files of a context are copied together, always. A .rsi that reaches
the controller without its matching config produces
``RSI_CREATE: Invalid index - signal output``.
"""
import argparse
import shutil
import sys
from pathlib import Path

from .context import CONTEXTS, DEFAULT_CONTEXT, context_files, describe_contexts


def deploy(name: str = DEFAULT_CONTEXT, out_dir: str = "rsi_deploy",
           overwrite: bool = False) -> list:
    """Copy one context's four files into *out_dir*.

    Args:
        name: context name (see :func:`RSIPI.available_contexts`)
        out_dir: destination directory, created if needed
        overwrite: replace files that are already there

    Returns:
        The list of destination paths written.

    Raises:
        RSIConfigError: unknown context name
        FileExistsError: a destination file exists and overwrite is False
    """
    destination = Path(out_dir)
    destination.mkdir(parents=True, exist_ok=True)

    written = []
    for source in context_files(name):
        target = destination / source.name
        if target.exists() and not overwrite:
            raise FileExistsError(
                f"{target} already exists - pass --overwrite to replace it")
        shutil.copyfile(source, target)
        written.append(target)
    return written


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m RSIPI.deploy",
        description="Copy an RSI context's files out of the package, ready to "
                    "put on a KUKA controller.",
        epilog="Contexts:\n" + describe_contexts(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--context", default=DEFAULT_CONTEXT,
                        choices=sorted(CONTEXTS),
                        help=f"which context (default: {DEFAULT_CONTEXT})")
    parser.add_argument("--out", default="rsi_deploy",
                        help="destination folder (default: ./rsi_deploy)")
    parser.add_argument("--overwrite", action="store_true",
                        help="replace files already in the destination")
    parser.add_argument("--list", action="store_true",
                        help="list the available contexts and exit")
    args = parser.parse_args(argv)

    if args.list:
        print(describe_contexts())
        return 0

    try:
        written = deploy(args.context, args.out, args.overwrite)
    except (FileExistsError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    print(f"Copied {len(written)} files for context '{args.context}':")
    for path in written:
        print(f"  {path}")
    print()
    print("Copy ALL of them (as the Expert user group) to:")
    print(r"  C:\KRC\ROBOTER\Config\User\Common\SensorInterface\ ")
    print()
    print("The KRL programs are separate - they are in controller/Program/ in "
          "the repo and go to:")
    print(r"  C:\KRC\ROBOTER\KRC\R1\Program\ ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
