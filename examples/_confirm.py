"""Ask before the robot does anything.

These examples get run standing next to a robot, so every step that commands
motion or switches an output waits for an explicit "yes" first. Answer
anything else and the script skips that step and carries on - nothing is
half-done.

    from _confirm import confirm

    if confirm("Move the TCP 50 mm in +X", "~25 mm/s, returns afterwards"):
        api.motion.move_cartesian_trajectory({"X": 50}, steps=50)

Set RSIPI_ASSUME_YES=1 to answer everything automatically. That exists for
examples/dry_run.py against the emulated controller. Never set it with a
robot attached.
"""
import os

__all__ = ["confirm", "assume_yes"]


def assume_yes() -> bool:
    """True when prompts are being answered automatically."""
    return os.environ.get("RSIPI_ASSUME_YES") == "1"


def confirm(action: str, detail: str = "") -> bool:
    """Ask before doing something to the robot. Returns True to proceed.

    Args:
        action: what is about to happen, in plain words
        detail: speed, distance, which output - whatever the operator needs
            to decide whether it is safe right now
    """
    if assume_yes():
        print(f"  [auto-yes] {action}")
        return True

    print(f"\n>> {action}")
    if detail:
        print(f"   {detail}")
    try:
        answer = input("   Proceed? [yes/NO]: ").strip().lower()
    except EOFError:          # piped input that ran out - do NOT move
        print("   no input - skipping")
        return False
    if answer == "yes":
        return True
    print("   skipped")
    return False
