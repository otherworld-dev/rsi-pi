"""``RSIPI.__version__`` must be the version pyproject.toml publishes.

The two drifted once (the package said 2.0.0 while PyPI got 0.1.1), so the
release build now fails if they disagree.
"""

import re
from pathlib import Path

import RSIPI

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def test_dunder_version_matches_pyproject():
    text = PYPROJECT.read_text(encoding="utf-8")
    declared = re.search(r'^version\s*=\s*"([^"]+)"', text, re.MULTILINE).group(1)
    assert RSIPI.__version__ == declared
