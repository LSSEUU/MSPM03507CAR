#!/usr/bin/env python3
"""Compatibility entry point for :mod:`m6link`.

Existing imports and command arguments remain available; all serial ownership
is delegated to ``m6link.Session`` through the package CLI/compatibility layer.
"""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "python" / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from m6link.compat import *  # noqa: F401,F403,E402
from m6link.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
