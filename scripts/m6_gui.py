#!/usr/bin/env python3
"""Launch the repository-local M6 GUI without importing pyserial directly."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = ROOT / "python" / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from m6gui.app import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
