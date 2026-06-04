#!/usr/bin/env python3
"""Run from this folder: python run.py"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from micro_simulator_real.main import main

if __name__ == "__main__":
    main()
