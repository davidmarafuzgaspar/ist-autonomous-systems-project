#!/usr/bin/env python3
"""Run from this folder: python run.py"""

from __future__ import annotations

import sys
from pathlib import Path

_MICRO_SIMS = Path(__file__).resolve().parent.parent
_CONTROLLER = _MICRO_SIMS.parent / "Controller"
for _path in (_MICRO_SIMS, _CONTROLLER):
    _s = str(_path)
    if _s not in sys.path:
        sys.path.insert(0, _s)

from micro_simulator_dynamic.main import main

if __name__ == "__main__":
    main()
