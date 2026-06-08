#!/usr/bin/env python3
"""Run from this folder: python run.py"""

from __future__ import annotations

import sys
from pathlib import Path

_MICRO_SIMS = Path(__file__).resolve().parent.parent
if str(_MICRO_SIMS) not in sys.path:
    sys.path.insert(0, str(_MICRO_SIMS))

from micro_simulator_dynamic.main import main

if __name__ == "__main__":
    main()
