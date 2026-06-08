import sys
from pathlib import Path

_MICRO_SIMS = Path(__file__).resolve().parent.parent
if str(_MICRO_SIMS) not in sys.path:
    sys.path.insert(0, str(_MICRO_SIMS))

from micro_simulator_dynamic.main import main

main()
