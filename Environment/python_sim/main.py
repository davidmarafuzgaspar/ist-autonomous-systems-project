"""AlphaBot2 pure-Python simulator on the intersection board."""

from __future__ import annotations

import argparse
import pathlib
import sys

DT_S = 0.05
HEADLESS_STEPS = 20

if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from python_sim.simulation import AlphaBotSimulation
    from python_sim.visualizer import SimulationViewer
else:
    from .simulation import AlphaBotSimulation
    from .visualizer import SimulationViewer


def _run_headless(steps: int) -> None:
    sim = AlphaBotSimulation.create_default()
    sim.set_command(linear_m_s=0.12, angular_rad_s=0.0)
    for step in range(steps):
        snap = sim.step(DT_S)
        loc = "--"
        if snap.localized_cell is not None:
            c = snap.localized_cell
            loc = f"cell({c.cell_row},{c.cell_col}) marker={c.marker_id}"
        print(f"step={step:03d} line={snap.line_binary} obstacles={list(snap.obstacle_binary)} localized={loc}")


def main() -> None:
    parser = argparse.ArgumentParser(description="AlphaBot2 simulator (WASD in GUI)")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="print sensor lines to the terminal (no GUI)",
    )
    args = parser.parse_args()

    if args.headless:
        _run_headless(HEADLESS_STEPS)
        return

    SimulationViewer(AlphaBotSimulation.create_default(), dt_s=DT_S).run()


if __name__ == "__main__":
    main()
