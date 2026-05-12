from __future__ import annotations

import argparse
import pathlib
import sys


if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from python_sim.simulation import AlphaBotSimulation
    from python_sim.visualizer import SimulationViewer
else:
    from .simulation import AlphaBotSimulation
    from .visualizer import SimulationViewer


def run_headless(steps: int, dt_s: float) -> None:
    simulation = AlphaBotSimulation.create_default()
    simulation.set_command(linear_m_s=0.12, angular_rad_s=0.0)
    for step in range(steps):
        snapshot = simulation.step(dt_s)
        localized_cell = "--"
        if snapshot.localized_cell is not None:
            localized_cell = (
                f"cell({snapshot.localized_cell.cell_row}, {snapshot.localized_cell.cell_col})"
                f" via marker {snapshot.localized_cell.marker_id}"
            )

        print(
            f"step={step:03d} "
            f"pose=({simulation.robot.pose.x:+.3f}, {simulation.robot.pose.y:+.3f}, {simulation.robot.pose.yaw:+.3f}) "
            f"line={snapshot.line_binary} "
            f"obstacles={list(snapshot.obstacle_binary)} "
            f"markers={[marker.marker_id for marker in snapshot.camera_visible_markers]} "
            f"localized={localized_cell}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Pure Python AlphaBot2 simulator")
    parser.add_argument("--headless", action="store_true", help="run without the Tkinter viewer")
    parser.add_argument("--steps", type=int, default=20, help="number of steps in headless mode")
    parser.add_argument("--dt", type=float, default=0.05, help="simulation step in seconds")
    args = parser.parse_args()

    if args.headless:
        run_headless(steps=args.steps, dt_s=args.dt)
        return

    simulation = AlphaBotSimulation.create_default()
    viewer = SimulationViewer(simulation=simulation, dt_s=args.dt)
    viewer.run()


if __name__ == "__main__":
    main()

