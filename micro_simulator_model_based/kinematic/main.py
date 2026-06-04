from __future__ import annotations

from .simulation import AlphaBotSimulation
from .visualizer import SimulationViewer
from .world_setup import run_world_setup

DT_S = 0.05


def main() -> None:
    setup_seed = None
    simulation = None

    while True:
        if simulation is None:
            setup = run_world_setup(initial=setup_seed)
            setup_seed = None
            if setup is None:
                return
            simulation = AlphaBotSimulation.from_setup(setup)

        if not SimulationViewer(simulation, dt_s=DT_S).run():
            return

        setup_seed = simulation.to_setup()
        simulation = None


if __name__ == "__main__":
    main()
