from __future__ import annotations

from .grid_setup import run_map_setup
from .iteration_viewer import InteractiveValueIterationViewer
from .world import GAMMA_DEFAULT, MAX_ITERATIONS_DEFAULT, THETA_DEFAULT, IntersectionWorld


def main() -> None:
    setup_seed: IntersectionWorld | None = None
    world: IntersectionWorld | None = None

    while True:
        if world is None:
            world = run_map_setup(initial_world=setup_seed)
            setup_seed = None
            if world is None:
                return

        if not InteractiveValueIterationViewer(
            world=world,
            gamma=GAMMA_DEFAULT,
            theta=THETA_DEFAULT,
            max_iterations=MAX_ITERATIONS_DEFAULT,
        ).run():
            return

        setup_seed = world
        world = None


if __name__ == "__main__":
    main()
