"""Map setup → interactive value iteration with forward slip."""

from __future__ import annotations

import argparse

from .grid_setup import run_map_setup
from .iteration_viewer import InteractiveValueIterationViewer
from .world import GAMMA_DEFAULT, MAX_ITERATIONS_DEFAULT, THETA_DEFAULT, IntersectionWorld


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive value iteration (slip).")
    parser.add_argument("--skip-setup", action="store_true", help="Skip map editor")
    parser.add_argument("--rows", type=int, default=3, help="Grid rows if setup skipped")
    parser.add_argument("--cols", type=int, default=5, help="Grid cols if setup skipped")
    args = parser.parse_args()

    use_setup = not args.skip_setup
    setup_seed: IntersectionWorld | None = None
    world: IntersectionWorld | None = None

    while True:
        if world is None:
            if use_setup:
                world = run_map_setup(
                    initial_rows=args.rows,
                    initial_cols=args.cols,
                    initial_world=setup_seed,
                )
                setup_seed = None
                if world is None:
                    return
            else:
                world = IntersectionWorld.from_size(args.rows, args.cols)

        if not InteractiveValueIterationViewer(
            world=world,
            gamma=GAMMA_DEFAULT,
            theta=THETA_DEFAULT,
            max_iterations=MAX_ITERATIONS_DEFAULT,
        ).run():
            return

        setup_seed = world
        world = None
        use_setup = True


if __name__ == "__main__":
    main()
