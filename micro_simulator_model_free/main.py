from __future__ import annotations

from .gui import QLearningViewer, run_map_setup
from .model import IntersectionWorld


def main() -> None:
    setup_seed: IntersectionWorld | None = None
    world: IntersectionWorld | None = None

    while True:
        if world is None:
            world = run_map_setup(initial_world=setup_seed)
            setup_seed = None
            if world is None:
                return

        if not QLearningViewer(world).run():
            return

        setup_seed = world
        world = None


if __name__ == "__main__":
    main()
