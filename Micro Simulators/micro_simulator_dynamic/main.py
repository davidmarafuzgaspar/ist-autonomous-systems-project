from __future__ import annotations

from .gui import RealRuntimeViewer, run_map_setup
from .model import RealRuntimeSim, Scenario


def main() -> None:
    seed: Scenario | None = None
    sim: RealRuntimeSim | None = None

    while True:
        if sim is None:
            scenario = run_map_setup(initial=seed)
            seed = None
            if scenario is None:
                return
            sim = RealRuntimeSim(scenario)

        if not RealRuntimeViewer(sim).run():
            return

        seed = sim.scenario
        sim = None


if __name__ == "__main__":
    main()
