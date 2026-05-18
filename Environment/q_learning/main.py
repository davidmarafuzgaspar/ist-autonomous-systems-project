"""Q-learning adaptation: optimal path on O0, then obstacles move (O1), learn from collisions."""

from __future__ import annotations

import argparse
import pathlib
import sys

GAMMA = 0.85

if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from q_learning.experiment import run_full_experiment, run_phase_a
    from q_learning.viewer import QLearningViewer
else:
    from .experiment import run_full_experiment, run_phase_a
    from .viewer import QLearningViewer


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Fase A: VI em O0 -> pi0* e caminho otimo. "
            "Fase B: obstaculos mudam; segue pi0*, Q-learning apos colisoes. "
            "Default: viewer interativo."
        ),
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="experimento no terminal (sem janela)",
    )
    parser.add_argument("--phase-a-only", action="store_true", help="so MDP / caminho otimo em O0")
    parser.add_argument("--episodes", type=int, default=40, help="episodios Q na fase B")
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--gamma", type=float, default=GAMMA)
    parser.add_argument("-q", "--quiet", action="store_true")
    args = parser.parse_args()

    verbose = not args.quiet

    if not args.headless and not args.phase_a_only:
        print("Viewer: Fase A = caminho otimo. Depois 'Mudar mapa' = Fase B + Q-learning.\n")
        QLearningViewer().run()
        return

    if args.phase_a_only:
        run_phase_a(gamma=args.gamma, max_steps=args.max_steps, verbose=verbose)
        return

    run_full_experiment(
        gamma=args.gamma,
        episodes=args.episodes,
        max_steps=args.max_steps,
        verbose=verbose,
    )


if __name__ == "__main__":
    main()
