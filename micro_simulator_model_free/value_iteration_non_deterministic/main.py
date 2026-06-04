"""Stochastic MDP: VI expects slip on forward; configurable P(fwd/left/right)."""

from __future__ import annotations

import argparse
import pathlib
import random
import sys

GAMMA = 0.85
THETA = 1e-3
MAX_ITERATIONS = 1000
ROLLOUT_STEPS = 40

_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from mdp_algorithm.display import oriented_policy_glyph, print_layout, print_policy, print_values
from mdp_algorithm.iteration_viewer import InteractiveValueIterationViewer
from mdp_algorithm.value_iteration import ValueIteration
from mdp_algorithm.world import (
    HEADING_DELTA_RC,
    IntersectionWorld,
    OrientedAction,
    PoseState,
)


def _print_header(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{bar}\n{title}\n{bar}")


def _print_bellman_note(world: IntersectionWorld) -> None:
    pf, pl, pr = world.normalized_motion_probs()
    print(
        "Bellman (stochastic forward):\n"
        "  Q(s,F) = sum_{s'} P(s'|s,F) [ R + gamma V(s') ]\n"
        f"  P(intended)={pf:.0%}  P(slip left)={pl:.0%}  P(slip right)={pr:.0%}\n"
        "  L/R turns remain deterministic."
    )


def _roll_out_policy(
    world: IntersectionWorld,
    policy: dict[PoseState, OrientedAction | None],
    max_steps: int,
    *,
    stochastic: bool = False,
    seed: int = 0,
) -> None:
    rng = random.Random(seed)
    state = world.initial_state()
    total_reward = 0.0
    mode = "stochastic rollout" if stochastic else "deterministic (intended F only)"
    print(f"Rollout mode: {mode}")

    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            print(f"step {step:02d}  already at goal  total_reward={total_reward:.1f}")
            return
        action = policy.get(state)
        if action is None:
            print(f"step {step:02d}  at goal  total_reward={total_reward:.1f}")
            return
        if stochastic:
            next_state, hit_wall = world.sample_transition(state, action, rng)
        else:
            next_state, hit_wall = world.transition(state, action)
        reward = world.reward_transition(state, action, next_state, hit_wall)
        total_reward += reward
        glyph = oriented_policy_glyph(action, state.heading)
        slip = ""
        if stochastic and action == OrientedAction.FORWARD:
            d_row = next_state.cell.row - state.cell.row
            d_col = next_state.cell.col - state.cell.col
            if (d_row, d_col) != HEADING_DELTA_RC.get(state.heading, (0, 0)):
                slip = " SLIP"
        wall = " WALL" if hit_wall else ""
        print(
            f"step {step:02d}  cell=({state.cell.row},{state.cell.col})"
            f"  h={state.heading.name}  action={glyph}{slip}"
            f"  -> ({next_state.cell.row},{next_state.cell.col}) h={next_state.heading.name}"
            f"{wall}  reward={reward:>8.1f}  total={total_reward:>9.1f}"
        )
        state = next_state
        if world.is_terminal(state):
            print(f"reached goal at step {step}  total_reward={total_reward:.1f}")
            return
    print(f"did not reach goal in {max_steps} steps  total_reward={total_reward:.1f}")


def _build_world(args: argparse.Namespace) -> IntersectionWorld:
    world = IntersectionWorld()
    if args.p_forward is not None:
        world.motion_prob_forward = args.p_forward
    if args.p_left is not None:
        world.motion_prob_left = args.p_left
    if args.p_right is not None:
        world.motion_prob_right = args.p_right
    return world


def _run_interactive(world: IntersectionWorld, args: argparse.Namespace) -> None:
    InteractiveValueIterationViewer(
        world=world,
        gamma=args.gamma,
        theta=THETA,
        max_iterations=MAX_ITERATIONS,
    ).run()


def _run_final(world: IntersectionWorld, args: argparse.Namespace) -> None:
    _print_header("WORLD LAYOUT")
    print_layout(world)
    _print_bellman_note(world)

    _print_header("VALUE ITERATION")
    result = ValueIteration(world, gamma=args.gamma, theta=THETA, max_iterations=MAX_ITERATIONS).solve()
    status = "converged" if result.converged else "stopped (max iterations)"
    print(f"{status} after {result.iterations} iterations (delta = {result.final_delta:.6f})")

    _print_header("V* per cell [max_h V]")
    print_values(world, world.aggregate_max_v_per_cell(result.values))

    _print_header("Policy per cell [aggregated]")
    pol = world.aggregated_policy_per_cell(result.values, args.gamma)
    print_policy(world, pol, world.display_heading_map_for_cell_policy(pol, result.values, args.gamma))

    _print_header("Rollout pi* (deterministic F)")
    _roll_out_policy(world, result.policy, ROLLOUT_STEPS, stochastic=False)

    _print_header("Rollout pi* (stochastic F)")
    _roll_out_policy(world, result.policy, ROLLOUT_STEPS, stochastic=True, seed=args.seed)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Non-deterministic MDP (forward slip). Default: interactive viewer.",
    )
    parser.add_argument("--final", action="store_true", help="VI + rollouts in terminal")
    parser.add_argument("--gamma", type=float, default=GAMMA)
    parser.add_argument("--seed", type=int, default=0, help="RNG for stochastic rollout")
    parser.add_argument(
        "--p-forward",
        type=float,
        default=None,
        help="weight for intended forward (default 0.7; renormalized with left/right)",
    )
    parser.add_argument("--p-left", type=float, default=None, help="slip perpendicular left")
    parser.add_argument("--p-right", type=float, default=None, help="slip perpendicular right")
    args = parser.parse_args()

    world = _build_world(args)

    if args.final:
        _run_final(world, args)
    else:
        _run_interactive(world, args)


if __name__ == "__main__":
    main()
