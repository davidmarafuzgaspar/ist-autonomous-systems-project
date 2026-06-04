"""Entry point: same workflow as deterministic VI, plus slip weights and stochastic rollout."""

from __future__ import annotations

import argparse
import pathlib
import random
import sys

from .grid_setup import run_map_setup
from .world import (
    GAMMA_DEFAULT,
    HEADING_DELTA_RC,
    MAX_ITERATIONS_DEFAULT,
    THETA_DEFAULT,
    GridAction,
    IntersectionWorld,
    PoseState,
)

GAMMA = GAMMA_DEFAULT
THETA = THETA_DEFAULT
MAX_ITERATIONS = MAX_ITERATIONS_DEFAULT
ROLLOUT_STEPS = 40

if __package__ in (None, ""):
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
    from value_iteration_non_deterministic.display import (
        policy_glyph,
        print_layout,
        print_policy,
        print_values,
    )
    from value_iteration_non_deterministic.iteration_viewer import InteractiveValueIterationViewer
    from value_iteration_non_deterministic.value_iteration import ValueIteration
else:
    from .display import policy_glyph, print_layout, print_policy, print_values
    from .iteration_viewer import InteractiveValueIterationViewer
    from .value_iteration import ValueIteration


def _print_header(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{bar}\n{title}\n{bar}")


def _print_slip_note(world: IntersectionWorld) -> None:
    p_int, p_left, p_right = world.normalized_slip_probs()
    print(
        f"Forward slip (renormalized): intended={p_int:.0%}  "
        f"left={p_left:.0%}  right={p_right:.0%}"
    )


def _roll_out_policy(
    world: IntersectionWorld,
    policy: dict[PoseState, GridAction | None],
    max_steps: int,
    *,
    stochastic: bool = False,
    seed: int = 0,
) -> None:
    rng = random.Random(seed)
    state = world.initial_state()
    total_reward = 0.0
    mode = "stochastic" if stochastic else "intended-only"
    print(f"Rollout: {mode}")

    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            print(f"step {step:02d}  already at goal  total_reward={total_reward:.1f}")
            return
        action = policy.get(state)
        if action is None:
            print(f"step {step:02d}  at goal  total_reward={total_reward:.1f}")
            return
        if stochastic:
            next_state, reward, done = world.sample_transition(state, action, rng)
        else:
            next_state, reward, done = world.simulate_step(state, action)
        total_reward += reward
        glyph = policy_glyph(action, state.heading, world)
        slip = ""
        if stochastic:
            next_heading = world.movement_heading_for_action(state.heading, action)
            dr, dc = HEADING_DELTA_RC[next_heading]
            moved = (
                next_state.cell.row - state.cell.row,
                next_state.cell.col - state.cell.col,
            )
            if moved != (dr, dc):
                slip = " slip"
        print(
            f"step {step:02d}  cell=({state.cell.row},{state.cell.col})"
            f"  h={state.heading.name}  action={glyph}{slip}"
            f"  -> ({next_state.cell.row},{next_state.cell.col}) h={next_state.heading.name}"
            f"  reward={reward:>8.1f}  total={total_reward:>9.1f}"
        )
        state = next_state
        if done or world.is_terminal(state):
            print(f"reached goal at step {step}  total_reward={total_reward:.1f}")
            return
    print(f"did not reach goal in {max_steps} steps  total_reward={total_reward:.1f}")


def _apply_slip_args(world: IntersectionWorld, args: argparse.Namespace) -> None:
    if args.slip_intended is not None:
        world.slip_prob_intended = args.slip_intended
    if args.slip_left is not None:
        world.slip_prob_left = args.slip_left
    if args.slip_right is not None:
        world.slip_prob_right = args.slip_right


def _run_interactive(
    world: IntersectionWorld | None = None,
    *,
    skip_setup: bool = False,
    setup_rows: int = 5,
    setup_cols: int = 5,
) -> None:
    use_setup = not skip_setup
    setup_seed: IntersectionWorld | None = None
    while True:
        if world is None:
            if use_setup:
                world = run_map_setup(
                    initial_rows=setup_rows,
                    initial_cols=setup_cols,
                    initial_world=setup_seed,
                )
                setup_seed = None
                if world is None:
                    return
            else:
                world = IntersectionWorld.from_size(setup_rows, setup_cols)
        change_world = InteractiveValueIterationViewer(
            world=world,
            gamma=GAMMA,
            theta=THETA,
            max_iterations=MAX_ITERATIONS,
        ).run()
        if not change_world:
            return
        setup_seed = world
        world = None
        use_setup = True


def _run_final_with_world(world: IntersectionWorld, *, seed: int = 0) -> None:
    _print_header("WORLD LAYOUT")
    print_layout(world)
    _print_slip_note(world)

    _print_header("VALUE ITERATION")
    result = ValueIteration(world, gamma=GAMMA, theta=THETA, max_iterations=MAX_ITERATIONS).solve()
    status = "converged" if result.converged else "stopped (max iterations)"
    print(f"{status} after {result.iterations} iterations (delta = {result.final_delta:.6f})")

    _print_header("V* per cell [max_h V]")
    print_values(world, world.aggregate_max_v_per_cell(result.values))

    _print_header("Policy per cell [aggregated]")
    pol = world.aggregated_policy_per_cell(result.values, GAMMA)
    print_policy(world, pol, world.display_heading_map_for_cell_policy(pol, result.values, GAMMA))

    _print_header("Rollout [intended forward]")
    _roll_out_policy(world, result.policy, ROLLOUT_STEPS, stochastic=False)

    _print_header("Rollout [stochastic forward]")
    _roll_out_policy(world, result.policy, ROLLOUT_STEPS, stochastic=True, seed=seed)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Value iteration with forward slip on the intersection MDP.",
    )
    parser.add_argument("--final", action="store_true", help="VI + rollouts in terminal")
    parser.add_argument("--rows", type=int, default=3, help="Grid rows (--skip-setup)")
    parser.add_argument("--cols", type=int, default=5, help="Grid cols (--skip-setup)")
    parser.add_argument("--skip-setup", action="store_true", help="Skip map editor")
    parser.add_argument("--seed", type=int, default=0, help="RNG for stochastic rollout")
    parser.add_argument("--slip-intended", type=float, default=None, help="Intended forward weight")
    parser.add_argument("--slip-left", type=float, default=None, help="Slip-left weight")
    parser.add_argument("--slip-right", type=float, default=None, help="Slip-right weight")
    args = parser.parse_args()

    if args.final:
        if args.skip_setup:
            world = IntersectionWorld.from_size(args.rows, args.cols)
        else:
            world = IntersectionWorld()
        _apply_slip_args(world, args)
        _run_final_with_world(world, seed=args.seed)
    else:
        world = None
        if args.skip_setup:
            world = IntersectionWorld.from_size(args.rows, args.cols)
            _apply_slip_args(world, args)
        _run_interactive(
            world=world,
            skip_setup=args.skip_setup,
            setup_rows=args.rows,
            setup_cols=args.cols,
        )


if __name__ == "__main__":
    main()
