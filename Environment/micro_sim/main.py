"""Entry point: oriented MDP value iteration on the intersection grid."""

from __future__ import annotations

import argparse
import pathlib
import sys

GAMMA = 0.85
THETA = 1e-3
MAX_ITERATIONS = 1000
ROLLOUT_STEPS = 40

if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from micro_sim.display import oriented_policy_glyph, print_layout, print_policy, print_values
    from micro_sim.iteration_viewer import InteractiveValueIterationViewer
    from micro_sim.value_iteration import ValueIteration
    from micro_sim.world import IntersectionWorld, OrientedAction, PoseState
else:
    from .display import oriented_policy_glyph, print_layout, print_policy, print_values
    from .iteration_viewer import InteractiveValueIterationViewer
    from .value_iteration import ValueIteration
    from .world import IntersectionWorld, OrientedAction, PoseState


def _print_header(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{bar}\n{title}\n{bar}")


def _roll_out_policy(
    world: IntersectionWorld,
    policy: dict[PoseState, OrientedAction | None],
    max_steps: int,
) -> None:
    state = world.initial_state()
    total_reward = 0.0
    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            print(f"step {step:02d}  already at goal  total_reward={total_reward:.1f}")
            return
        action = policy.get(state)
        if action is None:
            print(f"step {step:02d}  at goal  total_reward={total_reward:.1f}")
            return
        next_state, hit_wall = world.transition(state, action)
        reward = world.reward_transition(state, action, next_state, hit_wall)
        total_reward += reward
        glyph = oriented_policy_glyph(action, state.heading)
        print(
            f"step {step:02d}  cell=({state.cell.row},{state.cell.col})"
            f"  h={state.heading.name}  action={glyph}"
            f"  -> ({next_state.cell.row},{next_state.cell.col}) h={next_state.heading.name}"
            f"  reward={reward:>8.1f}  total={total_reward:>9.1f}"
        )
        state = next_state
        if world.is_terminal(state):
            print(f"reached goal at step {step}  total_reward={total_reward:.1f}")
            return
    print(f"did not reach goal in {max_steps} steps  total_reward={total_reward:.1f}")


def _run_interactive() -> None:
    InteractiveValueIterationViewer(
        world=IntersectionWorld(),
        gamma=GAMMA,
        theta=THETA,
        max_iterations=MAX_ITERATIONS,
    ).run()


def _run_final() -> None:
    world = IntersectionWorld()

    _print_header("WORLD LAYOUT")
    print_layout(world)

    _print_header("VALUE ITERATION")
    result = ValueIteration(world, gamma=GAMMA, theta=THETA, max_iterations=MAX_ITERATIONS).solve()
    status = "converged" if result.converged else "stopped (max iterations)"
    print(f"{status} after {result.iterations} iterations (delta = {result.final_delta:.6f})")

    _print_header("V* per cell [max_h V]")
    print_values(world, world.aggregate_max_v_per_cell(result.values))

    _print_header("Policy per cell [aggregated]")
    pol = world.aggregated_policy_per_cell(result.values, GAMMA)
    print_policy(world, pol, world.display_heading_map_for_cell_policy(pol, result.values, GAMMA))

    _print_header("Rollout [pi*(cell,h)]")
    _roll_out_policy(world, result.policy, ROLLOUT_STEPS)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Intersection MDP (F/L/R). Default: interactive viewer.",
    )
    parser.add_argument(
        "--final",
        action="store_true",
        help="print converged V*, policy and rollout (terminal only)",
    )
    args = parser.parse_args()
    if args.final:
        _run_final()
    else:
        _run_interactive()


if __name__ == "__main__":
    main()
