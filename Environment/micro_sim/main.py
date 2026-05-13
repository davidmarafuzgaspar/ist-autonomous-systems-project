"""Entry point: solve the intersection MDP via Bellman value iteration."""

from __future__ import annotations

import argparse
import pathlib
import sys


if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from micro_sim.display import print_layout, print_policy, print_values
    from micro_sim.value_iteration import ValueIteration
    from micro_sim.viewer import PolicyViewer
    from micro_sim.world import Action, GridCell, IntersectionWorld
else:
    from .display import print_layout, print_policy, print_values
    from .value_iteration import ValueIteration
    from .viewer import PolicyViewer
    from .world import Action, GridCell, IntersectionWorld


def _print_header(title: str) -> None:
    bar = "=" * len(title)
    print(f"\n{bar}\n{title}\n{bar}")


def _print_formula(gamma: float, theta: float) -> None:
    print(
        "Bellman optimality (deterministic transitions):\n"
        "    V*(s) = max_a [ R(s, a, s') + gamma * V*(s') ]\n"
        "    pi*(s) = argmax_a [ R(s, a, s') + gamma * V*(s') ]\n"
        f"Discount gamma = {gamma}\n"
        f"Stopping threshold theta = {theta}\n"
    )


def _roll_out_policy(
    world: IntersectionWorld,
    policy: dict[GridCell, Action | None],
    max_steps: int,
) -> None:
    state = world.start
    total_reward = 0.0
    for step in range(1, max_steps + 1):
        action = policy.get(state)
        if action is None:
            print(f"step {step:02d}  state={state}  policy=GOAL  total_reward={total_reward:.1f}")
            return
        next_state, hit_wall = world.next_state(state, action)
        reward = world.reward(state, next_state, hit_wall)
        total_reward += reward
        print(
            f"step {step:02d}  state=({state.row},{state.col})"
            f"  action={action.value:<5s}"
            f"  next=({next_state.row},{next_state.col})"
            f"  hit_wall={hit_wall}"
            f"  reward={reward:>8.1f}"
            f"  total={total_reward:>9.1f}"
        )
        state = next_state
        if world.is_terminal(state):
            print(f"reached goal at step {step}  total_reward={total_reward:.1f}")
            return
    print(f"did not reach goal in {max_steps} steps  total_reward={total_reward:.1f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="MDP value iteration on the intersection grid")
    parser.add_argument("--gamma", type=float, default=0.85, help="discount factor")
    parser.add_argument("--theta", type=float, default=1e-3, help="convergence threshold")
    parser.add_argument("--max-iterations", type=int, default=1000, help="cap on iterations")
    parser.add_argument("--rollout-steps", type=int, default=40, help="steps to follow the policy")
    parser.add_argument("--verbose", action="store_true", help="print delta at every iteration")
    parser.add_argument("--headless", action="store_true", help="skip the Tkinter viewer")
    parser.add_argument("--show-values", action="store_true", help="overlay V*(s) next to each arrow")
    args = parser.parse_args()

    world = IntersectionWorld()

    _print_header("WORLD LAYOUT (SS=start, GG=goal, ##=obstacle)")
    print_layout(world)

    _print_header("MDP DEFINITION AND BELLMAN EQUATION")
    _print_formula(args.gamma, args.theta)

    _print_header("RUNNING VALUE ITERATION")
    solver = ValueIteration(
        world=world,
        gamma=args.gamma,
        theta=args.theta,
        max_iterations=args.max_iterations,
        verbose=args.verbose,
    )
    result = solver.solve()
    status = "converged" if result.converged else "stopped (max iterations)"
    print(f"{status} after {result.iterations} iterations (final delta = {result.final_delta:.6f})")

    _print_header("OPTIMAL VALUES V*(s)")
    print_values(world, result.values)

    _print_header("OPTIMAL POLICY pi*(s)")
    print_policy(world, result.policy)

    _print_header("ROLLOUT FROM START FOLLOWING pi*")
    _roll_out_policy(world, result.policy, args.rollout_steps)

    if args.headless:
        return

    viewer = PolicyViewer(
        world=world,
        policy=result.policy,
        values=result.values,
        show_values=args.show_values,
    )
    viewer.run()


if __name__ == "__main__":
    main()
