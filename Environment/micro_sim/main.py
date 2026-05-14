"""Entry point: solve the intersection MDP via Bellman value iteration."""

from __future__ import annotations

import argparse
import pathlib
import sys


if __package__ in (None, ""):
    sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))
    from micro_sim.analysis import SWEEP_PRESETS, print_sweep_table, run_sweep
    from micro_sim.display import print_layout, print_policy, print_values
    from micro_sim.iteration_viewer import InteractiveValueIterationViewer
    from micro_sim.value_iteration import ValueIteration
    from micro_sim.viewer import PolicyViewer
    from micro_sim.world import Action, GridCell, IntersectionWorld, MdpAction, MdpState, PoseState
else:
    from .analysis import SWEEP_PRESETS, print_sweep_table, run_sweep
    from .display import print_layout, print_policy, print_values
    from .iteration_viewer import InteractiveValueIterationViewer
    from .value_iteration import ValueIteration
    from .viewer import PolicyViewer
    from .world import Action, GridCell, IntersectionWorld, MdpAction, MdpState, PoseState


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
    policy: dict[MdpState, MdpAction | None],
    max_steps: int,
) -> None:
    state = world.initial_mdp_state()
    total_reward = 0.0
    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            print(f"step {step:02d}  already at goal  total_reward={total_reward:.1f}")
            return
        action = policy.get(state)
        if action is None:
            print(f"step {step:02d}  state={state}  policy=GOAL  total_reward={total_reward:.1f}")
            return
        next_state, hit_wall = world.transition(state, action)
        reward = world.reward_transition(state, action, next_state, hit_wall)
        total_reward += reward
        if world.oriented_mdp:
            assert isinstance(state, PoseState)
            assert isinstance(next_state, PoseState)
            print(
                f"step {step:02d}  cell=({state.cell.row},{state.cell.col})"
                f"  h={state.heading.name}"
                f"  action={action.value}"
                f"  next_cell=({next_state.cell.row},{next_state.cell.col})"
                f"  next_h={next_state.heading.name}"
                f"  hit_wall={hit_wall}"
                f"  reward={reward:>8.1f}"
                f"  total={total_reward:>9.1f}"
            )
        else:
            assert isinstance(state, GridCell) and isinstance(next_state, GridCell)
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


def _build_world(args: argparse.Namespace) -> IntersectionWorld:
    world = IntersectionWorld()
    if getattr(args, "oriented", False):
        world.oriented_mdp = True
    if args.goal_reward is not None:
        world.goal_reward = args.goal_reward
    if args.collision_penalty is not None:
        world.collision_penalty = args.collision_penalty
    if args.away_penalty is not None:
        world.away_from_goal_penalty = args.away_penalty
    if args.step_cost is not None:
        world.step_cost = args.step_cost
    return world


def main() -> None:
    parser = argparse.ArgumentParser(description="MDP value iteration on the intersection grid")
    parser.add_argument("--gamma", type=float, default=0.85, help="discount factor")
    parser.add_argument("--theta", type=float, default=1e-3, help="convergence threshold")
    parser.add_argument("--max-iterations", type=int, default=1000, help="cap on iterations")
    parser.add_argument("--rollout-steps", type=int, default=40, help="steps to follow the policy")
    parser.add_argument("--verbose", action="store_true", help="print delta at every iteration")
    parser.add_argument("--headless", action="store_true", help="skip the Tkinter viewer")
    parser.add_argument("--show-values", action="store_true", help="overlay V*(s) next to each arrow")

    parser.add_argument("--goal-reward", type=float, default=None, help="override goal reward")
    parser.add_argument("--collision-penalty", type=float, default=None, help="override collision penalty")
    parser.add_argument("--away-penalty", type=float, default=None, help="override 'moved away from goal' penalty")
    parser.add_argument("--step-cost", type=float, default=None, help="override per-step cost")

    parser.add_argument(
        "--sweep",
        choices=sorted(SWEEP_PRESETS.keys()),
        default=None,
        help="run a sensitivity sweep over one parameter and print a table (skips the viewer)",
    )
    parser.add_argument(
        "--trace",
        action="store_true",
        help="print V(s) after every iteration so you can see the goal value propagating",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="launch the interactive Tkinter viewer with a 'Next iteration' button",
    )
    parser.add_argument(
        "--oriented",
        action="store_true",
        help="use oriented MDP (cell + heading); actions are F/L/R (set turn 90° reward to 0 to disable)",
    )
    parser.add_argument(
        "--jacobi",
        action="store_true",
        help="use synchronous Jacobi updates (each iter k+1 reads only V_k); "
        "default is Gauss-Seidel (in-place, faster but iter 1 already shows propagated values)",
    )

    args = parser.parse_args()

    if args.interactive:
        world = _build_world(args)
        world.oriented_mdp = True
        _print_header("WORLD LAYOUT (SS=start, GG=goal, ##=obstacle)")
        print_layout(world)
        _print_header("MDP DEFINITION AND BELLMAN EQUATION")
        _print_formula(args.gamma, args.theta)
        print(
            f"Rewards: goal={world.goal_reward}, collision={world.collision_penalty}, "
            f"away_from_goal={world.away_from_goal_penalty}, step_cost={world.step_cost}\n"
            f"Oriented MDP: start={world.start} heading={world.start_heading.name}, "
            f"turn_90={world.turn_90_reward}\n"
        )
        viewer = InteractiveValueIterationViewer(
            world=world,
            gamma=args.gamma,
            theta=args.theta,
            max_iterations=args.max_iterations,
            synchronous=args.jacobi,
        )
        viewer.run()
        return

    if args.sweep is not None:
        _print_header(f"SWEEP: {args.sweep}")
        rows = run_sweep(
            sweep_name=args.sweep,
            base_world_factory=lambda: _build_world(args),
            gamma=args.gamma,
            theta=args.theta,
            max_iterations=args.max_iterations,
            rollout_steps=args.rollout_steps,
        )
        print_sweep_table(args.sweep, rows)
        print(
            "\ndiff_cells = number of states where pi*(s) differs from the baseline policy\n"
            "(baseline uses the current --goal-reward/--collision-penalty/--away-penalty/--step-cost/--gamma)"
        )
        return

    world = _build_world(args)

    _print_header("WORLD LAYOUT (SS=start, GG=goal, ##=obstacle)")
    print_layout(world)

    _print_header("MDP DEFINITION AND BELLMAN EQUATION")
    _print_formula(args.gamma, args.theta)
    print(
        f"Rewards: goal={world.goal_reward}, collision={world.collision_penalty}, "
        f"away_from_goal={world.away_from_goal_penalty}, step_cost={world.step_cost}\n"
        + (
            f"Oriented MDP: start_heading={world.start_heading.name}, "
            f"turn_90={world.turn_90_reward}\n"
            if world.oriented_mdp
            else ""
        )
    )

    _print_header("RUNNING VALUE ITERATION")
    print(f"mode = {'Jacobi (synchronous)' if args.jacobi else 'Gauss-Seidel (in-place)'}\n")
    solver = ValueIteration(
        world=world,
        gamma=args.gamma,
        theta=args.theta,
        max_iterations=args.max_iterations,
        verbose=args.verbose,
        synchronous=args.jacobi,
    )

    on_iteration = None
    if args.trace:
        def on_iteration(iteration: int, delta: float, snapshot: dict) -> None:
            if iteration == 0:
                print("  iter   0  (initial V_0 = 0 in every cell)")
            else:
                print(f"  iter {iteration:3d}  delta = {delta:.4f}")
            table = world.aggregate_max_v_per_cell(snapshot) if world.oriented_mdp else snapshot
            print_values(world, table)

    result = solver.solve(on_iteration=on_iteration)
    status = "converged" if result.converged else "stopped (max iterations)"
    print(f"{status} after {result.iterations} iterations (final delta = {result.final_delta:.6f})")

    _print_header("OPTIMAL VALUES V*(s)")
    v_table = world.aggregate_max_v_per_cell(result.values) if world.oriented_mdp else result.values
    print_values(world, v_table)

    _print_header("OPTIMAL POLICY pi*(s)")
    if world.oriented_mdp:
        pol_table = world.representative_policy_per_cell(result.values, result.policy)
        print_policy(world, pol_table)
    else:
        print_policy(world, result.policy)  # type: ignore[arg-type]

    _print_header("ROLLOUT FROM START FOLLOWING pi*")
    _roll_out_policy(world, result.policy, args.rollout_steps)

    if args.headless:
        return

    viewer = PolicyViewer(
        world=world,
        policy=world.representative_policy_per_cell(result.values, result.policy)
        if world.oriented_mdp
        else result.policy,  # type: ignore[arg-type]
        values=v_table,
        show_values=args.show_values,
        representative_heading=world.representative_heading_per_cell(result.values)
        if world.oriented_mdp
        else None,
    )
    viewer.run()


if __name__ == "__main__":
    main()
