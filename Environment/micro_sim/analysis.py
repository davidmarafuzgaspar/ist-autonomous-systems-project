"""Sensitivity analysis for the intersection MDP.

Sweeps a single MDP parameter and reports, for every value:

- iterations to converge
- ``V*(start)``
- number of rollout steps to reach the goal
- total (undiscounted) reward of the rollout
- number of cells where ``pi*`` differs from the baseline policy

This is meant to make the impact of each reward term and the discount
factor visible at a glance, without changing source code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from .value_iteration import ValueIteration
from .world import Action, GridCell, IntersectionWorld, MdpAction, MdpState


@dataclass
class SweepConfig:
    name: str
    values: list[float]
    apply: Callable[[IntersectionWorld, float], None] = field(repr=False)
    affects_gamma: bool = False


def _set_goal_reward(world: IntersectionWorld, value: float) -> None:
    world.goal_reward = value


def _set_collision_penalty(world: IntersectionWorld, value: float) -> None:
    world.collision_penalty = value


def _set_away_penalty(world: IntersectionWorld, value: float) -> None:
    world.away_from_goal_penalty = value


def _set_step_cost(world: IntersectionWorld, value: float) -> None:
    world.step_cost = value


SWEEP_PRESETS: dict[str, SweepConfig] = {
    "step-cost": SweepConfig(
        name="step_cost",
        values=[-0.1, -1.0, -10.0, -100.0, -500.0, -2000.0],
        apply=_set_step_cost,
    ),
    "away-penalty": SweepConfig(
        name="away_from_goal_penalty",
        values=[-1.0, -10.0, -100.0, -500.0, -2000.0],
        apply=_set_away_penalty,
    ),
    "collision-penalty": SweepConfig(
        name="collision_penalty",
        values=[-10.0, -100.0, -500.0, -2000.0, -10000.0],
        apply=_set_collision_penalty,
    ),
    "goal-reward": SweepConfig(
        name="goal_reward",
        values=[10.0, 100.0, 1000.0, 10000.0, 100000.0],
        apply=_set_goal_reward,
    ),
    "gamma": SweepConfig(
        name="gamma",
        values=[0.50, 0.70, 0.85, 0.95, 0.99],
        apply=lambda world, value: None,
        affects_gamma=True,
    ),
}


@dataclass
class SweepRow:
    value: float
    iterations: int
    converged: bool
    v_start: float
    rollout_steps: int
    rollout_reward: float
    reached_goal: bool
    policy_diff_cells: int


def _rollout_summary(
    world: IntersectionWorld,
    policy: dict[MdpState, MdpAction | None],
    max_steps: int,
) -> tuple[int, float, bool]:
    state = world.initial_mdp_state()
    total_reward = 0.0
    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            return step - 1, total_reward, True
        action = policy.get(state)
        if action is None:
            return step - 1, total_reward, True
        next_state, hit_wall = world.transition(state, action)
        total_reward += world.reward_transition(state, action, next_state, hit_wall)
        state = next_state
        if world.is_terminal(state):
            return step, total_reward, True
    return max_steps, total_reward, False


def _policy_diff_cells(
    world: IntersectionWorld,
    gamma_a: float,
    values_a: dict[MdpState, float],
    gamma_b: float,
    values_b: dict[MdpState, float],
) -> int:
    if world.oriented_mdp:
        pa = world.aggregated_policy_per_cell(values_a, gamma_a)
        pb = world.aggregated_policy_per_cell(values_b, gamma_b)
    else:
        pa = a  # type: ignore[assignment]
        pb = b  # type: ignore[assignment]
    diff = 0
    for cell in pa:
        if cell in pb and pa[cell] != pb[cell]:
            diff += 1
    return diff


def run_sweep(
    sweep_name: str,
    base_world_factory: Callable[[], IntersectionWorld],
    gamma: float,
    theta: float,
    max_iterations: int,
    rollout_steps: int,
) -> list[SweepRow]:
    """Run the named sweep against a fresh baseline world for each value."""

    if sweep_name not in SWEEP_PRESETS:
        raise KeyError(f"unknown sweep '{sweep_name}'. choices: {list(SWEEP_PRESETS)}")

    preset = SWEEP_PRESETS[sweep_name]

    baseline_world = base_world_factory()
    baseline_solver = ValueIteration(
        world=baseline_world,
        gamma=gamma,
        theta=theta,
        max_iterations=max_iterations,
    )
    baseline_result = baseline_solver.solve()

    rows: list[SweepRow] = []
    for value in preset.values:
        world = base_world_factory()
        if preset.affects_gamma:
            current_gamma = value
        else:
            current_gamma = gamma
            preset.apply(world, value)

        solver = ValueIteration(
            world=world,
            gamma=current_gamma,
            theta=theta,
            max_iterations=max_iterations,
        )
        result = solver.solve()

        steps, total_reward, reached = _rollout_summary(world, result.policy, rollout_steps)
        diff = _policy_diff_cells(
            world,
            gamma,
            baseline_result.values,
            current_gamma,
            result.values,
        )
        rows.append(
            SweepRow(
                value=value,
                iterations=result.iterations,
                converged=result.converged,
                v_start=result.values.get(world.initial_mdp_state(), 0.0),
                rollout_steps=steps,
                rollout_reward=total_reward,
                reached_goal=reached,
                policy_diff_cells=diff,
            )
        )

    return rows


def print_sweep_table(sweep_name: str, rows: list[SweepRow]) -> None:
    preset = SWEEP_PRESETS[sweep_name]
    header = (
        f"{preset.name:>22s}  "
        f"{'iter':>5s}  "
        f"{'conv':>4s}  "
        f"{'V*(start)':>12s}  "
        f"{'steps':>5s}  "
        f"{'rollout_reward':>15s}  "
        f"{'reached':>7s}  "
        f"{'diff_cells':>10s}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        print(
            f"{row.value:>22.4f}  "
            f"{row.iterations:>5d}  "
            f"{'yes' if row.converged else 'no':>4s}  "
            f"{row.v_start:>12.2f}  "
            f"{row.rollout_steps:>5d}  "
            f"{row.rollout_reward:>15.2f}  "
            f"{'yes' if row.reached_goal else 'no':>7s}  "
            f"{row.policy_diff_cells:>10d}"
        )
