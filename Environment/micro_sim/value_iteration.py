"""Bellman value iteration on the deterministic intersection MDP.

Supports both cell-only states (``GridCell``) and oriented states
(``PoseState``) depending on ``IntersectionWorld.oriented_mdp``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .world import IntersectionWorld, MdpAction, MdpState


IterationCallback = Callable[[int, float, dict[MdpState, float]], None]


@dataclass
class ValueIterationResult:
    values: dict[MdpState, float]
    policy: dict[MdpState, MdpAction | None]
    iterations: int
    final_delta: float
    converged: bool


class ValueIteration:
    def __init__(
        self,
        world: IntersectionWorld,
        gamma: float = 0.85,
        theta: float = 1e-3,
        max_iterations: int = 1000,
        verbose: bool = False,
        synchronous: bool = False,
    ) -> None:
        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.verbose = verbose
        self.synchronous = synchronous

    def _action_value(
        self,
        state: MdpState,
        action: MdpAction,
        values: dict[MdpState, float],
    ) -> float:
        return self.world.bellman_action_value(state, action, values, self.gamma)

    def _bellman_backup(
        self,
        state: MdpState,
        values: dict[MdpState, float],
    ) -> float:
        actions = self.world.iter_actions()
        return max(self._action_value(state, action, values) for action in actions)

    def initial_values(self) -> dict[MdpState, float]:
        return {state: 0.0 for state in self.world.get_all_states()}

    def step(
        self,
        values: dict[MdpState, float],
    ) -> tuple[dict[MdpState, float], float]:
        if self.synchronous:
            snapshot = dict(values)
            new_values = dict(values)
            delta = 0.0
            for state in self.world.get_all_states():
                if self.world.is_terminal(state):
                    continue
                old_value = snapshot[state]
                new_value = self._bellman_backup(state, snapshot)
                new_values[state] = new_value
                delta = max(delta, abs(new_value - old_value))
            return new_values, delta

        new_values = dict(values)
        delta = 0.0
        for state in self.world.get_all_states():
            if self.world.is_terminal(state):
                continue
            old_value = new_values[state]
            new_value = self._bellman_backup(state, new_values)
            new_values[state] = new_value
            delta = max(delta, abs(new_value - old_value))
        return new_values, delta

    def greedy_policy(
        self,
        values: dict[MdpState, float],
    ) -> dict[MdpState, MdpAction | None]:
        policy: dict[MdpState, MdpAction | None] = {}
        actions = self.world.iter_actions()
        for state in self.world.get_all_states():
            if self.world.is_terminal(state):
                policy[state] = None
                continue
            policy[state] = max(
                actions,
                key=lambda action: self._action_value(state, action, values),
            )
        return policy

    def solve(
        self,
        on_iteration: IterationCallback | None = None,
    ) -> ValueIterationResult:
        states = self.world.get_all_states()
        values: dict[MdpState, float] = {state: 0.0 for state in states}

        iteration = 0
        delta = float("inf")
        converged = False

        if on_iteration is not None:
            on_iteration(0, float("inf"), dict(values))

        while iteration < self.max_iterations:
            values, delta = self.step(values)

            iteration += 1
            if self.verbose:
                print(f"  iter {iteration:4d}  delta = {delta:.6f}")
            if on_iteration is not None:
                on_iteration(iteration, delta, dict(values))

            if delta < self.theta:
                converged = True
                break

        actions = self.world.iter_actions()
        policy: dict[MdpState, MdpAction | None] = {}
        for state in states:
            if self.world.is_terminal(state):
                policy[state] = None
                continue
            policy[state] = max(
                actions,
                key=lambda action: self._action_value(state, action, values),
            )

        return ValueIterationResult(
            values=values,
            policy=policy,
            iterations=iteration,
            final_delta=delta,
            converged=converged,
        )
