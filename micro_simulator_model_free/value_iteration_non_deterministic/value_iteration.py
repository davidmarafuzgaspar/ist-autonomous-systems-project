"""Value iteration using expected Q-values over the slip distribution."""

from __future__ import annotations

from dataclasses import dataclass

from .world import (
    GAMMA_DEFAULT,
    MAX_ITERATIONS_DEFAULT,
    THETA_DEFAULT,
    GridAction,
    IntersectionWorld,
    PoseState,
)


@dataclass
class ValueIterationResult:
    values: dict[PoseState, float]
    policy: dict[PoseState, GridAction | None]
    iterations: int
    final_delta: float
    converged: bool


class ValueIteration:
    """Tabular VI over expected Q (slip). ``synchronous``: Jacobi vs Gauss–Seidel."""

    def __init__(
        self,
        world: IntersectionWorld,
        gamma: float = GAMMA_DEFAULT,
        theta: float = THETA_DEFAULT,
        max_iterations: int = MAX_ITERATIONS_DEFAULT,
        synchronous: bool = False,
    ) -> None:
        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.synchronous = synchronous

    def _action_value(
        self,
        state: PoseState,
        action: GridAction,
        values: dict[PoseState, float],
    ) -> float:
        return self.world.bellman_action_value(state, action, values, self.gamma)

    def _bellman_backup(self, state: PoseState, values: dict[PoseState, float]) -> float:
        return max(self._action_value(state, action, values) for action in self.world.iter_actions())

    def initial_values(self) -> dict[PoseState, float]:
        return {state: 0.0 for state in self.world.get_all_states()}

    def step(self, values: dict[PoseState, float]) -> tuple[dict[PoseState, float], float]:
        if self.synchronous:
            snapshot = dict(values)
            new_values = dict(values)
            delta = 0.0
            for state in self.world.get_all_states():
                if self.world.is_terminal(state):
                    continue
                new_value = self._bellman_backup(state, snapshot)
                delta = max(delta, abs(new_value - snapshot[state]))
                new_values[state] = new_value
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

    def greedy_policy(self, values: dict[PoseState, float]) -> dict[PoseState, GridAction | None]:
        policy: dict[PoseState, GridAction | None] = {}
        actions = self.world.iter_actions()
        for state in self.world.get_all_states():
            if self.world.is_terminal(state):
                policy[state] = None
                continue
            policy[state] = max(actions, key=lambda action: self._action_value(state, action, values))
        return policy

    def solve(self) -> ValueIterationResult:
        values = self.initial_values()
        iteration = 0
        delta = float("inf")
        converged = False

        while iteration < self.max_iterations:
            values, delta = self.step(values)
            iteration += 1
            if delta < self.theta:
                converged = True
                break

        return ValueIterationResult(
            values=values,
            policy=self.greedy_policy(values),
            iterations=iteration,
            final_delta=delta,
            converged=converged,
        )
