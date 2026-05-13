"""Bellman value iteration on the deterministic intersection MDP.

Step-by-step trace
------------------

A callback can be attached to observe every iteration. It receives the
iteration number, the current ``delta`` and a *copy* of the current
value function. This is what powers the ``--trace`` flag in
``main.py``.



Mathematical background
-----------------------

Given an MDP ``M = (S, A, T, R, gamma)``, the optimal state-value
function ``V*`` satisfies the Bellman optimality equation:

    V*(s) = max over a in A of  sum over s' of  T(s, a, s') * [ R(s, a, s') + gamma * V*(s') ]

In this project transitions are deterministic, so ``T(s, a, s')`` is 1
for a single next state ``s' = T(s, a)`` and 0 everywhere else. The
Bellman equation collapses to:

    V*(s) = max over a of  [ R(s, a, s') + gamma * V*(s') ]

Value iteration computes ``V*`` by repeatedly applying the Bellman
backup operator ``B`` to an arbitrary initial value function ``V_0``:

    V_{k+1}(s) = (B V_k)(s) = max over a of [ R(s, a, s') + gamma * V_k(s') ]

The operator ``B`` is a contraction with modulus ``gamma`` in the
infinity norm. This means each iteration multiplies the maximum error
by at most ``gamma``, so the algorithm converges geometrically to
``V*`` from any initial guess. We stop when the largest single-state
update ``delta = max_s |V_{k+1}(s) - V_k(s)|`` drops below ``theta``.

Once ``V*`` is known, the greedy policy is optimal:

    pi*(s) = argmax over a of [ R(s, a, s') + gamma * V*(s') ]

The goal state is treated as terminal: its value is fixed at 0 and no
backup is performed for it. Terminal rewards are captured the first
time a non-goal cell transitions into the goal cell.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .world import Action, GridCell, IntersectionWorld


IterationCallback = Callable[[int, float, dict[GridCell, float]], None]


@dataclass
class ValueIterationResult:
    values: dict[GridCell, float]
    policy: dict[GridCell, Action | None]
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
        """
        synchronous=False (default): Gauss-Seidel. Updates are written in
            place during the sweep, so later states in the same iteration
            already see the new values of earlier states. Faster
            convergence in practice.
        synchronous=True: Jacobi. Each iteration produces a new table
            using only the previous iteration's values, so the first
            iteration shows pure immediate rewards (no leaked future
            values).
        """

        self.world = world
        self.gamma = gamma
        self.theta = theta
        self.max_iterations = max_iterations
        self.verbose = verbose
        self.synchronous = synchronous

    def _action_value(
        self,
        state: GridCell,
        action: Action,
        values: dict[GridCell, float],
    ) -> float:
        """Compute the action-value ``Q(s, a) = R + gamma * V(s')``.

        With deterministic transitions there is exactly one ``s'`` per
        ``(s, a)`` pair, so no expectation is required.
        """

        next_cell, hit_wall = self.world.next_state(state, action)
        reward = self.world.reward(state, next_cell, hit_wall)
        return reward + self.gamma * values.get(next_cell, 0.0)

    def _bellman_backup(
        self,
        state: GridCell,
        values: dict[GridCell, float],
    ) -> float:
        """One step of the Bellman optimality operator at ``state``.

        Returns ``max_a Q(s, a)``.
        """

        return max(self._action_value(state, action, values) for action in Action)

    def initial_values(self) -> dict[GridCell, float]:
        return {state: 0.0 for state in self.world.get_all_states()}

    def step(
        self,
        values: dict[GridCell, float],
    ) -> tuple[dict[GridCell, float], float]:
        """Apply one Bellman sweep over all non-terminal states.

        Honors ``self.synchronous``:

        - ``False`` (Gauss-Seidel): writes in place, later states in the
          same iteration see earlier updates.
        - ``True`` (Jacobi): builds a brand new table from a snapshot,
          so the iteration is fully synchronous.
        """

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
        values: dict[GridCell, float],
    ) -> dict[GridCell, Action | None]:
        """Extract the greedy policy from a value table."""

        policy: dict[GridCell, Action | None] = {}
        for state in self.world.get_all_states():
            if self.world.is_terminal(state):
                policy[state] = None
                continue
            policy[state] = max(
                Action,
                key=lambda action: self._action_value(state, action, values),
            )
        return policy

    def solve(
        self,
        on_iteration: IterationCallback | None = None,
    ) -> ValueIterationResult:
        states = self.world.get_all_states()
        values: dict[GridCell, float] = {state: 0.0 for state in states}

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

        policy: dict[GridCell, Action | None] = {}
        for state in states:
            if self.world.is_terminal(state):
                policy[state] = None
                continue
            policy[state] = max(
                Action,
                key=lambda action: self._action_value(state, action, values),
            )

        return ValueIterationResult(
            values=values,
            policy=policy,
            iterations=iteration,
            final_delta=delta,
            converged=converged,
        )
