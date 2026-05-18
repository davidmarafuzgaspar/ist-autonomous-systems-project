"""Tabular Q-learning for policy adaptation after map change."""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from mdp_algorithm.world import IntersectionWorld, OrientedAction, PoseState


@dataclass
class QTable:
    gamma: float = 0.85
    alpha: float = 0.2
    epsilon: float = 0.1

    _q: dict[tuple[PoseState, OrientedAction], float] = field(default_factory=dict)

    def get(self, state: PoseState, action: OrientedAction) -> float:
        return self._q.get((state, action), 0.0)

    def set(self, state: PoseState, action: OrientedAction, value: float) -> None:
        self._q[(state, action)] = value

    def init_from_q_star(self, q_star: dict[tuple[PoseState, OrientedAction], float]) -> None:
        self._q = dict(q_star)

    def best_value(self, state: PoseState, world: IntersectionWorld) -> float:
        if world.is_terminal(state):
            return 0.0
        return max(self.get(state, a) for a in world.iter_actions())

    def greedy_action(self, state: PoseState, world: IntersectionWorld) -> OrientedAction:
        return max(world.iter_actions(), key=lambda a: self.get(state, a))

    def update(
        self,
        state: PoseState,
        action: OrientedAction,
        reward: float,
        next_state: PoseState,
        world: IntersectionWorld,
    ) -> float:
        old = self.get(state, action)
        target = reward + self.gamma * self.best_value(next_state, world)
        self._q[(state, action)] = old + self.alpha * (target - old)
        return self._q[(state, action)] - old

    def select_action(
        self,
        state: PoseState,
        world: IntersectionWorld,
        rng: random.Random,
    ) -> OrientedAction:
        if rng.random() < self.epsilon:
            return rng.choice(world.iter_actions())
        return self.greedy_action(state, world)


def q_star_from_values(
    world: IntersectionWorld,
    values: dict[PoseState, float],
    gamma: float,
) -> dict[tuple[PoseState, OrientedAction], float]:
    q: dict[tuple[PoseState, OrientedAction], float] = {}
    for state in world.get_all_states():
        for action in world.iter_actions():
            q[(state, action)] = world.bellman_action_value(state, action, values, gamma)
    return q
