"""Estado partilhado entre passos do viewer."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from enum import Enum

from mdp_algorithm.world import GridCell, IntersectionWorld, OrientedAction, PoseState

from .agent import QTable
from .experiment import (
    OnlineAdapter,
    PhaseAResult,
    build_obstacles_blocking_path,
    make_world_new_obstacles,
    run_phase_a,
)


class ViewerMode(Enum):
    FASE_A = "A"
    FASE_B = "B"


@dataclass
class LastStepInfo:
    action: OrientedAction
    reward: float
    hit_wall: bool
    old_q: float
    new_q: float
    used_pi0: bool


@dataclass
class ViewerSession:
    gamma: float = 0.85
    max_steps: int = 80

    phase_a: PhaseAResult | None = None
    world_o0: IntersectionWorld = field(init=False)
    world_o1: IntersectionWorld | None = None
    mode: ViewerMode = ViewerMode.FASE_A
    display_world: IntersectionWorld = field(init=False)

    state: PoseState | None = None
    step_in_episode: int = 0
    episode: int = 0
    episode_return: float = 0.0
    episode_done: bool = True
    collisions_ep: int = 0

    q: QTable = field(default_factory=QTable)
    adapter: OnlineAdapter | None = None
    rng: random.Random = field(default_factory=lambda: random.Random(0))

    reference_path: list[GridCell] = field(default_factory=list)
    trail: list[PoseState] = field(default_factory=list)
    last: LastStepInfo | None = None

    def __post_init__(self) -> None:
        if self.phase_a is None:
            self.phase_a = run_phase_a(verbose=False)
        self.world_o0 = self.phase_a.world
        self.display_world = self.world_o0
        self.reference_path = self.phase_a.rollout.cells_visited()
        self.begin_episode_a()

    def begin_episode_a(self) -> None:
        self.mode = ViewerMode.FASE_A
        self.display_world = self.world_o0
        self.episode += 1
        self.state = self.world_o0.initial_state()
        self.step_in_episode = 0
        self.episode_return = 0.0
        self.episode_done = False
        self.collisions_ep = 0
        self.trail = [self.state]
        self.last = None

    def switch_to_map_b(self) -> None:
        if self.world_o1 is None:
            new_obs = build_obstacles_blocking_path(
                self.world_o0.obstacles,
                self.reference_path,
                skip_start=self.world_o0.start,
                skip_goal=self.world_o0.goal,
            )
            self.world_o1 = make_world_new_obstacles(self.world_o0, new_obs)

        self.mode = ViewerMode.FASE_B
        self.display_world = self.world_o1
        self.q = QTable(gamma=self.gamma)
        self.q.init_from_q_star(self.phase_a.q_star)
        self.adapter = OnlineAdapter(
            q=self.q,
            old_policy=self.phase_a.policy,
            rng=self.rng,
        )
        self.begin_episode_b()

    def begin_episode_b(self) -> None:
        assert self.world_o1 is not None and self.adapter is not None
        self.episode += 1
        self.state = self.world_o1.initial_state()
        self.step_in_episode = 0
        self.episode_return = 0.0
        self.episode_done = False
        self.collisions_ep = 0
        self.trail = [self.state]
        self.last = None
        if self.episode > 1:
            self.adapter.had_collision = False

    def step(self) -> bool:
        """Returns False if cannot step."""
        if self.episode_done:
            if self.mode == ViewerMode.FASE_A:
                self.begin_episode_a()
            else:
                self.begin_episode_b()
        assert self.state is not None
        world = self.display_world

        if world.is_terminal(self.state):
            self.episode_done = True
            return False

        if self.mode == ViewerMode.FASE_A:
            return self._step_a(world)
        return self._step_b(world)

    def _step_a(self, world: IntersectionWorld) -> bool:
        assert self.state is not None
        action = self.phase_a.policy.get(self.state)
        if action is None:
            self.episode_done = True
            return False
        return self._apply(world, action, learn=False, used_pi0=True)

    def _step_b(self, world: IntersectionWorld) -> bool:
        assert self.state is not None and self.adapter is not None
        used_pi0 = (
            self.adapter.use_old_policy_until_collision
            and not self.adapter.had_collision
            and self.adapter.old_policy.get(self.state) is not None
        )
        action = self.adapter.pick_action(self.state, world)
        return self._apply(world, action, learn=True, used_pi0=used_pi0)

    def _apply(
        self,
        world: IntersectionWorld,
        action: OrientedAction,
        *,
        learn: bool,
        used_pi0: bool,
    ) -> bool:
        assert self.state is not None
        old_q = self.q.get(self.state, action) if learn else 0.0
        next_state, hit_wall = world.transition(self.state, action)
        reward = world.reward_transition(self.state, action, next_state, hit_wall)

        if learn and self.adapter is not None:
            self.adapter.observe(self.state, action, reward, next_state, hit_wall, world)
            if self.episode == 1 and self.adapter.had_collision:
                self.adapter.use_old_policy_until_collision = False
        new_q = self.q.get(self.state, action) if learn else 0.0

        self.step_in_episode += 1
        self.episode_return += reward
        if hit_wall:
            self.collisions_ep += 1

        self.last = LastStepInfo(
            action=action,
            reward=reward,
            hit_wall=hit_wall,
            old_q=old_q,
            new_q=new_q if learn else old_q,
            used_pi0=used_pi0,
        )

        self.state = next_state
        self.trail.append(self.state)
        if world.is_terminal(self.state) or self.step_in_episode >= self.max_steps:
            self.episode_done = True
        return True
