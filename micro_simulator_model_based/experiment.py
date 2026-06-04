"""Phase A: MDP optimal on O0. Phase B: new obstacles O1, follow pi0*, learn Q on collisions."""

from __future__ import annotations

import random
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass

from mdp_algorithm.display import oriented_policy_glyph, print_layout
from mdp_algorithm.value_iteration import ValueIteration, ValueIterationResult
from mdp_algorithm.world import GridCell, IntersectionWorld, OrientedAction, PoseState

from .agent import QTable, q_star_from_values


@dataclass
class RolloutRecord:
    states: list[PoseState]
    actions: list[OrientedAction]
    rewards: list[float]
    hit_walls: list[bool]
    total_reward: float
    reached_goal: bool
    collisions: int

    @property
    def steps(self) -> int:
        return len(self.actions)

    def cells_visited(self) -> list[GridCell]:
        return [s.cell for s in self.states]


@dataclass
class PhaseAResult:
    world: IntersectionWorld
    vi: ValueIterationResult
    policy: dict[PoseState, OrientedAction | None]
    q_star: dict[tuple[PoseState, OrientedAction], float]
    rollout: RolloutRecord


@dataclass
class AdaptationResult:
    rollout_old_policy: RolloutRecord
    training_episodes: list[RolloutRecord]
    final_rollout: RolloutRecord
    vi_new: ValueIterationResult | None = None


def _pose_str(s: PoseState) -> str:
    return f"({s.cell.row},{s.cell.col}) h={s.heading.name}"


def rollout(
    world: IntersectionWorld,
    policy: dict[PoseState, OrientedAction | None]
    | Callable[[PoseState, IntersectionWorld], OrientedAction],
    start: PoseState | None = None,
    max_steps: int = 80,
    *,
    verbose: bool = False,
    label: str = "",
) -> RolloutRecord:
    state = start or world.initial_state()
    states = [state]
    actions: list[OrientedAction] = []
    rewards: list[float] = []
    hit_walls: list[bool] = []
    total = 0.0
    collisions = 0

    if verbose and label:
        print(f"\n--- {label}  start {_pose_str(state)} ---")

    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            break
        if callable(policy):
            action = policy(state, world)
        else:
            action = policy.get(state)
            if action is None:
                break

        next_state, hit_wall = world.transition(state, action)
        reward = world.reward_transition(state, action, next_state, hit_wall)
        actions.append(action)
        rewards.append(reward)
        hit_walls.append(hit_wall)
        total += reward
        if hit_wall:
            collisions += 1

        if verbose:
            g = oriented_policy_glyph(action, state.heading)
            wall = " WALL" if hit_wall else ""
            print(
                f"  {step:02d}  {_pose_str(state)}  {g}"
                f"  -> {_pose_str(next_state)}{wall}"
                f"  r={reward:>8.1f}  total={total:>9.1f}"
            )

        state = next_state
        states.append(state)
        if world.is_terminal(state):
            if verbose:
                print(f"  GOAL em {step} passos  colisoes={collisions}")
            break

    if verbose and not world.is_terminal(state):
        print(f"  sem goal em {max_steps} passos  colisoes={collisions}")

    return RolloutRecord(
        states=states,
        actions=actions,
        rewards=rewards,
        hit_walls=hit_walls,
        total_reward=total,
        reached_goal=world.is_terminal(state),
        collisions=collisions,
    )


def run_phase_a(
    world: IntersectionWorld | None = None,
    *,
    gamma: float = 0.85,
    theta: float = 1e-3,
    max_iterations: int = 1000,
    max_steps: int = 80,
    verbose: bool = True,
) -> PhaseAResult:
    world = world or IntersectionWorld()
    if verbose:
        print("\n=== FASE A — mapa O0 (MDP, caminho otimo de referencia) ===")
        print_layout(world)

    vi_result = ValueIteration(world, gamma=gamma, theta=theta, max_iterations=max_iterations).solve()
    q_star = q_star_from_values(world, vi_result.values, gamma)

    if verbose:
        print(f"VI: convergiu={vi_result.converged}  iteracoes={vi_result.iterations}")

    rec = rollout(world, vi_result.policy, max_steps=max_steps, verbose=verbose, label="rollout pi0* em O0")

    return PhaseAResult(
        world=world,
        vi=vi_result,
        policy=vi_result.policy,
        q_star=q_star,
        rollout=rec,
    )


def build_obstacles_blocking_path(
    base: set[GridCell],
    path_cells: list[GridCell],
    *,
    skip_start: GridCell,
    skip_goal: GridCell,
    block_count: int = 2,
) -> set[GridCell]:
    """O1: keep old obstacles shifted + block cells along the former optimal path."""
    shifted: set[GridCell] = set()
    for cell in base:
        nr, nc = cell.row, cell.col
        if nc + 1 < 5 and GridCell(nr, nc + 1) not in base:
            shifted.add(GridCell(nr, nc + 1))
        elif nr - 1 >= 0:
            shifted.add(GridCell(nr - 1, nc))
        else:
            shifted.add(cell)

    candidates = [
        c for c in path_cells
        if c != skip_start and c != skip_goal and c not in shifted
    ]
    mid = len(candidates) // 2
    for i in range(min(block_count, len(candidates))):
        idx = mid + i - block_count // 2
        if 0 <= idx < len(candidates):
            shifted.add(candidates[idx])
    return shifted


def make_world_new_obstacles(
    template: IntersectionWorld,
    new_obstacles: set[GridCell],
) -> IntersectionWorld:
    w = deepcopy(template)
    w.obstacles = set(new_obstacles)
    return w


@dataclass
class OnlineAdapter:
    q: QTable
    old_policy: dict[PoseState, OrientedAction | None]
    rng: random.Random
    use_old_policy_until_collision: bool = True
    had_collision: bool = False

    def pick_action(self, state: PoseState, world: IntersectionWorld) -> OrientedAction:
        if self.use_old_policy_until_collision and not self.had_collision:
            action = self.old_policy.get(state)
            if action is not None:
                return action
        return self.q.select_action(state, world, self.rng)

    def observe(
        self,
        state: PoseState,
        action: OrientedAction,
        reward: float,
        next_state: PoseState,
        hit_wall: bool,
        world: IntersectionWorld,
    ) -> None:
        if hit_wall:
            self.had_collision = True
        self.q.update(state, action, reward, next_state, world)


def train_after_map_change(
    world_new: IntersectionWorld,
    phase_a: PhaseAResult,
    *,
    episodes: int = 40,
    max_steps: int = 80,
    gamma: float = 0.85,
    init_q_from_vi: bool = True,
    verbose: bool = True,
) -> AdaptationResult:
    q = QTable(gamma=gamma)
    if init_q_from_vi:
        q.init_from_q_star(phase_a.q_star)

    adapter = OnlineAdapter(
        q=q,
        old_policy=phase_a.policy,
        rng=random.Random(0),
    )

    if verbose:
        print("\n=== FASE B — mapa O1 (obstaculos mudaram) ===")
        print_layout(world_new)
        print("\nSeguir pi0* no mapa novo (antes / durante aprendizagem):")

    training: list[RolloutRecord] = []

    def policy_fn(s: PoseState, w: IntersectionWorld) -> OrientedAction:
        return adapter.pick_action(s, w)

    def step_hook(
        s: PoseState,
        a: OrientedAction,
        r: float,
        s_next: PoseState,
        wall: bool,
        w: IntersectionWorld,
    ) -> None:
        adapter.observe(s, a, r, s_next, wall, w)

    for ep in range(1, episodes + 1):
        adapter.had_collision = False
        label = ""
        if verbose:
            if ep == 1:
                label = "episodio 1: pi0* ate colidir, depois Q"
            elif ep <= 3:
                label = f"episodio {ep} (Q)"

        rec = rollout_with_learning(
            world_new,
            policy_fn,
            step_hook,
            max_steps=max_steps,
            verbose=verbose and ep <= 3,
            label=label,
        )
        training.append(rec)
        if ep == 1:
            adapter.use_old_policy_until_collision = False
        if verbose and (ep % 10 == 0 or ep == episodes):
            ok = sum(1 for t in training if t.reached_goal)
            print(
                f"  [ep {ep}/{episodes}]  sucesso={ok}/{ep}  "
                f"ultimo: {'GOAL' if rec.reached_goal else '---'}  col={rec.collisions}"
            )

    first = training[0]

    final = rollout(
        world_new,
        q.greedy_action,
        max_steps=max_steps,
        verbose=verbose,
        label="rollout final: argmax Q em O1",
    )

    vi_new = ValueIteration(world_new, gamma=gamma).solve()
    bench = rollout(
        world_new,
        vi_new.policy,
        max_steps=max_steps,
        verbose=verbose,
        label="rollout pi1* (VI em O1, benchmark)",
    )

    if verbose:
        print_summary(phase_a, first, final, bench)

    return AdaptationResult(
        rollout_old_policy=first,
        training_episodes=training,
        final_rollout=final,
        vi_new=vi_new,
    )


def rollout_with_learning(
    world: IntersectionWorld,
    policy: Callable[[PoseState, IntersectionWorld], OrientedAction],
    on_step: Callable[[PoseState, OrientedAction, float, PoseState, bool, IntersectionWorld], None],
    start: PoseState | None = None,
    max_steps: int = 80,
    *,
    verbose: bool = False,
    label: str = "",
) -> RolloutRecord:
    state = start or world.initial_state()
    states = [state]
    actions: list[OrientedAction] = []
    rewards: list[float] = []
    hit_walls: list[bool] = []
    total = 0.0
    collisions = 0

    if verbose and label:
        print(f"\n--- {label}  start {_pose_str(state)} ---")

    for step in range(1, max_steps + 1):
        if world.is_terminal(state):
            break
        action = policy(state, world)
        next_state, hit_wall = world.transition(state, action)
        reward = world.reward_transition(state, action, next_state, hit_wall)
        on_step(state, action, reward, next_state, hit_wall, world)
        actions.append(action)
        rewards.append(reward)
        hit_walls.append(hit_wall)
        total += reward
        if hit_wall:
            collisions += 1
        if verbose:
            g = oriented_policy_glyph(action, state.heading)
            wall = " WALL" if hit_wall else ""
            print(
                f"  {step:02d}  {_pose_str(state)}  {g}"
                f"  -> {_pose_str(next_state)}{wall}"
                f"  r={reward:>8.1f}  total={total:>9.1f}"
            )
        state = next_state
        states.append(state)
        if world.is_terminal(state):
            break

    return RolloutRecord(
        states=states,
        actions=actions,
        rewards=rewards,
        hit_walls=hit_walls,
        total_reward=total,
        reached_goal=world.is_terminal(state),
        collisions=collisions,
    )


def print_summary(
    phase_a: PhaseAResult,
    first_on_new: RolloutRecord,
    final_q: RolloutRecord,
    bench: RolloutRecord,
) -> None:
    print("\n=== Resumo ===")
    print(f"Fase A (O0)  pi0*:  {phase_a.rollout.steps} passos  goal={phase_a.rollout.reached_goal}")
    print(
        f"Fase B  pi0* em O1: {first_on_new.steps} passos  goal={first_on_new.reached_goal}  "
        f"colisoes={first_on_new.collisions}"
    )
    print(
        f"Fase B  Q greedy:   {final_q.steps} passos  goal={final_q.reached_goal}  "
        f"colisoes={final_q.collisions}"
    )
    print(
        f"Benchmark pi1*:    {bench.steps} passos  goal={bench.reached_goal}"
    )


def run_full_experiment(
    *,
    gamma: float = 0.85,
    episodes: int = 40,
    max_steps: int = 80,
    verbose: bool = True,
) -> tuple[PhaseAResult, IntersectionWorld, AdaptationResult]:
    phase_a = run_phase_a(gamma=gamma, max_steps=max_steps, verbose=verbose)
    new_obs = build_obstacles_blocking_path(
        phase_a.world.obstacles,
        phase_a.rollout.cells_visited(),
        skip_start=phase_a.world.start,
        skip_goal=phase_a.world.goal,
    )
    world_new = make_world_new_obstacles(phase_a.world, new_obs)
    adapt = train_after_map_change(
        world_new,
        phase_a,
        episodes=episodes,
        max_steps=max_steps,
        gamma=gamma,
        verbose=verbose,
    )
    return phase_a, world_new, adapt
