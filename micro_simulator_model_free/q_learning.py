from __future__ import annotations

import random
from dataclasses import dataclass

import numpy as np

from .world import GridAction, GridCell, Heading, IntersectionWorld, PoseState

ALPHA_DEFAULT = 0.2
GAMMA_DEFAULT = 0.85
EPSILON_START_DEFAULT = 1.0
EPSILON_END_DEFAULT = 0.05
EPSILON_DECAY_DEFAULT = 0.995
NUM_EPISODES_DEFAULT = 1000
MAX_STEPS_DEFAULT = 50

ACTION_GLYPHS = {
    GridAction.STRAIGHT: "S",
    GridAction.TURN_RIGHT: "R",
    GridAction.TURN_LEFT: "L",
    GridAction.TURN_AROUND: "A",
}

HEADING_ARROW_GLYPHS: dict[Heading, str] = {
    Heading.N: "↑",
    Heading.E: "→",
    Heading.S: "↓",
    Heading.W: "←",
}


def policy_arrow_glyph(view_heading: Heading, action: GridAction) -> str:
    """Same semantics as the canvas arrows: straight = move dir.; turn = new facing."""
    if action == GridAction.STRAIGHT:
        move_h = view_heading
    elif action == GridAction.TURN_RIGHT:
        move_h = view_heading.turn_right()
    elif action == GridAction.TURN_LEFT:
        move_h = view_heading.turn_left()
    else:
        move_h = view_heading.turn_right().turn_right()
    return HEADING_ARROW_GLYPHS[move_h]


@dataclass(frozen=True)
class StepResult:
    episode: int
    step: int
    state: PoseState
    action: GridAction
    reward: float
    next_state: PoseState
    done: bool
    q_before: float
    q_after: float
    epsilon: float
    explored: bool
    episode_return: float


class QLearningTrainer:
    def __init__(
        self,
        world: IntersectionWorld,
        *,
        alpha: float = ALPHA_DEFAULT,
        gamma: float = GAMMA_DEFAULT,
        epsilon_start: float = EPSILON_START_DEFAULT,
        epsilon_end: float = EPSILON_END_DEFAULT,
        epsilon_decay: float = EPSILON_DECAY_DEFAULT,
        max_steps: int = MAX_STEPS_DEFAULT,
        num_episodes: int = NUM_EPISODES_DEFAULT,
        seed: int | None = 0,
    ) -> None:
        self.world = world
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon_start = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.max_steps = max_steps
        self.num_episodes = num_episodes
        self.rng = random.Random(seed)

        self.q_table = np.zeros((world.rows, world.cols, 4, 4), dtype=np.float64)
        self.epsilon = epsilon_start
        self.episode = 0
        self.step_in_episode = 0
        self.episode_return = 0.0
        self.episode_done = True
        self.total_episodes_finished = 0
        self.success_count = 0
        self.last_episode_return = 0.0
        self.last_episode_solved = False
        self.state = world.initial_state()
        self.last_step: StepResult | None = None

    def sync_world(self, world: IntersectionWorld) -> None:
        rows, cols = world.rows, world.cols
        if rows != self.q_table.shape[0] or cols != self.q_table.shape[1]:
            self.q_table = np.zeros((rows, cols, 4, 4), dtype=np.float64)
        else:
            self.q_table.fill(0.0)
        self.world = world
        self.reset_training()

    def reset_training(self) -> None:
        self.q_table.fill(0.0)
        self.epsilon = self.epsilon_start
        self.episode = 0
        self.step_in_episode = 0
        self.episode_return = 0.0
        self.episode_done = True
        self.total_episodes_finished = 0
        self.success_count = 0
        self.last_episode_return = 0.0
        self.last_episode_solved = False
        self.state = self.world.initial_state()
        self.last_step = None

    def start_episode(self) -> None:
        if self.total_episodes_finished >= self.num_episodes:
            return
        self.episode += 1
        self.step_in_episode = 0
        self.episode_return = 0.0
        self.episode_done = False
        self.state = self.world.initial_state()
        self.last_step = None

    def can_step(self) -> bool:
        return (
            not self.episode_done
            and self.episode > 0
            and self.total_episodes_finished < self.num_episodes
        )

    def training_finished(self) -> bool:
        return self.total_episodes_finished >= self.num_episodes

    def _select_action(self, state: PoseState) -> tuple[GridAction, bool]:
        explored = self.rng.random() < self.epsilon
        if explored:
            return self.rng.choice(self.world.iter_actions()), True
        row, col, h = state.cell.row, state.cell.col, state.heading.value
        action_idx = int(np.argmax(self.q_table[row, col, h, :]))
        return GridAction(action_idx), False

    def _update_q(
        self,
        state: PoseState,
        action: GridAction,
        reward: float,
        next_state: PoseState,
        done: bool,
    ) -> tuple[float, float]:
        row, col, h = state.cell.row, state.cell.col, state.heading.value
        a = int(action)
        q_before = float(self.q_table[row, col, h, a])
        if done:
            target = reward
        else:
            nr, nc, nh = next_state.cell.row, next_state.cell.col, next_state.heading.value
            target = reward + self.gamma * float(np.max(self.q_table[nr, nc, nh, :]))
        q_after = q_before + self.alpha * (target - q_before)
        self.q_table[row, col, h, a] = q_after
        return q_before, q_after

    def step(self) -> StepResult | None:
        if self.episode_done:
            if self.training_finished():
                return None
            self.start_episode()
        if self.episode_done or not self.can_step():
            return None

        state = self.state
        action, explored = self._select_action(state)
        next_state, reward, done = self.world.simulate_step(state, action)
        q_before, q_after = self._update_q(state, action, reward, next_state, done)

        self.step_in_episode += 1
        self.episode_return += reward
        eps_used = self.epsilon

        result = StepResult(
            episode=self.episode,
            step=self.step_in_episode,
            state=state,
            action=action,
            reward=reward,
            next_state=next_state,
            done=done,
            q_before=q_before,
            q_after=q_after,
            epsilon=eps_used,
            explored=explored,
            episode_return=self.episode_return,
        )
        self.last_step = result
        self.state = next_state

        if done or self.step_in_episode >= self.max_steps:
            self._finish_episode()
        return result

    def _finish_episode(self) -> None:
        self.episode_done = True
        self.total_episodes_finished += 1
        self.last_episode_return = self.episode_return
        self.last_episode_solved = bool(self.last_step and self.last_step.done)
        if self.last_episode_solved:
            self.success_count += 1
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)

    def run_all_episodes(self) -> int:
        """Headless training loop — same structure as ``solver._train_model_free``."""
        count = 0
        while not self.training_finished():
            self.run_episode()
            count += 1
        return count

    def run_episode(self) -> list[StepResult]:
        if self.training_finished():
            return []
        if self.episode_done:
            self.start_episode()
        records: list[StepResult] = []
        while self.can_step():
            rec = self.step()
            if rec is None:
                break
            records.append(rec)
        return records

    def greedy_action(self, state: PoseState) -> GridAction:
        row, col, h = state.cell.row, state.cell.col, state.heading.value
        return GridAction(int(np.argmax(self.q_table[row, col, h, :])))

    def max_q(self, state: PoseState) -> float:
        row, col, h = state.cell.row, state.cell.col, state.heading.value
        return float(np.max(self.q_table[row, col, h, :]))

    def q_value(self, state: PoseState, action: GridAction) -> float:
        row, col, h = state.cell.row, state.cell.col, state.heading.value
        return float(self.q_table[row, col, h, int(action)])

    def greedy_rollout(
        self,
        start: PoseState,
        *,
        max_steps: int = 128,
    ) -> list[PoseState]:
        """Follow greedy policy from ``start`` until goal, loop, or step limit."""
        state = start
        path = [state]
        seen: set[tuple[int, int, int]] = set()
        for _ in range(max_steps):
            if self.world.is_terminal(state):
                break
            key = (state.cell.row, state.cell.col, state.heading.value)
            if key in seen:
                break
            seen.add(key)
            action = self.greedy_action(state)
            next_state, _, done = self.world.simulate_step(state, action)
            if next_state == state:
                break
            path.append(next_state)
            state = next_state
            if done:
                break
        return path

    def format_policy_report(self, *, heading: Heading | None = None) -> str:
        """ASCII policy (like ``solver.Solver.format_policy_report``)."""
        headings = [heading] if heading is not None else list(Heading)
        title = (
            f"Policy (greedy Q) — heading {heading.name} (↑→↓← per cell):"
            if heading is not None
            else "Policy (greedy Q) — ↑→↓← per cell (move / facing after turn):"
        )
        lines: list[str] = [title]
        for h in headings:
            heading = h
            if len(headings) > 1:
                lines.append(f"  Heading {heading.name}:")
            prefix = "    " if len(headings) > 1 else "  "
            for row in range(self.world.rows):
                cells: list[str] = []
                for col in range(self.world.cols):
                    cell = GridCell(row, col)
                    if self.world.is_obstacle(cell):
                        cells.append("#")
                    else:
                        act = self.greedy_action(PoseState(cell, heading))
                        cells.append(policy_arrow_glyph(heading, act))
                lines.append(prefix + " ".join(cells))
        return "\n".join(lines)
