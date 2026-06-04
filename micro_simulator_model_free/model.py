"""Grid MDP + algorithm."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence

DEFAULT_GRID: list[list[int]] = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
]
DEFAULT_START_ROW = 0
DEFAULT_START_COL = 0
DEFAULT_START_HEADING_NAME = "S"
DEFAULT_GOAL_ROW = 4
DEFAULT_GOAL_COL = 4

GOAL_REWARD_DEFAULT = 100.0
ILLEGAL_MOVE_REWARD_DEFAULT = -50.0
REWARD_STRAIGHT_DEFAULT = -1.0
REWARD_TURN_RIGHT_DEFAULT = -5.0
REWARD_TURN_LEFT_DEFAULT = -5.0
REWARD_TURN_AROUND_DEFAULT = -10.0

FREE_CELL = 0
OBSTACLE_CELL = 1


class Heading(IntEnum):
    N = 0
    E = 1
    S = 2
    W = 3

    def turn_left(self) -> Heading:
        return Heading((int(self) - 1) % 4)

    def turn_right(self) -> Heading:
        return Heading((int(self) + 1) % 4)

    @classmethod
    def from_str(cls, name: str) -> Heading:
        key = name.strip().upper()
        try:
            return cls[key]
        except KeyError as exc:
            raise ValueError(f"Invalid heading {name!r}; use N, E, S, or W") from exc


HEADING_DELTA_RC: dict[Heading, tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}


class GridAction(IntEnum):
    STRAIGHT = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    TURN_AROUND = 3


@dataclass(frozen=True)
class GridCell:
    row: int
    col: int


@dataclass(frozen=True)
class PoseState:
    cell: GridCell
    heading: Heading


MIN_GRID_SIZE = 2
MAX_GRID_SIZE = 12


def empty_grid(rows: int, cols: int) -> list[list[int]]:
    if rows < MIN_GRID_SIZE or cols < MIN_GRID_SIZE:
        raise ValueError(f"grid must be at least {MIN_GRID_SIZE}x{MIN_GRID_SIZE}")
    if rows > MAX_GRID_SIZE or cols > MAX_GRID_SIZE:
        raise ValueError(f"grid must be at most {MAX_GRID_SIZE}x{MAX_GRID_SIZE}")
    return [[FREE_CELL for _ in range(cols)] for _ in range(rows)]


def _clone_grid(grid: Sequence[Sequence[int]]) -> list[list[int]]:
    return [list(row) for row in grid]


@dataclass
class IntersectionWorld:
    grid: list[list[int]] = field(default_factory=lambda: _clone_grid(DEFAULT_GRID))
    start: GridCell = field(default_factory=lambda: GridCell(DEFAULT_START_ROW, DEFAULT_START_COL))
    goal: GridCell = field(default_factory=lambda: GridCell(DEFAULT_GOAL_ROW, DEFAULT_GOAL_COL))
    start_heading: Heading = field(
        default_factory=lambda: Heading.from_str(DEFAULT_START_HEADING_NAME)
    )
    spacing_m: float = 0.30

    goal_reward: float = GOAL_REWARD_DEFAULT
    illegal_move_reward: float = ILLEGAL_MOVE_REWARD_DEFAULT
    reward_straight: float = REWARD_STRAIGHT_DEFAULT
    reward_turn_right: float = REWARD_TURN_RIGHT_DEFAULT
    reward_turn_left: float = REWARD_TURN_LEFT_DEFAULT
    reward_turn_around: float = REWARD_TURN_AROUND_DEFAULT

    @property
    def rows(self) -> int:
        return len(self.grid)

    @property
    def cols(self) -> int:
        return len(self.grid[0]) if self.rows else 0

    @property
    def action_rewards(self) -> dict[GridAction, float]:
        return {
            GridAction.STRAIGHT: self.reward_straight,
            GridAction.TURN_RIGHT: self.reward_turn_right,
            GridAction.TURN_LEFT: self.reward_turn_left,
            GridAction.TURN_AROUND: self.reward_turn_around,
        }

    @property
    def obstacles(self) -> set[GridCell]:
        blocked: set[GridCell] = set()
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == OBSTACLE_CELL:
                    blocked.add(GridCell(row, col))
        return blocked

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_obstacle(self, cell: GridCell) -> bool:
        return self.in_bounds(cell.row, cell.col) and self.grid[cell.row][cell.col] == OBSTACLE_CELL

    def is_traversable(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.grid[row][col] == FREE_CELL

    def is_terminal(self, state: PoseState) -> bool:
        return state.cell == self.goal

    def initial_state(self) -> PoseState:
        return PoseState(self.start, self.start_heading)

    def iter_actions(self) -> list[GridAction]:
        return list(GridAction)

    def get_all_states(self) -> list[PoseState]:
        states: list[PoseState] = []
        for row in range(self.rows):
            for col in range(self.cols):
                if not self.is_traversable(row, col):
                    continue
                cell = GridCell(row, col)
                for heading in Heading:
                    states.append(PoseState(cell, heading))
        return states

    def _heading_after_action(self, heading: Heading, action: GridAction) -> Heading:
        if action == GridAction.TURN_RIGHT:
            return heading.turn_right()
        if action == GridAction.TURN_LEFT:
            return heading.turn_left()
        if action == GridAction.TURN_AROUND:
            return heading.turn_right().turn_right()
        return heading

    def simulate_step(
        self,
        state: PoseState,
        action: GridAction,
    ) -> tuple[PoseState, float, bool]:
        row, col = state.cell.row, state.cell.col
        heading = state.heading
        next_heading = self._heading_after_action(heading, action)

        dr, dc = HEADING_DELTA_RC[next_heading]
        next_row, next_col = row + dr, col + dc
        if not self.is_traversable(next_row, next_col):
            return PoseState(state.cell, heading), self.illegal_move_reward, False

        next_cell = GridCell(next_row, next_col)
        done = next_cell == self.goal
        if done:
            return PoseState(next_cell, next_heading), self.goal_reward, True

        return PoseState(next_cell, next_heading), self.action_rewards[action], False

    def world_xy(self, cell: GridCell) -> tuple[float, float]:
        half_col = (self.cols - 1) / 2.0
        half_row = (self.rows - 1) / 2.0
        x_m = (cell.col - half_col) * self.spacing_m
        y_m = (half_row - cell.row) * self.spacing_m
        return x_m, y_m

    @classmethod
    def from_size(
        cls,
        rows: int,
        cols: int,
        *,
        grid: Sequence[Sequence[int]] | None = None,
        start: GridCell | None = None,
        goal: GridCell | None = None,
        start_heading: Heading | str | None = None,
    ) -> IntersectionWorld:
        g = _clone_grid(grid) if grid is not None else empty_grid(rows, cols)
        if len(g) != rows or any(len(row) != cols for row in g):
            raise ValueError("grid dimensions do not match rows x cols")
        s = start if start is not None else GridCell(0, 0)
        gl = goal if goal is not None else GridCell(rows - 1, cols - 1)
        h = (
            start_heading
            if isinstance(start_heading, Heading)
            else Heading.from_str(start_heading or DEFAULT_START_HEADING_NAME)
        )
        return cls(grid=g, start=s, goal=gl, start_heading=h)


import random
from dataclasses import dataclass

import numpy as np

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
