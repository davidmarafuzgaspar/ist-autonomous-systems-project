"""Oriented grid MDP with stochastic perpendicular slip on the forward step."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence

DEFAULT_GRID: list[list[int]] = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
]
DEFAULT_START_ROW = 0
DEFAULT_START_COL = 0
DEFAULT_START_HEADING_NAME = "S"
DEFAULT_GOAL_ROW = 2
DEFAULT_GOAL_COL = 4

GAMMA_DEFAULT = 0.85
THETA_DEFAULT = 1e-6
MAX_ITERATIONS_DEFAULT = 200

GOAL_REWARD_DEFAULT = 100.0
ILLEGAL_MOVE_REWARD_DEFAULT = -50.0
REWARD_STRAIGHT_DEFAULT = -1.0
REWARD_TURN_RIGHT_DEFAULT = -5.0
REWARD_TURN_LEFT_DEFAULT = -5.0
REWARD_TURN_AROUND_DEFAULT = -10.0

SLIP_INTENDED_DEFAULT = 0.70
SLIP_LEFT_DEFAULT = 0.15
SLIP_RIGHT_DEFAULT = 0.15

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


ACTION_LABELS: dict[GridAction, str] = {
    GridAction.STRAIGHT: "STRAIGHT",
    GridAction.TURN_RIGHT: "TURN_RIGHT",
    GridAction.TURN_LEFT: "TURN_LEFT",
    GridAction.TURN_AROUND: "TURN_AROUND",
}


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

TransitionOutcome = tuple[PoseState, float, bool]


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
    """
    Grid world with expected Bellman backups over forward slip.

    Turn dynamics match the deterministic model; after each turn, the
    forward cell is intended, slip-left, or slip-right with configurable weights.
    """

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

    slip_prob_intended: float = SLIP_INTENDED_DEFAULT
    slip_prob_left: float = SLIP_LEFT_DEFAULT
    slip_prob_right: float = SLIP_RIGHT_DEFAULT

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

    def normalized_slip_probs(self) -> tuple[float, float, float]:
        total = self.slip_prob_intended + self.slip_prob_left + self.slip_prob_right
        if total <= 0.0:
            return (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
        return (
            self.slip_prob_intended / total,
            self.slip_prob_left / total,
            self.slip_prob_right / total,
        )

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

    def _forward_outcome(
        self,
        state: PoseState,
        move_heading: Heading,
        action: GridAction,
    ) -> TransitionOutcome:
        row, col = state.cell.row, state.cell.col
        original_heading = state.heading
        next_heading = self._heading_after_action(original_heading, action)

        dr, dc = HEADING_DELTA_RC[move_heading]
        next_row, next_col = row + dr, col + dc
        if not self.is_traversable(next_row, next_col):
            return PoseState(state.cell, original_heading), self.illegal_move_reward, False

        next_cell = GridCell(next_row, next_col)
        if next_cell == self.goal:
            return PoseState(next_cell, next_heading), self.goal_reward, True
        return PoseState(next_cell, next_heading), self.action_rewards[action], False

    def transition_distribution(
        self,
        state: PoseState,
        action: GridAction,
    ) -> list[tuple[PoseState, float, bool, float]]:
        """Return ``(next_state, reward, done, probability)``; probabilities sum to 1."""
        next_heading = self._heading_after_action(state.heading, action)
        p_int, p_left, p_right = self.normalized_slip_probs()
        branches = (
            (next_heading, p_int),
            (next_heading.turn_left(), p_left),
            (next_heading.turn_right(), p_right),
        )
        return [
            (*self._forward_outcome(state, move_h, action), prob)
            for move_h, prob in branches
        ]

    def simulate_step(
        self,
        state: PoseState,
        action: GridAction,
    ) -> TransitionOutcome:
        """Rollout helper: always use the intended forward branch (no slip sample)."""
        next_heading = self._heading_after_action(state.heading, action)
        return self._forward_outcome(state, next_heading, action)

    def sample_transition(
        self,
        state: PoseState,
        action: GridAction,
        rng: random.Random,
    ) -> TransitionOutcome:
        dist = self.transition_distribution(state, action)
        u = rng.random()
        cumulative = 0.0
        for next_state, reward, done, prob in dist:
            cumulative += prob
            if u <= cumulative:
                return next_state, reward, done
        last = dist[-1]
        return last[0], last[1], last[2]

    def bellman_action_value(
        self,
        state: PoseState,
        action: GridAction,
        values: dict[PoseState, float],
        gamma: float,
    ) -> float:
        total = 0.0
        for next_state, reward, done, prob in self.transition_distribution(state, action):
            future = 0.0 if done else gamma * values.get(next_state, 0.0)
            total += prob * (reward + future)
        return total

    def aggregate_max_v_per_cell(self, values: dict[PoseState, float]) -> dict[GridCell, float]:
        out: dict[GridCell, float] = {}
        for pose, value in values.items():
            prev = out.get(pose.cell)
            out[pose.cell] = value if prev is None else max(prev, value)
        return out

    def aggregated_policy_per_cell(
        self,
        values: dict[PoseState, float],
        gamma: float,
    ) -> dict[GridCell, GridAction | None]:
        actions = self.iter_actions()
        result: dict[GridCell, GridAction | None] = {}
        for row in range(self.rows):
            for col in range(self.cols):
                cell = GridCell(row, col)
                if self.is_obstacle(cell):
                    continue
                if cell == self.goal:
                    result[cell] = None
                    continue
                if cell == self.start:
                    pose = PoseState(cell, self.start_heading)
                    result[cell] = max(
                        actions,
                        key=lambda a: self.bellman_action_value(pose, a, values, gamma),
                    )
                    continue

                def q_best_heading(a: GridAction) -> float:
                    return max(
                        self.bellman_action_value(PoseState(cell, h), a, values, gamma)
                        for h in Heading
                    )

                result[cell] = max(actions, key=q_best_heading)
        return result

    def display_heading_for_cell_action(
        self,
        cell: GridCell,
        action: GridAction | None,
        values: dict[PoseState, float],
        gamma: float,
    ) -> Heading:
        if action is None:
            return Heading.N
        if cell == self.start:
            return self.start_heading
        return max(
            Heading,
            key=lambda h: (
                self.bellman_action_value(PoseState(cell, h), action, values, gamma),
                -int(h),
            ),
        )

    def display_heading_map_for_cell_policy(
        self,
        cell_policy: dict[GridCell, GridAction | None],
        values: dict[PoseState, float],
        gamma: float,
    ) -> dict[GridCell, Heading]:
        return {
            cell: self.display_heading_for_cell_action(cell, act, values, gamma)
            for cell, act in cell_policy.items()
            if act is not None
        }

    def movement_heading_for_action(self, draw_heading: Heading, action: GridAction) -> Heading:
        return self._heading_after_action(draw_heading, action)

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

    def toggle_obstacle(self, row: int, col: int) -> bool:
        if not self.in_bounds(row, col):
            return False
        cell = GridCell(row, col)
        if cell == self.start or cell == self.goal:
            return False
        if self.grid[row][col] == OBSTACLE_CELL:
            self.grid[row][col] = FREE_CELL
        else:
            self.grid[row][col] = OBSTACLE_CELL
        return True

    def set_start(self, row: int, col: int) -> bool:
        if not self.is_traversable(row, col):
            return False
        cell = GridCell(row, col)
        if cell == self.goal:
            return False
        self.start = cell
        return True

    def set_goal(self, row: int, col: int) -> bool:
        if not self.is_traversable(row, col):
            return False
        cell = GridCell(row, col)
        if cell == self.start:
            return False
        self.goal = cell
        return True

    def resize_grid(self, rows: int, cols: int) -> None:
        self.grid = empty_grid(rows, cols)
        if not self.in_bounds(self.start.row, self.start.col):
            self.start = GridCell(0, 0)
        if not self.is_traversable(self.start.row, self.start.col):
            self.start = self._first_free_cell() or self.start
        if not self.in_bounds(self.goal.row, self.goal.col):
            self.goal = GridCell(rows - 1, cols - 1)
        if not self.is_traversable(self.goal.row, self.goal.col) or self.goal == self.start:
            self.goal = self._last_free_cell() or self.goal

    def _first_free_cell(self) -> GridCell | None:
        for row in range(self.rows):
            for col in range(self.cols):
                if self.is_traversable(row, col):
                    return GridCell(row, col)
        return None

    def _last_free_cell(self) -> GridCell | None:
        for row in range(self.rows - 1, -1, -1):
            for col in range(self.cols - 1, -1, -1):
                if self.is_traversable(row, col):
                    return GridCell(row, col)
        return None
