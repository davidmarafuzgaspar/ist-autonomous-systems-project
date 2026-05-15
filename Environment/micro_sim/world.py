"""Oriented MDP on the 5x5 intersection grid (aligned with ``python_sim``)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Heading(Enum):
    """Cardinal heading (row decreases = North)."""

    N = 0
    E = 1
    S = 2
    W = 3

    def turn_left(self) -> Heading:
        return Heading((self.value - 1) % 4)

    def turn_right(self) -> Heading:
        return Heading((self.value + 1) % 4)


HEADING_DELTA_RC: dict[Heading, tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}


class OrientedAction(Enum):
    FORWARD = "F"
    TURN_LEFT = "L"
    TURN_RIGHT = "R"


@dataclass(frozen=True)
class GridCell:
    row: int
    col: int


@dataclass(frozen=True)
class PoseState:
    cell: GridCell
    heading: Heading


def _default_obstacles() -> set[GridCell]:
    return {
        GridCell(2, 0),
        GridCell(0, 1),
        GridCell(2, 3),
        GridCell(3, 2),
        GridCell(3, 4),
    }


@dataclass
class IntersectionWorld:
    """5x5 grid; state ``PoseState(cell, heading)``; actions F / L / R."""

    rows: int = 5
    cols: int = 5
    spacing_m: float = 0.30
    obstacles: set[GridCell] = field(default_factory=_default_obstacles)
    start: GridCell = GridCell(4, 0)
    goal: GridCell = GridCell(0, 4)
    start_heading: Heading = Heading.N

    goal_reward: float = 10000.0
    collision_penalty: float = -500.0
    away_from_goal_penalty: float = -100.0
    step_cost: float = -1.0
    turn_90_reward: float = -5.0

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def is_obstacle(self, cell: GridCell) -> bool:
        return cell in self.obstacles

    def is_traversable(self, cell: GridCell) -> bool:
        return self.in_bounds(cell) and not self.is_obstacle(cell)

    def is_terminal(self, state: PoseState) -> bool:
        return state.cell == self.goal

    def initial_state(self) -> PoseState:
        return PoseState(self.start, self.start_heading)

    def iter_actions(self) -> list[OrientedAction]:
        return list(OrientedAction)

    def get_all_states(self) -> list[PoseState]:
        states: list[PoseState] = []
        for row in range(self.rows):
            for col in range(self.cols):
                cell = GridCell(row, col)
                if self.is_obstacle(cell):
                    continue
                for heading in Heading:
                    states.append(PoseState(cell, heading))
        return states

    def transition(self, state: PoseState, action: OrientedAction) -> tuple[PoseState, bool]:
        if action == OrientedAction.FORWARD:
            d_row, d_col = HEADING_DELTA_RC[state.heading]
            target = GridCell(state.cell.row + d_row, state.cell.col + d_col)
            if self.is_traversable(target):
                return PoseState(target, state.heading), False
            return state, True
        if action == OrientedAction.TURN_LEFT:
            return PoseState(state.cell, state.heading.turn_left()), False
        if action == OrientedAction.TURN_RIGHT:
            return PoseState(state.cell, state.heading.turn_right()), False
        raise AssertionError(f"unknown action: {action}")

    def reward_transition(
        self,
        state: PoseState,
        action: OrientedAction,
        next_state: PoseState,
        hit_wall: bool,
    ) -> float:
        if action in (OrientedAction.TURN_LEFT, OrientedAction.TURN_RIGHT):
            return self.turn_90_reward
        return self._move_reward(state.cell, next_state.cell, hit_wall)

    def _move_reward(self, current: GridCell, next_cell: GridCell, hit_wall: bool) -> float:
        if next_cell == self.goal:
            return self.goal_reward
        if hit_wall:
            return self.collision_penalty
        if self._manhattan_to_goal(next_cell) > self._manhattan_to_goal(current):
            return self.away_from_goal_penalty
        return self.step_cost

    def bellman_action_value(
        self,
        state: PoseState,
        action: OrientedAction,
        values: dict[PoseState, float],
        gamma: float,
    ) -> float:
        next_state, hit_wall = self.transition(state, action)
        reward = self.reward_transition(state, action, next_state, hit_wall)
        return reward + gamma * values.get(next_state, 0.0)

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
    ) -> dict[GridCell, OrientedAction | None]:
        actions = list(OrientedAction)
        result: dict[GridCell, OrientedAction | None] = {}
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

                def q_best_heading(a: OrientedAction) -> float:
                    return max(
                        self.bellman_action_value(PoseState(cell, h), a, values, gamma)
                        for h in Heading
                    )

                result[cell] = max(actions, key=q_best_heading)
        return result

    def display_heading_for_cell_action(
        self,
        cell: GridCell,
        action: OrientedAction | None,
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
                -h.value,
            ),
        )

    def display_heading_map_for_cell_policy(
        self,
        cell_policy: dict[GridCell, OrientedAction | None],
        values: dict[PoseState, float],
        gamma: float,
    ) -> dict[GridCell, Heading]:
        return {
            cell: self.display_heading_for_cell_action(cell, act, values, gamma)
            for cell, act in cell_policy.items()
            if act is not None
        }

    def world_xy(self, cell: GridCell) -> tuple[float, float]:
        half_col = (self.cols - 1) / 2.0
        half_row = (self.rows - 1) / 2.0
        x_m = (cell.col - half_col) * self.spacing_m
        y_m = (half_row - cell.row) * self.spacing_m
        return x_m, y_m

    def _manhattan_to_goal(self, cell: GridCell) -> int:
        return abs(cell.row - self.goal.row) + abs(cell.col - self.goal.col)
