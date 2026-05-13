"""Discrete MDP world for the AlphaBot2 intersection grid.

This module models the cross-shaped board from ``python_sim`` as a finite
Markov Decision Process (MDP):

    M = (S, A, T, R, gamma)

where:

- ``S`` is the set of grid cells (one per intersection of the board)
- ``A`` is the set of robot actions ``{UP, DOWN, LEFT, RIGHT}``
- ``T(s, a)`` is the deterministic transition function
- ``R(s, a, s')`` is the reward function (goal, collision and shaping terms)
- ``gamma`` is the discount factor used by the planner (not stored here)

The transition model used here is fully deterministic, matching the
"para ja deterministico" choice from the planning phase. The obstacle
layout and grid geometry are kept aligned with
``Environment/python_sim/simulation.py`` so the policy computed on top
of this MDP can be reused by the continuous simulator later.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Action(Enum):
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"


ACTION_DELTAS_RC: dict[Action, tuple[int, int]] = {
    Action.UP: (-1, 0),
    Action.DOWN: (1, 0),
    Action.LEFT: (0, -1),
    Action.RIGHT: (0, 1),
}


@dataclass(frozen=True)
class GridCell:
    row: int
    col: int


def _default_obstacles() -> set[GridCell]:
    """Obstacle cells aligned with ``python_sim.simulation.default_obstacles``.

    The five obstacles sit at the following world (x, y) positions, with
    the corresponding (row, col) on a 5x5 grid centered at the origin:

    - world (-0.60,  0.00) -> (row=2, col=0)
    - world (-0.30, +0.60) -> (row=0, col=1)
    - world (+0.30,  0.00) -> (row=2, col=3)
    - world ( 0.00, -0.30) -> (row=3, col=2)
    - world (+0.60, -0.30) -> (row=3, col=4)
    """

    return {
        GridCell(2, 0),
        GridCell(0, 1),
        GridCell(2, 3),
        GridCell(3, 2),
        GridCell(3, 4),
    }


@dataclass
class IntersectionWorld:
    """5x5 grid of intersections with obstacles, start, and goal.

    The state representation is intentionally minimal: just the cell the
    robot stands on. Heading is not part of the MDP state because the
    robot is allowed to commit to any of the four cardinal actions from
    any cell.

    Coordinate conventions:

    - rows grow downward (row 0 is the top of the board, world y = +max)
    - cols grow to the right (col 0 is the left edge, world x = -max)
    - action ``UP`` decreases ``row`` (moves towards world y = +y)
    """

    rows: int = 5
    cols: int = 5
    spacing_m: float = 0.30
    obstacles: set[GridCell] = field(default_factory=_default_obstacles)
    start: GridCell = GridCell(4, 0)
    goal: GridCell = GridCell(0, 4)

    goal_reward: float = 10000.0
    collision_penalty: float = -500.0
    away_from_goal_penalty: float = -100.0
    step_cost: float = -1.0

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def is_obstacle(self, cell: GridCell) -> bool:
        return cell in self.obstacles

    def is_traversable(self, cell: GridCell) -> bool:
        return self.in_bounds(cell) and not self.is_obstacle(cell)

    def is_terminal(self, cell: GridCell) -> bool:
        return cell == self.goal

    def get_all_states(self) -> list[GridCell]:
        """Return every reachable (non-obstacle) cell on the grid."""

        return [
            GridCell(row, col)
            for row in range(self.rows)
            for col in range(self.cols)
            if not self.is_obstacle(GridCell(row, col))
        ]

    def next_state(self, cell: GridCell, action: Action) -> tuple[GridCell, bool]:
        """Deterministic transition T(s, a).

        Returns the resulting cell and a flag indicating whether the move
        was blocked. A blocked move leaves the robot in the same cell.
        Blocking happens when the target cell is out of bounds or holds
        an obstacle.
        """

        d_row, d_col = ACTION_DELTAS_RC[action]
        target = GridCell(cell.row + d_row, cell.col + d_col)
        if self.is_traversable(target):
            return target, False
        return cell, True

    def reward(self, current: GridCell, next_cell: GridCell, hit_wall: bool) -> float:
        """Reward function R(s, a, s').

        The reward shape mirrors the reference Micro Simulator:

        - reaching the goal grants ``goal_reward``
        - bumping into a wall or obstacle pays ``collision_penalty``
        - moving away from the goal (Manhattan distance grows) pays
          ``away_from_goal_penalty``
        - any other move pays ``step_cost``
        """

        if next_cell == self.goal:
            return self.goal_reward
        if hit_wall:
            return self.collision_penalty

        old_dist = self._manhattan_to_goal(current)
        new_dist = self._manhattan_to_goal(next_cell)
        if new_dist > old_dist:
            return self.away_from_goal_penalty
        return self.step_cost

    def world_xy(self, cell: GridCell) -> tuple[float, float]:
        """Convert grid (row, col) to world (x_m, y_m).

        Uses the same convention as ``python_sim``: the grid is centered
        at the origin, columns grow to the right (x+), and rows grow
        downward (y-).
        """

        half_col = (self.cols - 1) / 2.0
        half_row = (self.rows - 1) / 2.0
        x_m = (cell.col - half_col) * self.spacing_m
        y_m = (half_row - cell.row) * self.spacing_m
        return x_m, y_m

    def _manhattan_to_goal(self, cell: GridCell) -> int:
        return abs(cell.row - self.goal.row) + abs(cell.col - self.goal.col)
