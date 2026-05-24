#!/usr/bin/env python3
"""
Discrete intersection grid for AlphaBot2 runtime pose tracking.

Cell encoding: 0 = traversable intersection, 1 = obstacle.
Row 0 is north; heading N decreases row, E increases col.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Sequence

# Map layout and start pose are configured in main.py (passed to from_matrix).

FREE_CELL = 0
OBSTACLE_CELL = 1

STRAIGHT = 0
TURN_RIGHT = 1
TURN_LEFT = 2
TURN_AROUND = 3

OBSTACLE_GLYPH = "##"
FREE_GLYPH = ".."
GOAL_GLYPH = "GG"
ROBOT_GLYPHS: dict[str, str] = {
    "N": "R^",
    "E": "R>",
    "S": "Rv",
    "W": "R<",
}


class Heading(Enum):
    """Cardinal heading (row decreases = North)."""

    N = 0
    E = 1
    S = 2
    W = 3

    @classmethod
    def from_str(cls, name: str) -> Heading:
        key = name.strip().upper()
        try:
            return cls[key]
        except KeyError as exc:
            raise ValueError(f"Invalid heading {name!r}; use N, E, S, or W") from exc

    def turn_right(self) -> Heading:
        return Heading((self.value + 1) % 4)

    def turn_left(self) -> Heading:
        return Heading((self.value - 1) % 4)

    def label(self) -> str:
        return self.name


HEADING_DELTA: dict[Heading, tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}


class GridWorld:
    """Intersection grid pose: (row, col, heading)."""

    def __init__(
        self,
        grid: Sequence[Sequence[int]],
        start: tuple[int, int],
        heading: Heading,
        goal: tuple[int, int] | None = None,
    ) -> None:
        self._grid = [list(row) for row in grid]
        self._rows = len(self._grid)
        self._cols = len(self._grid[0]) if self._rows else 0
        self._row, self._col = start
        self._heading = heading
        self._goal = goal
        self._validate_dimensions()
        self._validate_start()
        self._validate_goal()

    @classmethod
    def from_matrix(
        cls,
        grid: Sequence[Sequence[int]],
        start: tuple[int, int],
        heading: str | Heading,
        goal: tuple[int, int] | None = None,
    ) -> GridWorld:
        h = heading if isinstance(heading, Heading) else Heading.from_str(heading)
        return cls(grid, start, h, goal)

    @property
    def row(self) -> int:
        return self._row

    @property
    def col(self) -> int:
        return self._col

    @property
    def heading(self) -> Heading:
        return self._heading

    def pose_str(self) -> str:
        return f"({self._row},{self._col}) heading {self._heading.label()}"

    @property
    def goal(self) -> tuple[int, int] | None:
        return self._goal

    def is_at_goal(self) -> bool:
        if self._goal is None:
            return False
        return (self._row, self._col) == self._goal

    def is_traversable(self, row: int, col: int) -> bool:
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            return False
        return self._grid[row][col] == FREE_CELL

    def step_to_next_intersection(self) -> bool:
        """Advance one cell along current heading (junction arrival)."""
        dr, dc = HEADING_DELTA[self._heading]
        nr, nc = self._row + dr, self._col + dc
        if not self.is_traversable(nr, nc):
            return False
        self._row, self._col = nr, nc
        return True

    def turn_right(self) -> None:
        """Update heading after SEARCH finds the new branch (turn right)."""
        self._heading = self._heading.turn_right()

    def turn_left(self) -> None:
        """Update heading after SEARCH finds the new branch (turn left)."""
        self._heading = self._heading.turn_left()

    def turn_around(self) -> None:
        """Update heading after SEARCH finds the branch behind the robot."""
        self.turn_right()
        self.turn_right()

    def get_valid_actions(self, row: int, col: int, heading: Heading) -> list[int]:
        """Return legal actions from pose (row, col, heading)."""
        action_to_heading = {
            STRAIGHT: heading,
            TURN_RIGHT: heading.turn_right(),
            TURN_LEFT: heading.turn_left(),
            TURN_AROUND: heading.turn_right().turn_right(),
        }
        valid_actions: list[int] = []
        for action, next_heading in action_to_heading.items():
            dr, dc = HEADING_DELTA[next_heading]
            nr, nc = row + dr, col + dc
            if self.is_traversable(nr, nc):
                valid_actions.append(action)
        return valid_actions

    def heading_to_search_dir(self) -> str:
        """Physical SEARCH spin direction; policy is always turn right."""
        return "right"

    def format_map(self) -> str:
        lines = ["----------- Map -----------"]
        for row in range(self._rows):
            cells: list[str] = []
            for col in range(self._cols):
                if row == self._row and col == self._col:
                    cells.append(ROBOT_GLYPHS[self._heading.label()])
                elif self._goal is not None and (row, col) == self._goal:
                    cells.append(GOAL_GLYPH)
                elif self._grid[row][col] == OBSTACLE_CELL:
                    cells.append(OBSTACLE_GLYPH)
                else:
                    cells.append(FREE_GLYPH)
            lines.append("  " + "  ".join(cells))
        lines.append(f"--- pose {self.pose_str()} ---")
        return "\n".join(lines)

    def print_map(self, logger: Any | None = None) -> None:
        """Emit map once: ROS logger if given, otherwise stdout."""
        text = self.format_map()
        if logger is not None and hasattr(logger, "info"):
            logger.info("")
            for line in text.splitlines():
                logger.info(line)
            logger.info("")
        else:
            print()
            print(text)
            print()

    def _validate_dimensions(self) -> None:
        if self._rows == 0 or self._cols == 0:
            raise ValueError("MAP must be non-empty")
        for i, row in enumerate(self._grid):
            if len(row) != self._cols:
                raise ValueError(f"MAP row {i} has width {len(row)}, expected {self._cols}")

    def _validate_start(self) -> None:
        r, c = self._row, self._col
        if not (0 <= r < self._rows and 0 <= c < self._cols):
            raise ValueError(
                f"Start ({r},{c}) out of bounds for {self._rows}x{self._cols} grid"
            )
        if self._grid[r][c] != FREE_CELL:
            raise ValueError(f"START ({r},{c}) must be on a free cell (0), not obstacle (1)")

    def _validate_goal(self) -> None:
        if self._goal is None:
            return
        gr, gc = self._goal
        if not (0 <= gr < self._rows and 0 <= gc < self._cols):
            raise ValueError(
                f"Goal ({gr},{gc}) out of bounds for {self._rows}x{self._cols} grid"
            )
        if self._grid[gr][gc] != FREE_CELL:
            raise ValueError(f"Goal ({gr},{gc}) must be on a free cell (0), not obstacle (1)")
