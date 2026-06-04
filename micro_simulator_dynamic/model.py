"""Runtime sim: known map (VI), true map (hidden), sense ahead, replan with popup."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from solver import (
    ACTION_STRAIGHT,
    ACTION_TURN_AROUND,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    MODE_MODEL_BASED,
    Solver,
)
from world import FREE_CELL, HEADING_DELTA, OBSTACLE_CELL, GridWorld, Heading

FREE = FREE_CELL
OBSTACLE = OBSTACLE_CELL
HIDDEN = 2

AskRetrainFn = Callable[[int, int], bool]

AUTO_MISSION_MAX_STEPS = 256
VI_LOG_INTERVAL = 20


@dataclass
class Scenario:
    known_grid: list[list[int]]
    true_grid: list[list[int]]
    start: tuple[int, int]
    goal: tuple[int, int]
    start_heading: Heading
    solver_mode: str = MODE_MODEL_BASED

    @property
    def grid(self) -> list[list[int]]:
        """Alias for known map (e.g. when re-opening setup)."""
        return self.known_grid

    @property
    def rows(self) -> int:
        return len(self.known_grid)

    @property
    def cols(self) -> int:
        return len(self.known_grid[0]) if self.rows else 0

    @classmethod
    def empty(cls, rows: int = 5, cols: int = 5) -> Scenario:
        free = [[FREE for _ in range(cols)] for _ in range(rows)]
        return cls(
            known_grid=[row[:] for row in free],
            true_grid=[row[:] for row in free],
            start=(0, 0),
            goal=(rows - 1, cols - 1),
            start_heading=Heading.S,
        )


@dataclass
class SenseResult:
    ahead_row: int | None
    ahead_col: int | None
    discovered_new: bool
    blocks_plan: bool
    replanned: bool
    message: str


@dataclass
class RealRuntimeSim:
    scenario: Scenario
    solver: Solver = field(init=False)
    robot: GridWorld = field(init=False)
    trained: bool = False
    log: list[str] = field(default_factory=list)
    trail: list[tuple[int, int, str]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._reset_robot_pose()
        self._init_solver()
        self._record_trail()

    @property
    def rows(self) -> int:
        return self.scenario.rows

    @property
    def cols(self) -> int:
        return self.scenario.cols

    def _known_matrix(self) -> list[list[int]]:
        return [
            [OBSTACLE if cell == OBSTACLE else FREE for cell in row]
            for row in self.scenario.known_grid
        ]

    def _true_grid_copy(self) -> list[list[int]]:
        return [row[:] for row in self.scenario.true_grid]

    def _init_solver(self) -> None:
        self.solver = Solver(
            GridWorld.from_matrix(
                self._known_matrix(),
                self.scenario.start,
                self.scenario.start_heading,
                self.scenario.goal,
            ),
            mode=self.scenario.solver_mode,
        )

    def _true_blocked(self, row: int, col: int) -> bool:
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return True
        return self.scenario.true_grid[row][col] != FREE

    def _known_blocked(self, row: int, col: int) -> bool:
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return True
        return self.scenario.known_grid[row][col] == OBSTACLE

    def _reset_robot_pose(self) -> None:
        self.robot = GridWorld.from_matrix(
            self._true_grid_copy(),
            self.scenario.start,
            self.scenario.start_heading,
            self.scenario.goal,
        )

    def _sync_solver_world(self) -> None:
        self.solver.world = GridWorld.from_matrix(
            self._known_matrix(),
            (self.robot.row, self.robot.col),
            self.robot.heading,
            self.scenario.goal,
        )
        self.solver.rows = self.rows
        self.solver.cols = self.cols
        self.solver.goal = self.scenario.goal

    def _sync_robot_grid(self) -> None:
        self.robot = GridWorld.from_matrix(
            self._true_grid_copy(),
            (self.robot.row, self.robot.col),
            self.robot.heading,
            self.scenario.goal,
        )

    def _emit(self, message: str) -> None:
        self.log.append(message)
        if len(self.log) > 120:
            self.log = self.log[-120:]

    def _record_trail(self) -> None:
        key = (self.robot.row, self.robot.col, self.robot.heading.name)
        if not self.trail or self.trail[-1] != key:
            self.trail.append(key)

    def clear_trail(self) -> None:
        self.trail.clear()
        self._record_trail()

    def cell_ahead(self) -> tuple[int, int] | None:
        dr, dc = HEADING_DELTA[self.robot.heading]
        nr, nc = self.robot.row + dr, self.robot.col + dc
        if not self.robot.is_in_bounds(nr, nc):
            return None
        return nr, nc

    def policy_action(self, row: int, col: int, heading: Heading) -> int | None:
        if not self.trained:
            return None
        return self.solver.get_action(row, col, heading)

    def current_policy_action(self) -> int | None:
        return self.policy_action(self.robot.row, self.robot.col, self.robot.heading)

    def optimal_path_from(
        self,
        row: int,
        col: int,
        heading: Heading,
        *,
        max_steps: int = 64,
    ) -> list[tuple[int, int, Heading]]:
        if not self.trained:
            return []
        path = [(row, col, heading)]
        seen: set[tuple[int, int, int]] = set()
        for _ in range(max_steps):
            action = self.solver.get_action(row, col, heading)
            next_row, next_col, next_heading, _, done = self.solver._simulate_step(
                row, col, heading, action
            )
            if (next_row, next_col, next_heading) == (row, col, heading):
                break
            key = (next_row, next_col, next_heading.value)
            if key in seen:
                break
            seen.add(key)
            row, col, heading = next_row, next_col, next_heading
            path.append((row, col, heading))
            if done:
                break
        return path

    def optimal_path_from_robot(self) -> list[tuple[int, int, Heading]]:
        return self.optimal_path_from(
            self.robot.row, self.robot.col, self.robot.heading
        )

    def train(self) -> None:
        self._reset_robot_pose()
        self.clear_trail()
        self._init_solver()
        self._emit("[sim] Value iteration on known map…")
        self.solver.train(log_fn=self._emit, log_interval=VI_LOG_INTERVAL)
        self._sync_solver_world()
        self.trained = True
        self._emit("[sim] Training complete.")

    def replan(self) -> None:
        if not self.trained:
            self._emit("[sim] Train first.")
            return
        self._sync_solver_world()
        self._emit("[sim] Replan (VI) from current pose…")
        self.solver.replan_from_state(
            self.robot.row,
            self.robot.col,
            self.robot.heading,
            log_fn=self._emit,
            log_interval=VI_LOG_INTERVAL,
        )
        self._emit("[sim] Replan done.")

    def sense_ahead(self) -> SenseResult:
        ahead = self.cell_ahead()
        if ahead is None:
            msg = "Ahead: out of bounds."
            self._emit(msg)
            return SenseResult(None, None, False, False, False, msg)

        ar, ac = ahead
        if not self._true_blocked(ar, ac):
            msg = f"Ahead ({ar},{ac}): clear."
            self._emit(msg)
            return SenseResult(ar, ac, False, False, False, msg)

        if self._known_blocked(ar, ac):
            msg = f"Ahead ({ar},{ac}): known obstacle."
            self._emit(msg)
            return SenseResult(ar, ac, False, False, False, msg)

        self.scenario.known_grid[ar][ac] = OBSTACLE
        self.scenario.true_grid[ar][ac] = OBSTACLE
        self._sync_robot_grid()

        blocks = replanned = False
        if self.trained:
            blocks = self.solver.path_contains_cell(
                ar,
                ac,
                start_row=self.robot.row,
                start_col=self.robot.col,
                start_heading=self.robot.heading,
            )
            self.solver.update_discovered_obstacle(ar, ac)
            self._sync_solver_world()

        msg = f"NEW obstacle at ({ar},{ac})"
        if blocks:
            msg += " — blocks planned path"
        self._emit(msg)
        return SenseResult(ar, ac, True, blocks, replanned, msg)

    def handle_discovery(
        self,
        sense: SenseResult,
        ask_retrain: AskRetrainFn | None,
    ) -> str:
        if not sense.discovered_new or not sense.blocks_plan or not self.trained:
            return sense.message
        if ask_retrain is None:
            return sense.message + " (retrain skipped — no UI callback)"

        if ask_retrain(sense.ahead_row or 0, sense.ahead_col or 0):
            self.replan()
            return sense.message + " → replanned (VI on known map)."
        return sense.message + " → kept previous policy."

    def set_pose(self, row: int, col: int, heading: Heading) -> str:
        if not self.robot.is_in_bounds(row, col):
            return "Out of bounds."
        if self._true_blocked(row, col):
            return f"({row},{col}) blocked."
        self.robot = GridWorld.from_matrix(
            self._true_grid_copy(),
            (row, col),
            heading,
            self.scenario.goal,
        )
        self._sync_solver_world()
        self._record_trail()
        return self.robot.pose_str()

    def set_heading(self, heading: Heading) -> str:
        return self.set_pose(self.robot.row, self.robot.col, heading)

    def apply_policy_turn(self) -> str:
        if not self.trained:
            return "Train first."
        action = self.current_policy_action()
        if action is None:
            return "No policy."
        if action == ACTION_STRAIGHT:
            return "Policy: STRAIGHT — use Move forward / Next step."

        if action == ACTION_TURN_RIGHT:
            self.robot.turn_right()
        elif action == ACTION_TURN_LEFT:
            self.robot.turn_left()
        else:
            self.robot.turn_around()
        self._sync_solver_world()
        self._record_trail()
        return f"Turn → {self.robot.heading.name}"

    def move_forward(self, *, ask_retrain: AskRetrainFn | None = None) -> str:
        if self.robot.is_at_goal():
            return "At goal."
        if not self.trained:
            return "Train first."
        dr, dc = HEADING_DELTA[self.robot.heading]
        nr, nc = self.robot.row + dr, self.robot.col + dc
        if not self.robot.is_in_bounds(nr, nc):
            return "Out of bounds."
        if self._true_blocked(nr, nc):
            sense = self.sense_ahead()
            return self.handle_discovery(sense, ask_retrain)

        self.robot.step_to_next_intersection()
        self._sync_solver_world()
        self._record_trail()
        sense = self.sense_ahead()
        detail = self.handle_discovery(sense, ask_retrain)
        return f"Forward → {self.robot.pose_str()} · {detail}"

    def execute_policy_cycle(
        self,
        *,
        ask_retrain: AskRetrainFn | None = None,
    ) -> tuple[str, bool]:
        if self.robot.is_at_goal():
            return "Goal reached.", True
        if not self.trained:
            return "Train first.", False
        action = self.current_policy_action()
        if action is None:
            return "No policy.", False
        if action != ACTION_STRAIGHT:
            return self.apply_policy_turn(), False
        return self.move_forward(ask_retrain=ask_retrain), self.robot.is_at_goal()

    def manual_goto_adjacent(self, row: int, col: int) -> str:
        if (row, col) == (self.robot.row, self.robot.col):
            return "Same cell — use heading buttons."
        if abs(row - self.robot.row) + abs(col - self.robot.col) != 1:
            return "Click an adjacent cell."
        for h in Heading:
            dr, dc = HEADING_DELTA[h]
            if self.robot.row + dr == row and self.robot.col + dc == col:
                return self.set_pose(row, col, h)
        return "Invalid move."

    def reset_to_start(self) -> None:
        self._reset_robot_pose()
        self._sync_solver_world()
        self.clear_trail()
        self._emit("Back to start.")
