#!/usr/bin/env python3
"""Model-based, model-free, and dynamic solvers for AlphaBot2 grid navigation."""

from __future__ import annotations

import random
from collections.abc import Callable
from typing import Literal

import numpy as np

from world import (
    HEADING_DELTA,
    STRAIGHT,
    TURN_AROUND,
    TURN_LEFT,
    TURN_RIGHT,
    GridWorld,
    Heading,
)

ALPHA = 0.2
GAMMA = 0.85
EPSILON_START = 1.0
EPSILON_END = 0.05
EPSILON_DECAY = 0.995
NUM_EPISODES = 1000
MAX_STEPS = 50
VALUE_ITER_MAX_ITERS = 200
VALUE_ITER_TOL = 1e-6
Q_LEARNING_LOG_INTERVAL = 50
VALUE_ITER_LOG_INTERVAL = 1

ACTION_STRAIGHT = STRAIGHT
ACTION_TURN_RIGHT = TURN_RIGHT
ACTION_TURN_LEFT = TURN_LEFT
ACTION_TURN_AROUND = TURN_AROUND

ALL_ACTIONS = (
    ACTION_STRAIGHT,
    ACTION_TURN_RIGHT,
    ACTION_TURN_LEFT,
    ACTION_TURN_AROUND,
)

MODE_MODEL_FREE = "model_free"
MODE_MODEL_BASED = "model_based"
MODE_DYNAMIC = "dynamic"
SOLVER_MODES = (MODE_MODEL_FREE, MODE_MODEL_BASED, MODE_DYNAMIC)

PLANNER_Q_LEARNING = "q_learning"
PLANNER_VALUE_ITERATION = "value_iteration"
PLANNER_MODES = (PLANNER_Q_LEARNING, PLANNER_VALUE_ITERATION)

GOAL_REWARD = 100.0
ILLEGAL_MOVE_REWARD = -50.0
ACTION_REWARDS = {
    ACTION_STRAIGHT: -1.0,
    ACTION_TURN_RIGHT: -5.0,
    ACTION_TURN_LEFT: -5.0,
    ACTION_TURN_AROUND: -10.0,
}
ACTION_LABELS = {
    ACTION_STRAIGHT: "STRAIGHT",
    ACTION_TURN_RIGHT: "TURN_RIGHT",
    ACTION_TURN_LEFT: "TURN_LEFT",
    ACTION_TURN_AROUND: "TURN_AROUND",
}
ACTION_GLYPHS = {
    ACTION_STRAIGHT: "S",
    ACTION_TURN_RIGHT: "R",
    ACTION_TURN_LEFT: "L",
    ACTION_TURN_AROUND: "A",
}


class Solver:
    def __init__(
        self,
        world: GridWorld,
        alpha: float = ALPHA,
        gamma: float = GAMMA,
        epsilon_start: float = EPSILON_START,
        epsilon_end: float = EPSILON_END,
        epsilon_decay: float = EPSILON_DECAY,
        num_episodes: int = NUM_EPISODES,
        max_steps: int = MAX_STEPS,
        mode: Literal["model_free", "model_based", "dynamic"] = MODE_MODEL_FREE,
        value_iter_max_iters: int = VALUE_ITER_MAX_ITERS,
        value_iter_tol: float = VALUE_ITER_TOL,
    ) -> None:
        self.world = world
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon_start = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.num_episodes = num_episodes
        self.max_steps = max_steps
        self.mode = mode
        self.value_iter_max_iters = value_iter_max_iters
        self.value_iter_tol = value_iter_tol
        self.active_planner: Literal["q_learning", "value_iteration"] = PLANNER_Q_LEARNING

        self.start_row = world.row
        self.start_col = world.col
        self.start_heading = world.heading
        self.goal = world.goal

        rows = len(world._grid)
        cols = len(world._grid[0]) if rows else 0
        self.rows = rows
        self.cols = cols
        self.q_table = np.zeros((rows, cols, 4, 4), dtype=np.float64)
        self.values = np.zeros((rows, cols, 4), dtype=np.float64)
        self.policy = np.zeros((rows, cols, 4), dtype=np.int8)

        if self.mode not in SOLVER_MODES:
            raise ValueError(f"Unsupported solver mode {self.mode!r}; use {SOLVER_MODES}")

    def _simulate_step(
        self,
        row: int,
        col: int,
        heading: Heading,
        action: int,
    ) -> tuple[int, int, Heading, float, bool]:
        next_heading = heading
        if action == ACTION_TURN_RIGHT:
            next_heading = heading.turn_right()
        elif action == ACTION_TURN_LEFT:
            next_heading = heading.turn_left()
        elif action == ACTION_TURN_AROUND:
            next_heading = heading.turn_right().turn_right()

        dr, dc = HEADING_DELTA[next_heading]
        next_row, next_col = row + dr, col + dc
        if not self.world.is_traversable(next_row, next_col):
            # Illegal moves are explicitly part of learning: penalty + self-loop.
            return row, col, heading, ILLEGAL_MOVE_REWARD, False

        done = self.goal is not None and (next_row, next_col) == self.goal
        if done:
            return next_row, next_col, next_heading, GOAL_REWARD, True

        return next_row, next_col, next_heading, ACTION_REWARDS[action], False

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def _model_based_action_scores(self, row: int, col: int, heading: Heading) -> np.ndarray:
        scores = np.zeros(4, dtype=np.float64)
        for action in ALL_ACTIONS:
            next_row, next_col, next_heading, reward, done = self._simulate_step(
                row, col, heading, action
            )
            future = 0.0 if done else self.gamma * self.values[next_row, next_col, next_heading.value]
            scores[action] = reward + future
        return scores

    def _best_action(self, row: int, col: int, heading: Heading) -> int:
        if not self._in_bounds(row, col):
            return ACTION_STRAIGHT
        if self.active_planner == PLANNER_VALUE_ITERATION:
            if not self.world.is_traversable(row, col):
                return ACTION_STRAIGHT
            return int(self.policy[row, col, heading.value])
        state_qs = self.q_table[row, col, heading.value, :]
        return int(np.argmax(state_qs))

    def train(
        self,
        log_fn: Callable[[str], None] | None = None,
        log_interval: int | None = None,
    ) -> None:
        def emit(message: str) -> None:
            if log_fn is not None:
                log_fn(message)
            else:
                print(message)

        emit(f"[solver] mode={self.mode}")
        if self.mode == MODE_MODEL_BASED:
            vi_interval = (
                VALUE_ITER_LOG_INTERVAL if log_interval is None else log_interval
            )
            self._train_model_based(emit, vi_interval)
            self.active_planner = PLANNER_VALUE_ITERATION
        else:
            ql_interval = (
                Q_LEARNING_LOG_INTERVAL if log_interval is None else log_interval
            )
            self._train_model_free(emit, ql_interval)
            self.active_planner = PLANNER_Q_LEARNING
        emit(f"[solver] active_planner={self.active_planner}")

    def _train_model_free(
        self,
        emit: Callable[[str], None],
        log_interval: int,
    ) -> None:
        emit(
            f"[solver] training start episodes={self.num_episodes} max_steps={self.max_steps} "
            f"alpha={self.alpha} gamma={self.gamma} "
            f"epsilon=({self.epsilon_start}->{self.epsilon_end}, decay={self.epsilon_decay})"
        )
        epsilon = self.epsilon_start
        success_count = 0
        running_reward = 0.0
        running_steps = 0
        window_episodes = 0
        for episode in range(self.num_episodes):
            row, col, heading = self.start_row, self.start_col, self.start_heading
            episode_reward = 0.0
            solved = False
            for step in range(self.max_steps):
                if random.random() < epsilon:
                    action = random.choice(ALL_ACTIONS)
                else:
                    state_qs = self.q_table[row, col, heading.value, :]
                    action = int(np.argmax(state_qs))

                next_row, next_col, next_heading, reward, done = self._simulate_step(
                    row, col, heading, action
                )

                current_q = self.q_table[row, col, heading.value, action]
                if done:
                    target = reward
                else:
                    best_next = np.max(self.q_table[next_row, next_col, next_heading.value, :])
                    target = reward + self.gamma * best_next
                self.q_table[row, col, heading.value, action] = current_q + self.alpha * (
                    target - current_q
                )

                episode_reward += reward
                row, col, heading = next_row, next_col, next_heading
                if done:
                    solved = True
                    break

            running_reward += episode_reward
            running_steps += (step + 1)
            window_episodes += 1
            if solved:
                success_count += 1

            epsilon = max(self.epsilon_end, epsilon * self.epsilon_decay)
            is_log_episode = ((episode + 1) % max(1, log_interval) == 0) or (
                episode + 1 == self.num_episodes
            )
            if is_log_episode:
                avg_reward = running_reward / max(1, window_episodes)
                avg_steps = running_steps / max(1, window_episodes)
                emit(
                    f"[solver] episode {episode + 1}/{self.num_episodes} "
                    f"epsilon={epsilon:.3f} avg_reward={avg_reward:.2f} "
                    f"avg_steps={avg_steps:.1f} successes={success_count}/{window_episodes}"
                )
                running_reward = 0.0
                running_steps = 0
                success_count = 0
                window_episodes = 0
        for line in self.format_learned_map_report().splitlines():
            emit(line)
        emit("[solver] training complete")

    def _train_model_based(
        self,
        emit: Callable[[str], None],
        log_interval: int,
    ) -> None:
        rows, cols, _ = self.values.shape
        emit(
            f"[solver] value-iteration start max_iters={self.value_iter_max_iters} "
            f"tol={self.value_iter_tol} gamma={self.gamma}"
        )
        for iteration in range(self.value_iter_max_iters):
            delta = 0.0
            new_values = self.values.copy()
            for row in range(rows):
                for col in range(cols):
                    if not self.world.is_traversable(row, col):
                        continue
                    for heading in Heading:
                        if self.goal is not None and (row, col) == self.goal:
                            new_values[row, col, heading.value] = 0.0
                            self.policy[row, col, heading.value] = ACTION_STRAIGHT
                            continue

                        action_scores = self._model_based_action_scores(row, col, heading)
                        best_action = int(np.argmax(action_scores))
                        best_value = float(action_scores[best_action])
                        old_value = self.values[row, col, heading.value]
                        new_values[row, col, heading.value] = best_value
                        self.policy[row, col, heading.value] = best_action
                        delta = max(delta, abs(best_value - old_value))
            self.values = new_values

            if ((iteration + 1) % max(1, log_interval) == 0) or (
                iteration + 1 == self.value_iter_max_iters
            ):
                emit(
                    f"[solver] value-iteration iter={iteration + 1}/{self.value_iter_max_iters} "
                    f"delta={delta:.6f}"
                )
            if delta < self.value_iter_tol:
                emit(
                    f"[solver] value-iteration converged iter={iteration + 1} "
                    f"delta={delta:.6f}"
                )
                break
        emit("[solver] training complete")

    def get_action(self, row: int, col: int, heading: Heading) -> int:
        return self._best_action(row, col, heading)

    def format_policy_report(self) -> str:
        lines: list[str] = []
        lines.append("Per-heading action map (S=Straight, R=Right, L=Left, A=Around, #=Obstacle):")
        for heading in Heading:
            lines.append(f"Heading {heading.name}:")
            for row in range(self.rows):
                cells: list[str] = []
                for col in range(self.cols):
                    is_obstacle = not self.world.is_traversable(row, col)
                    if is_obstacle:
                        cells.append("#")
                        continue
                    action = self._best_action(row, col, heading)
                    cells.append(ACTION_GLYPHS[action])
                lines.append("  " + " ".join(cells))
            lines.append("")
        return "\n".join(lines).rstrip()

    def format_learned_map_report(self) -> str:
        lines: list[str] = []
        lines.append(
            "Learned map (##=obstacle, ..=free/unknown, R*=start pose, GG=goal):"
        )
        for row in range(self.rows):
            cells: list[str] = []
            for col in range(self.cols):
                if row == self.start_row and col == self.start_col:
                    cells.append(f"R{self.start_heading.name}")
                elif self.goal is not None and (row, col) == self.goal:
                    cells.append("GG")
                elif self.world.is_obstacle(row, col):
                    cells.append("##")
                else:
                    cells.append("..")
            lines.append("  " + "  ".join(cells))
        lines.append(
            f"start=({self.start_row},{self.start_col},{self.start_heading.name}) "
            f"goal={self.goal}"
        )
        return "\n".join(lines)

    def explain_action(self, row: int, col: int, heading: Heading) -> str:
        chosen_action = self.get_action(row, col, heading)
        if self.active_planner == PLANNER_VALUE_ITERATION:
            action_scores = self._model_based_action_scores(row, col, heading)
            scored_actions = ", ".join(
                f"{ACTION_LABELS[action]}={action_scores[action]:.2f}" for action in ALL_ACTIONS
            )
        else:
            scored_actions = ", ".join(
                f"{ACTION_LABELS[action]}={self.q_table[row, col, heading.value, action]:.2f}"
                for action in ALL_ACTIONS
            )
        return (
            f"state ({row},{col},{heading.name}) -> {ACTION_LABELS[chosen_action]} "
            f"from [{scored_actions}]"
        )

    def save(self, path: str) -> None:
        np.save(path, self.q_table)

    def load(self, path: str) -> None:
        self.q_table = np.load(path)

    def set_start_state(self, row: int, col: int, heading: Heading) -> None:
        self.start_row = row
        self.start_col = col
        self.start_heading = heading

    def update_discovered_obstacle(self, row: int, col: int) -> bool:
        return self.world.mark_obstacle(row, col)

    def path_contains_cell(
        self,
        target_row: int,
        target_col: int,
        *,
        start_row: int,
        start_col: int,
        start_heading: Heading,
        horizon: int = 64,
    ) -> bool:
        row, col, heading = start_row, start_col, start_heading
        seen_states: set[tuple[int, int, int]] = set()
        for _ in range(horizon):
            action = self.get_action(row, col, heading)
            next_row, next_col, next_heading, _, done = self._simulate_step(
                row, col, heading, action
            )
            if (next_row, next_col) == (target_row, target_col):
                return True
            state_key = (next_row, next_col, next_heading.value)
            if state_key in seen_states:
                return False
            seen_states.add(state_key)
            if done:
                return False
            # Self-loop means no planned progress.
            if (next_row, next_col, next_heading) == (row, col, heading):
                return False
            row, col, heading = next_row, next_col, next_heading
        return False

    def replan_from_state(
        self,
        row: int,
        col: int,
        heading: Heading,
        *,
        log_fn: Callable[[str], None] | None = None,
        log_interval: int | None = None,
    ) -> None:
        def emit(message: str) -> None:
            if log_fn is not None:
                log_fn(message)
            else:
                print(message)

        if self.mode != MODE_DYNAMIC:
            emit(
                f"[solver] replan ignored: mode={self.mode} "
                f"(runtime replan only in {MODE_DYNAMIC})"
            )
            return

        vi_interval = VALUE_ITER_LOG_INTERVAL if log_interval is None else log_interval
        self.set_start_state(row, col, heading)
        emit(f"[solver] dynamic replan from ({row},{col},{heading.name}) via value iteration")
        self._train_model_based(emit, vi_interval)
        self.active_planner = PLANNER_VALUE_ITERATION
        emit(f"[solver] active_planner={self.active_planner}")