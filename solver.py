#!/usr/bin/env python3
"""Q-learning solver for intersection-level AlphaBot2 navigation."""

from __future__ import annotations

import random
from collections.abc import Callable

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
MAX_STEPS = 200

ACTION_STRAIGHT = STRAIGHT
ACTION_TURN_RIGHT = TURN_RIGHT
ACTION_TURN_LEFT = TURN_LEFT
ACTION_TURN_AROUND = TURN_AROUND

GOAL_REWARD = 100.0
ILLEGAL_MOVE_REWARD = -10.0
ACTION_REWARDS = {
    ACTION_STRAIGHT: -1.0,
    ACTION_TURN_RIGHT: -3.0,
    ACTION_TURN_LEFT: -3.0,
    ACTION_TURN_AROUND: -6.0,
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
    ) -> None:
        self.world = world
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon_start = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.num_episodes = num_episodes
        self.max_steps = max_steps

        self.start_row = world.row
        self.start_col = world.col
        self.start_heading = world.heading
        self.goal = world.goal

        rows = len(world._grid)
        cols = len(world._grid[0]) if rows else 0
        self.q_table = np.zeros((rows, cols, 4, 4), dtype=np.float64)

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
            return row, col, heading, ILLEGAL_MOVE_REWARD, False

        done = self.goal is not None and (next_row, next_col) == self.goal
        if done:
            return next_row, next_col, next_heading, GOAL_REWARD, True

        return next_row, next_col, next_heading, ACTION_REWARDS[action], False

    def _next_heading_for_action(self, heading: Heading, action: int) -> Heading:
        if action == ACTION_TURN_RIGHT:
            return heading.turn_right()
        if action == ACTION_TURN_LEFT:
            return heading.turn_left()
        if action == ACTION_TURN_AROUND:
            return heading.turn_right().turn_right()
        return heading

    def _best_action(self, row: int, col: int, heading: Heading) -> int:
        valid_actions = self.world.get_valid_actions(row, col, heading)
        if not valid_actions:
            return ACTION_STRAIGHT
        state_qs = self.q_table[row, col, heading.value, valid_actions]
        return valid_actions[int(np.argmax(state_qs))]

    def train(
        self,
        log_fn: Callable[[str], None] | None = None,
        log_interval: int = 500,
    ) -> None:
        def emit(message: str) -> None:
            if log_fn is not None:
                log_fn(message)
            else:
                print(message)

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
                valid_actions = self.world.get_valid_actions(row, col, heading)
                if not valid_actions:
                    break

                if random.random() < epsilon:
                    action = random.choice(valid_actions)
                else:
                    state_qs = self.q_table[row, col, heading.value, valid_actions]
                    action = valid_actions[int(np.argmax(state_qs))]

                next_row, next_col, next_heading, reward, done = self._simulate_step(
                    row, col, heading, action
                )

                best_next = np.max(self.q_table[next_row, next_col, next_heading.value, :])
                current_q = self.q_table[row, col, heading.value, action]
                self.q_table[row, col, heading.value, action] = current_q + self.alpha * (
                    reward + self.gamma * best_next - current_q
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
        emit("[solver] training complete")

    def get_action(self, row: int, col: int, heading: Heading) -> int:
        return self._best_action(row, col, heading)

    def format_policy_report(self) -> str:
        rows = len(self.world._grid)
        cols = len(self.world._grid[0]) if rows else 0
        lines: list[str] = []
        lines.append("Per-heading action map (S=straight, R=right, L=left, U=u-turn, #=obstacle):")
        for heading in Heading:
            lines.append(f"heading {heading.name}:")
            for row in range(rows):
                cells: list[str] = []
                for col in range(cols):
                    if not self.world.is_traversable(row, col):
                        cells.append("#")
                        continue
                    action = self._best_action(row, col, heading)
                    cells.append(ACTION_GLYPHS[action])
                lines.append("  " + " ".join(cells))
            lines.append("")
        return "\n".join(lines).rstrip()

    def explain_action(self, row: int, col: int, heading: Heading) -> str:
        valid_actions = self.world.get_valid_actions(row, col, heading)
        if not valid_actions:
            return f"state ({row},{col},{heading.name}) no valid actions"
        chosen_action = self._best_action(row, col, heading)
        scored_actions = ", ".join(
            f"{ACTION_LABELS[action]}={self.q_table[row, col, heading.value, action]:.2f}"
            for action in valid_actions
        )
        return (
            f"state ({row},{col},{heading.name}) -> {ACTION_LABELS[chosen_action]} "
            f"from [{scored_actions}]"
        )

    def save(self, path: str) -> None:
        np.save(path, self.q_table)

    def load(self, path: str) -> None:
        self.q_table = np.load(path)
