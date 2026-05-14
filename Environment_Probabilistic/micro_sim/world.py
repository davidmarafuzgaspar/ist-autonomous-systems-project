"""Discrete MDP world for the AlphaBot2 intersection grid (stochastic motion).

Same grid and rewards as ``Environment/python_sim``, but **translation**
actions are noisy: with probabilities ``motion_prob_forward``,
``motion_prob_left``, ``motion_prob_right`` (normalized to sum to 1) the
robot attempts to move in the **intended** direction, or slips one cell in
the perpendicular **left** or **right** direction relative to that intent
(both in the grid frame). **Rotations** (``L`` / ``R`` in oriented mode) stay
deterministic.

Bellman backups use ``transition_distribution`` (expectation over ``s'``).
Rollouts sample one outcome per step via ``sample_transition``.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Union


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


class Heading(Enum):
    """Cardinal heading on the grid (row decreases = North / UP)."""

    N = 0
    E = 1
    S = 2
    W = 3

    def turn_left(self) -> "Heading":
        return Heading((self.value - 1) % 4)

    def turn_right(self) -> "Heading":
        return Heading((self.value + 1) % 4)


HEADING_DELTA_RC: dict[Heading, tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}


class OrientedAction(Enum):
    """Move forward in the current heading, or rotate 90° in place (L/R)."""

    FORWARD = "F"
    TURN_LEFT = "L"
    TURN_RIGHT = "R"


@dataclass(frozen=True)
class GridCell:
    row: int
    col: int


@dataclass(frozen=True)
class PoseState:
    """Robot at an intersection with a facing direction."""

    cell: GridCell
    heading: Heading


MdpState = Union[GridCell, PoseState]
MdpAction = Union[Action, OrientedAction]


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

    Two MDP modes:

    - ``oriented_mdp=False`` (default): state is a ``GridCell``; actions are
      ``Action`` (move in any cardinal direction from any pose).
    - ``oriented_mdp=True``: state is ``PoseState(cell, heading)``; actions
      are ``OrientedAction`` (``F`` forward with slip, ``L`` / ``R`` rotate 90°).
      Set ``turn_90_reward`` to ``0`` to disable the turn penalty.

    **Stochastic translation:** for ``F`` (and for each cardinal move in cell
    mode), the robot moves in the intended direction with probability
    ``motion_prob_forward``, or slips one cell to the perpendicular left /
    right with ``motion_prob_left`` / ``motion_prob_right`` (non-negative
    weights renormalized to sum to 1). Rotations are deterministic.

    Coordinate conventions:

    - rows grow downward (row 0 is the top of the board, world y = +max)
    - cols grow to the right (col 0 is the left edge, world x = -max)
    - ``Heading.N`` / ``Action.UP`` decrease ``row`` (towards world +y)
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

    oriented_mdp: bool = False
    start_heading: Heading = Heading.N
    turn_90_reward: float = -5.0

    #: Probabilities for noisy **translation** (intended, slip-left, slip-right).
    #: Defaults 0.6 / 0.2 / 0.2; values are renormalized if they do not sum to 1.
    motion_prob_forward: float = 0.6
    motion_prob_left: float = 0.2
    motion_prob_right: float = 0.2

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def is_obstacle(self, cell: GridCell) -> bool:
        return cell in self.obstacles

    def is_traversable(self, cell: GridCell) -> bool:
        return self.in_bounds(cell) and not self.is_obstacle(cell)

    def is_terminal(self, state: MdpState) -> bool:
        if self.oriented_mdp:
            assert isinstance(state, PoseState)
            return state.cell == self.goal
        assert isinstance(state, GridCell)
        return state == self.goal

    def initial_mdp_state(self) -> MdpState:
        if self.oriented_mdp:
            return PoseState(self.start, self.start_heading)
        return self.start

    def iter_actions(self) -> list[MdpAction]:
        if self.oriented_mdp:
            return list(OrientedAction)
        return list(Action)

    def get_all_states(self) -> list[MdpState]:
        if self.oriented_mdp:
            states: list[MdpState] = []
            for row in range(self.rows):
                for col in range(self.cols):
                    cell = GridCell(row, col)
                    if self.is_obstacle(cell):
                        continue
                    for heading in Heading:
                        states.append(PoseState(cell, heading))
            return states
        return [
            GridCell(row, col)
            for row in range(self.rows)
            for col in range(self.cols)
            if not self.is_obstacle(GridCell(row, col))
        ]

    def _normalized_motion_triple(self) -> tuple[float, float, float]:
        pf = max(0.0, self.motion_prob_forward)
        pl = max(0.0, self.motion_prob_left)
        pr = max(0.0, self.motion_prob_right)
        s = pf + pl + pr
        if s <= 0.0:
            return (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
        return (pf / s, pl / s, pr / s)

    def normalized_motion_probs(self) -> tuple[float, float, float]:
        """``(P_forward, P_left, P_right)`` after non-negative renormalization."""

        return self._normalized_motion_triple()

    @staticmethod
    def _merge_distribution(
        parts: list[tuple[MdpState, float, bool]],
    ) -> list[tuple[MdpState, float, bool]]:
        acc: dict[tuple[MdpState, bool], float] = {}
        for next_state, prob, hit in parts:
            key = (next_state, hit)
            acc[key] = acc.get(key, 0.0) + prob
        return [(s, p, h) for (s, h), p in acc.items()]

    def _try_move_grid(self, cell: GridCell, d_row: int, d_col: int) -> tuple[GridCell, bool]:
        target = GridCell(cell.row + d_row, cell.col + d_col)
        if self.is_traversable(target):
            return target, False
        return cell, True

    def transition_distribution(
        self,
        state: MdpState,
        action: MdpAction,
    ) -> list[tuple[MdpState, float, bool]]:
        """Distribution over ``(next_state, hit_wall)`` for Bellman expectation."""

        if self.oriented_mdp:
            assert isinstance(state, PoseState) and isinstance(action, OrientedAction)
            if action in (OrientedAction.TURN_LEFT, OrientedAction.TURN_RIGHT):
                nxt, hit = self._transition_oriented(state, action)
                return [(nxt, 1.0, hit)]
            return self._oriented_forward_distribution(state)
        assert isinstance(state, GridCell) and isinstance(action, Action)
        return self._cell_translate_distribution(state, action)

    def sample_transition(
        self,
        state: MdpState,
        action: MdpAction,
        rng: random.Random,
    ) -> tuple[MdpState, bool]:
        """Sample one successor from ``transition_distribution``."""

        dist = self.transition_distribution(state, action)
        u = rng.random()
        t = 0.0
        for next_state, prob, hit_wall in dist:
            t += prob
            if u <= t:
                return next_state, hit_wall
        return dist[-1][0], dist[-1][2]

    def transition(self, state: MdpState, action: MdpAction) -> tuple[MdpState, bool]:
        """Stochastic sample (uses global ``random`` if you do not pass a dedicated RNG)."""

        return self.sample_transition(state, action, random)

    def _cell_translate_distribution(
        self,
        cell: GridCell,
        action: Action,
    ) -> list[tuple[GridCell, float, bool]]:
        d_row, d_col = ACTION_DELTAS_RC[action]
        pf, pl, pr = self._normalized_motion_triple()
        l_row, l_col = -d_col, d_row
        r_row, r_col = d_col, -d_row
        c0, hit0 = self._try_move_grid(cell, d_row, d_col)
        c1, hit1 = self._try_move_grid(cell, l_row, l_col)
        c2, hit2 = self._try_move_grid(cell, r_row, r_col)
        raw: list[tuple[MdpState, float, bool]] = [
            (c0, pf, hit0),
            (c1, pl, hit1),
            (c2, pr, hit2),
        ]
        return self._merge_distribution(raw)  # type: ignore[return-value]

    def _oriented_forward_distribution(self, state: PoseState) -> list[tuple[PoseState, float, bool]]:
        h = state.heading
        c = state.cell
        pf, pl, pr = self._normalized_motion_triple()
        fr, fc = HEADING_DELTA_RC[h]
        lr, lc = HEADING_DELTA_RC[h.turn_left()]
        rr, rc = HEADING_DELTA_RC[h.turn_right()]
        raw: list[tuple[MdpState, float, bool]] = []
        for dr, dc, p in ((fr, fc, pf), (lr, lc, pl), (rr, rc, pr)):
            cell2, hit = self._try_move_grid(c, dr, dc)
            raw.append((PoseState(cell2, h), p, hit))
        return self._merge_distribution(raw)  # type: ignore[return-value]

    def reward_transition(
        self,
        state: MdpState,
        action: MdpAction,
        next_state: MdpState,
        hit_wall: bool,
    ) -> float:
        if self.oriented_mdp:
            return self._reward_oriented(state, action, next_state, hit_wall)  # type: ignore[arg-type]
        assert isinstance(state, GridCell) and isinstance(next_state, GridCell)
        return self.reward(state, next_state, hit_wall)

    def aggregate_max_v_per_cell(self, values: dict[MdpState, float]) -> dict[GridCell, float]:
        """For oriented MDP, max over headings; for cell MDP pass-through."""

        if not self.oriented_mdp:
            return values  # type: ignore[return-value]
        out: dict[GridCell, float] = {}
        for pose, value in values.items():
            assert isinstance(pose, PoseState)
            cell = pose.cell
            prev = out.get(cell)
            out[cell] = value if prev is None else max(prev, value)
        return out

    def bellman_action_value(
        self,
        state: MdpState,
        action: MdpAction,
        values: dict[MdpState, float],
        gamma: float,
    ) -> float:
        """Expected one-step Q(s, a) (matches stochastic value iteration backup)."""

        total = 0.0
        for next_state, prob, hit_wall in self.transition_distribution(state, action):
            reward = self.reward_transition(state, action, next_state, hit_wall)
            total += prob * (reward + gamma * values.get(next_state, 0.0))
        return total

    def aggregated_policy_per_cell(
        self,
        values: dict[MdpState, float],
        gamma: float,
    ) -> dict[GridCell, MdpAction | None]:
        """Per-cell action for oriented MDP: ``argmax_a max_h Q((cell,h), a)``.

        At ``start`` only ``start_heading`` is used (known initial pose). Elsewhere the action is
        the best achievable under **some** heading at that cell.
        """

        if not self.oriented_mdp:
            raise ValueError("aggregated_policy_per_cell is only defined for oriented_mdp")
        actions = list(OrientedAction)
        result: dict[GridCell, MdpAction | None] = {}
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
        action: MdpAction | None,
        values: dict[MdpState, float],
        gamma: float,
    ) -> Heading:
        """Heading **only** for drawing ``F`` / ``L`` / ``R`` (argmax_h Q for the chosen action)."""

        if not self.oriented_mdp or action is None or not isinstance(action, OrientedAction):
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
        cell_policy: dict[GridCell, MdpAction | None],
        values: dict[MdpState, float],
        gamma: float,
    ) -> dict[GridCell, Heading]:
        """Map each cell with a non-``None`` oriented action to a draw heading."""

        if not self.oriented_mdp:
            return {}
        out: dict[GridCell, Heading] = {}
        for cell, act in cell_policy.items():
            if act is None or not isinstance(act, OrientedAction):
                continue
            out[cell] = self.display_heading_for_cell_action(cell, act, values, gamma)
        return out

    def next_state(self, cell: GridCell, action: Action) -> tuple[GridCell, bool]:
        """Cell MDP: move in the chosen cardinal direction."""

        d_row, d_col = ACTION_DELTAS_RC[action]
        target = GridCell(cell.row + d_row, cell.col + d_col)
        if self.is_traversable(target):
            return target, False
        return cell, True

    def reward(self, current: GridCell, next_cell: GridCell, hit_wall: bool) -> float:
        """Cell MDP reward (also used for FORWARD in oriented mode)."""

        if next_cell == self.goal:
            return self.goal_reward
        if hit_wall:
            return self.collision_penalty

        old_dist = self._manhattan_to_goal(current)
        new_dist = self._manhattan_to_goal(next_cell)
        if new_dist > old_dist:
            return self.away_from_goal_penalty
        return self.step_cost

    def _transition_oriented(
        self,
        state: PoseState,
        action: OrientedAction,
    ) -> tuple[PoseState, bool]:
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
        raise AssertionError(f"unknown oriented action: {action}")

    def _reward_oriented(
        self,
        state: PoseState,
        action: OrientedAction,
        next_state: PoseState,
        hit_wall: bool,
    ) -> float:
        if action in (OrientedAction.TURN_LEFT, OrientedAction.TURN_RIGHT):
            return self.turn_90_reward
        return self.reward(state.cell, next_state.cell, hit_wall)

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
