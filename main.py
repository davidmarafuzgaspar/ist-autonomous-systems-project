#!/usr/bin/env python3
"""
AlphaBot2 – Line follower with junction handling
================================================
Track: black line on white. Sensors [fl, cl, c, cr, fr] → 1 = black, 0 = white.

Junction pipeline (each cycle):
──────────────────────────────────────────────────────────────────────────────
  FOLLOW  →  ALIGN  →  SEARCH  →  FOLLOW

  FOLLOW   Track the line (weighted P+D). On 4+ consecutive blacks → ALIGN.
           No pause on this transition.

  ALIGN    Creep forward through the cross bar until cleared → pause
           (JUNCTION_PAUSE_AFTER_ALIGN) → SEARCH.

  SEARCH   Rotate in place until the new branch is centred ([1,1,1] on cl,c,cr).
           Ignore that pattern for JUNCTION_COOLDOWN_SEARCH first (leave old line).
           Then pause (JUNCTION_PAUSE_AFTER_SEARCH) → FOLLOW.

  FOLLOW   After SEARCH, JUNCTION_COOLDOWN_FORWARD suppresses false junctions.

──────────────────────────────────────────────────────────────────────────────
Tune: LINEAR_FOLLOW, LINEAR_ALIGN, ANGULAR_SEARCH, KP/KD, junction pauses & cooldowns.
"""

from __future__ import annotations

import time
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

from solver import (
    ACTION_STRAIGHT,
    ACTION_TURN_AROUND,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    MODE_DYNAMIC,
    MODE_MODEL_BASED,
    MODE_MODEL_FREE,
    Solver,
)
from world import HEADING_DELTA, GridWorld

# ──────────────────────────────────────────────────────────────────────────
# Sensors
# ──────────────────────────────────────────────────────────────────────────
WHITE_THRESHOLD = 700          # raw reading >= this → white (0)

# ──────────────────────────────────────────────────────────────────────────
# Motion  (cmd_vel: linear.x m/s, angular.z rad/s)
# ─────────────────────────────────────────────────────────────────────────
LINEAR_FOLLOW = 0.17           # line follow
LINEAR_ALIGN  = 0.12          # creep through cross bar
ANGULAR_SEARCH = 4.0           # rad/s – spin during SEARCH phase

# ──────────────────────────────────────────────────────────────────────────
# Line follow  (weighted P + D; sensors fl … fr)
# ──────────────────────────────────────────────────────────────────────────
KP = 1.05                           # proportional gain 
KD = 1.60                           # derivative gain 
SENSOR_WEIGHTS = [-2, -1, 0, 1, 2]  # +error → line right → turn right

# ──────────────────────────────────────────────────────────────────────────
# Phases
# ──────────────────────────────────────────────────────────────────────────
PHASE_FOLLOW            = 0
PHASE_JUNCTION_ALIGN    = 1
PHASE_JUNCTION_SEARCH   = 2
PHASE_PAUSE             = 3
PHASE_GOAL              = 4

PHASE_LABELS = {
    PHASE_FOLLOW:           'FOLLOW',
    PHASE_JUNCTION_ALIGN:   'ALIGN',
    PHASE_JUNCTION_SEARCH:  'SEARCH',
    PHASE_PAUSE:            'PAUSE',
    PHASE_GOAL:             'GOAL',
}

# ──────────────────────────────────────────────────────────────────────────
# Junction
# ──────────────────────────────────────────────────────────────────────────
JUNCTION_PAUSE_AFTER_ALIGN  = 0.5   # s – stop after ALIGN, before SEARCH
JUNCTION_PAUSE_AFTER_SEARCH = 0.5   # s – stop after SEARCH, before FOLLOW
JUNCTION_COOLDOWN_SEARCH    = 0.5   # s – ignore [1,1,1] at start of SEARCH
JUNCTION_COOLDOWN_FORWARD   = 1.0   # s – ignore junction detect after follow resumes
JUNCTION_MIN_STRAIGHT_BLACK = 4     # consecutive blacks (not e.g. [1,1,0,1,1])

# ──────────────────────────────────────────────────────────────────────────
# Grid world  (0 = intersection, 1 = obstacle; row 0 = north)
# ──────────────────────────────────────────────────────────────────────────
MAP: list[list[int]] = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
]
START: tuple[int, int] = (0, 0)       # (row, col)
START_HEADING: str = "N"         # N | E | S | W
GOAL: tuple[int, int] = (2, 4)        # (row, col)

# ──────────────────────────────────────────────────────────────────────────
# Solver mode
# ──────────────────────────────────────────────────────────────────────────
# model_based: full MAP + value iteration, no runtime IR updates
# model_free:  full MAP + Q-learning offline, no runtime IR updates
# dynamic:     MAP as belief (0=free/unknown, 1=known obstacle); Q-learning
#              at startup; IR discovery + value-iteration replan when blocking
INITIAL_OBSTACLE_SAMPLE_TIMEOUT = 0.5
#SOLVER_MODE = MODE_MODEL_BASED
#SOLVER_MODE = MODE_MODEL_FREE
SOLVER_MODE = MODE_DYNAMIC

# ──────────────────────────────────────────────────────────────────────────
# Debug logging
# ──────────────────────────────────────────────────────────────────────────
PRINT_MAP = True                      # ASCII grid after startup / junction / align
PRINT_CONTROL_LOGS = False            # per-tick sensors, phases, CMD, P+D detail


class LineSensors(NamedTuple):
    """Binary line-sensor snapshot: [fl, cl, c, cr, fr]; 1 = black."""

    binary: list[int]
    fl: int
    cl: int
    c: int
    cr: int
    fr: int
    total_black: int

    @classmethod
    def from_msg(cls, msg: Int32MultiArray) -> LineSensors | None:
        raw = list(msg.data)
        if len(raw) < 5:
            return None
        binary = [0 if v >= WHITE_THRESHOLD else 1 for v in raw]
        fl, cl, c, cr, fr = binary
        return cls(binary, fl, cl, c, cr, fr, sum(binary))


def max_consecutive_black(binary: list[int]) -> int:
    """Longest run of adjacent sensors on black."""
    best = run = 0
    for v in binary:
        if v == 1:
            run += 1
            best = max(best, run)
        else:
            run = 0
    return best


def _state_label(phase: int) -> str:
    return PHASE_LABELS.get(phase, f'unknown({phase})')


class CrossHandler(Node):
    """Line follower with junction handling for AlphaBot2 (5 reflective sensors)."""

    def __init__(self):
        super().__init__('alphabot')

        # --- State machine ---
        self.phase = PHASE_FOLLOW
        self.search_dir = None        

        # --- Line-follow controller ---
        self.last_error = 0.0
        self.last_twist = Twist()

        # --- SEARCH phase timing ---
        self.search_line_allowed_after = 0.0
        self.junction_ignore_until = 0.0
        self._junction_cooldown_pending = False
        self._turn_around_second_leg_pending = False

        # --- Inter-phase pause ---
        self.pause_until = 0.0
        self._after_pause_phase = PHASE_FOLLOW
        self._pending_twist_after_pause = None

        # --- Obstacle sensing (sampled continuously, interpreted only at PAUSE completion) ---
        self._left_obstacle = False
        self._right_obstacle = False
        self._have_obstacle_sample = False
        self.solver = None

        # --- Discrete grid pose (intersections) ---
        initial_map = self._build_initial_map_for_solver()
        self.world = GridWorld.from_matrix(initial_map, START, START_HEADING, GOAL)

        # --- ROS Controller Interfaces ---
        self.pub = self.create_publisher(
            Twist,
            '/alphabot2/cmd_vel',
            10,
        )
        self.create_subscription(
            Obstacle,
            '/alphabot2/ir_obstacles_sensors',
            self.obstacle_callback,
            10,
        )
        self._prime_initial_obstacle_map_from_ir()

        self.solver = Solver(self.world, mode=SOLVER_MODE)
        self.solver.train(log_fn=self.get_logger().info)

        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/ir_line_sensors',
            self.sensor_callback,
            10,
        )

        # --- Logger Information ---
        self.get_logger().info('AlphaBot2 cross handler ready (line follow + junctions)')
        self.get_logger().info(
            f'Solver mode={SOLVER_MODE} active_planner={self.solver.active_planner}'
        )
        self.get_logger().info(f'Grid pose: {self.world.pose_str()}  goal={GOAL}')
        self._log_solver_report()
        self._log_map()
        if self.world.is_at_goal():
            self._handle_goal_reached()
            return
        if self._maybe_start_with_solver_turn():
            return
        self._log_control(
            f'Debug: PRINT_MAP={PRINT_MAP} PRINT_CONTROL_LOGS={PRINT_CONTROL_LOGS}'
        )
        self._log_control(
            f'Tuning: KP={KP} KD={KD} '
            f'LINEAR_FOLLOW={LINEAR_FOLLOW} LINEAR_ALIGN={LINEAR_ALIGN} '
            f'ANGULAR_SEARCH={ANGULAR_SEARCH} '
            f'JUNCTION_PAUSE_AFTER_ALIGN={JUNCTION_PAUSE_AFTER_ALIGN}s '
            f'JUNCTION_PAUSE_AFTER_SEARCH={JUNCTION_PAUSE_AFTER_SEARCH}s '
            f'JUNCTION_COOLDOWN_SEARCH={JUNCTION_COOLDOWN_SEARCH}s '
            f'JUNCTION_COOLDOWN_FORWARD={JUNCTION_COOLDOWN_FORWARD}s '
        )

    def _build_initial_map_for_solver(self) -> list[list[int]]:
        """All modes use global MAP (dynamic: 0=free/unknown belief, 1=known obstacle)."""
        return [list(row) for row in MAP]

    def obstacle_callback(self, msg: Obstacle) -> None:
        """Cache latest obstacle sample; interpretation is deferred to PAUSE completion."""
        self._left_obstacle = bool(msg.left_obstacle)
        self._right_obstacle = bool(msg.right_obstacle)
        self._have_obstacle_sample = True

    def _prime_initial_obstacle_map_from_ir(self) -> None:
        """Before solver training, check obstacle ahead from initial heading (dynamic only)."""
        if SOLVER_MODE != MODE_DYNAMIC:
            return
        deadline = time.monotonic() + INITIAL_OBSTACLE_SAMPLE_TIMEOUT
        while not self._have_obstacle_sample and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._have_obstacle_sample:
            self.get_logger().info('Initial obstacle check: no IR sample before solver training')
            return
        if not (self._left_obstacle or self._right_obstacle):
            self.get_logger().info('Initial obstacle check: no obstacle ahead in initial heading')
            return

        dr, dc = HEADING_DELTA[self.world.heading]
        obstacle_row = self.world.row + dr
        obstacle_col = self.world.col + dc
        if not self.world.is_in_bounds(obstacle_row, obstacle_col):
            self.get_logger().info(
                f'Initial obstacle check: obstacle maps out of bounds -> ({obstacle_row},{obstacle_col})'
            )
            return
        changed = self.world.mark_obstacle(obstacle_row, obstacle_col)
        if changed:
            self.get_logger().info(
                f'Initial obstacle check: marked obstacle at ({obstacle_row},{obstacle_col}) '
                f'from heading {self.world.heading.name}'
            )
        else:
            self.get_logger().info(
                f'Initial obstacle check: cell ({obstacle_row},{obstacle_col}) already blocked'
            )

    def _maybe_process_obstacle_sample(self) -> bool:
        """Interpret obstacle info only after a movement action has completed.

        Returns:
            True when updated policy should be applied immediately via SEARCH.
        """
        if SOLVER_MODE != MODE_DYNAMIC:
            return False
        self.get_logger().info(
            'Obstacle check: evaluating IR obstacle sample after movement completion'
        )
        if not self._have_obstacle_sample:
            self.get_logger().info('Obstacle check: no IR sample available yet')
            return False
        obstacle_ahead = self._left_obstacle or self._right_obstacle
        if not obstacle_ahead:
            self.get_logger().info('Obstacle check: no obstacle detected')
            return False

        self.get_logger().info(
            f'Obstacle check: obstacle detected (left={self._left_obstacle}, right={self._right_obstacle})'
        )
        dr, dc = HEADING_DELTA[self.world.heading]
        obstacle_row = self.world.row + dr
        obstacle_col = self.world.col + dc
        if not self.world.is_in_bounds(obstacle_row, obstacle_col):
            self.get_logger().info(
                f'Obstacle check: detected obstacle maps out of bounds -> ({obstacle_row},{obstacle_col})'
            )
            return False
        if self.world.is_obstacle(obstacle_row, obstacle_col):
            self.get_logger().info(
                f'Obstacle check: cell ({obstacle_row},{obstacle_col}) already known obstacle'
            )
            return False

        blocks_plan = self.solver.path_contains_cell(
            obstacle_row,
            obstacle_col,
            start_row=self.world.row,
            start_col=self.world.col,
            start_heading=self.world.heading,
        )
        changed = self.solver.update_discovered_obstacle(obstacle_row, obstacle_col)
        if not changed:
            self.get_logger().info(
                f'Obstacle check: could not update map for cell ({obstacle_row},{obstacle_col})'
            )
            return False

        self.get_logger().info(
            f'Discovered obstacle at ({obstacle_row},{obstacle_col}) while heading {self.world.heading.name}'
        )
        if blocks_plan:
            self.get_logger().info('Obstacle blocks current planned path -> replanning')
            self.solver.replan_from_state(
                self.world.row,
                self.world.col,
                self.world.heading,
                log_fn=self.get_logger().info,
            )
            self._log_solver_report()

            action_now = self.solver.get_action(
                self.world.row,
                self.world.col,
                self.world.heading,
            )
            if action_now != ACTION_STRAIGHT:
                self.get_logger().info(
                    f'Applying updated policy immediately -> SEARCH action {action_now}'
                )
                self.search_dir = action_now
                self._turn_around_second_leg_pending = False
                return True
        else:
            self.get_logger().info('Obstacle does not block current plan -> keep policy')
        return False

    def _log_solver_report(self) -> None:
        """Emit a readable post-training policy summary."""
        report = self.solver.format_policy_report()
        for line in report.splitlines():
            self.get_logger().info(line)

    def _maybe_start_with_solver_turn(self) -> bool:
        """Immediately apply startup turn policy at initial cell, if needed."""
        action = self.solver.get_action(self.world.row, self.world.col, self.world.heading)
        if action == ACTION_STRAIGHT:
            return False

        self._turn_around_second_leg_pending = False
        self.search_dir = action
        self.phase = PHASE_JUNCTION_SEARCH
        self._arm_junction_search()
        return True

    def _log_control(self, message: str) -> None:
        """Phase/sensor/CMD detail when PRINT_CONTROL_LOGS is enabled."""
        if PRINT_CONTROL_LOGS:
            self.get_logger().info(message)

    def _log_map(self) -> None:
        """Print grid when PRINT_MAP is enabled."""
        if PRINT_MAP:
            self.world.print_map(self.get_logger())

    def _handle_goal_reached(self) -> None:
        """Stop the robot and report mission success."""
        self.phase = PHASE_GOAL
        self._publish_zero_cmd()
        self.get_logger().info('')
        self.get_logger().info('========================================')
        self.get_logger().info(
            f'SUCCESS: goal {GOAL} reached — {self.world.pose_str()}'
        )
        self.get_logger().info('========================================')
        self.get_logger().info('')
        self._log_map()

    def _schedule_phase_pause(
        self,
        next_phase: int,
        *,
        pause_sec: float,
        twist_after: Twist | None = None,
        label: str = '',
    ) -> None:
        """Begin a timed full stop, then resume at ``next_phase``.

        Used after ALIGN (before SEARCH) and after SEARCH (before FOLLOW).
        Zero ``cmd_vel`` is published immediately and on each sensor tick
        until ``pause_sec`` expires.

        Args:
            next_phase: Phase constant to enter when the pause completes.
            pause_sec: Hold duration in seconds (``time.monotonic()``).
            twist_after: Optional cmd_vel published once after the pause ends.
            label: Log label; defaults to the target phase name.
        """
        self.phase = PHASE_PAUSE
        self._after_pause_phase = next_phase
        self._pending_twist_after_pause = twist_after
        self.pause_until = time.monotonic() + pause_sec

        self._publish_zero_cmd()
        dest = label or _state_label(next_phase)
        self._log_control(f'PAUSE {pause_sec:.2f}s → {dest}')

    def _handle_phase_pause(self) -> bool:
        """Run the pause state on each line-sensor callback.

        Returns:
            True if still pausing (caller should skip other phase logic).
            False if not in PHASE_PAUSE, or if the pause just completed.
        """
        if self.phase != PHASE_PAUSE:
            return False

        self._publish_zero_cmd()

        if time.monotonic() < self.pause_until:
            return True

        self._complete_phase_pause()
        return False

    def _publish_zero_cmd(self) -> None:
        """Brake via motion_driver: zero linear and angular velocity."""
        self.pub.publish(Twist())

    def _should_run_obstacle_check_after_pause(self, pause_target_phase: int) -> bool:
        """When to sample IR after a pause (dynamic mode only).

        STRAIGHT skips the ALIGN→SEARCH pause (second pause before FOLLOW still checks).
        Turns and U-turn still check when entering SEARCH (after ALIGN / leg 1) and FOLLOW.
        """
        if SOLVER_MODE != MODE_DYNAMIC:
            return False
        if pause_target_phase == PHASE_FOLLOW:
            return True
        if pause_target_phase == PHASE_JUNCTION_SEARCH:
            return self.search_dir != ACTION_STRAIGHT
        return False

    def _complete_phase_pause(self) -> None:
        """Leave PHASE_PAUSE: advance phase and run transition hooks."""
        pause_target = self._after_pause_phase
        self.phase = pause_target
        apply_search_now = False
        if self._should_run_obstacle_check_after_pause(pause_target):
            apply_search_now = self._maybe_process_obstacle_sample()
        if apply_search_now:
            self.phase = PHASE_JUNCTION_SEARCH

        pending = self._pending_twist_after_pause
        if pending is not None:
            self._publish(pending)
            self._pending_twist_after_pause = None

        self._log_control(f'PAUSE done → {_state_label(self.phase)}')

        if self.phase == PHASE_JUNCTION_SEARCH:
            self._arm_junction_search()
            return

        if self.phase == PHASE_FOLLOW and self._junction_cooldown_pending:
            self.junction_ignore_until = (
                time.monotonic() + JUNCTION_COOLDOWN_FORWARD
            )
            self._junction_cooldown_pending = False
            self._log_control(
                f'Junction cooldown forward {JUNCTION_COOLDOWN_FORWARD:.2f}s '
                f'(no re-detect)'
            )

    def _arm_junction_search(self) -> None:
        """Start SEARCH: spin in place; defer line-acquire until cooldown elapses."""
        if self.search_dir == ACTION_STRAIGHT:
            self._turn_around_second_leg_pending = False
            self.search_line_allowed_after = 0.0
            self._log_control('  SEARCH [straight] -> no rotation, resume FOLLOW')
            self._junction_cooldown_pending = True
            self._schedule_phase_pause(
                PHASE_FOLLOW,
                pause_sec=JUNCTION_PAUSE_AFTER_SEARCH,
                label='line follow',
            )
            return

        self.search_line_allowed_after = (
            time.monotonic() + JUNCTION_COOLDOWN_SEARCH
        )
        ang = self._search_angular_velocity()
        twist = Twist()
        twist.angular.z = ang
        self._publish(twist)
        self._log_control(
            f'  SEARCH [{self.search_dir}] @ {ang:+.1f} rad/s '
            f'(ignore [1,1,1] for {JUNCTION_COOLDOWN_SEARCH}s)'
        )

    # ── Pipeline: sensor ingress and phase dispatch ──────────────────────── #

    def sensor_callback(self, msg: Int32MultiArray) -> None:
        """Main control loop entry: one update per line-sensor message."""
        sensors = LineSensors.from_msg(msg)
        if sensors is None:
            return

        self._log_control(
            f'{_state_label(self.phase)}  binary={sensors.binary}  '
            f'sum={sensors.total_black}'
        )

        if self.phase == PHASE_GOAL:
            self._publish_zero_cmd()
            return

        if self._handle_phase_pause():
            return

        if self.phase == PHASE_FOLLOW:
            self._run_follow(sensors)
        elif self.phase == PHASE_JUNCTION_ALIGN:
            self._run_align(sensors)
        elif self.phase == PHASE_JUNCTION_SEARCH:
            self._run_search(sensors)

    def _run_follow(self, sensors: LineSensors) -> None:
        """FOLLOW: P+D tracking; junction detect → ALIGN; line-lost hold."""
        s = sensors.binary

        straight_black = max_consecutive_black(s)
        on_cooldown = time.monotonic() < self.junction_ignore_until
        junction = (
            not on_cooldown
            and straight_black >= JUNCTION_MIN_STRAIGHT_BLACK
        )

        if junction:
            self.last_twist = Twist()
            if not self.world.step_to_next_intersection():
                action = self.solver.get_action(
                    self.world.row, self.world.col, self.world.heading
                )
                self.search_dir = action
                self._turn_around_second_leg_pending = False
                if action == ACTION_STRAIGHT:
                    self.get_logger().error(
                        f'Grid: cannot advance from {self.world.pose_str()} '
                        f'along heading (blocked or out of bounds)'
                    )
                    return
                else:
                    self.get_logger().info(
                        'Grid: forward edge blocked at this junction; '
                        'switching directly to SEARCH turn-in-place'
                    )
                    self.phase = PHASE_JUNCTION_SEARCH
                    self._arm_junction_search()
                    return
            elif self.world.is_at_goal():
                self._handle_goal_reached()
                return
            else:
                self._log_map()
                action = self.solver.get_action(
                    self.world.row, self.world.col, self.world.heading
                )
                self.search_dir = action
                self._turn_around_second_leg_pending = False
            self.phase = PHASE_JUNCTION_ALIGN
            self._log_control(
                f'JUNCTION straight_black={straight_black} '
                f'→ ALIGN (will search {self.search_dir})'
            )
            twist = Twist()
            twist.linear.x = LINEAR_ALIGN
            twist.angular.z = 0.0
            self._publish(twist)
            return

        if sensors.total_black == 0:
            self.pub.publish(self.last_twist)
            self.get_logger().warn(
                f'Line lost – persisting last cmd '
                f'lin={self.last_twist.linear.x:.3f}  '
                f'ang={self.last_twist.angular.z:.3f}'
            )
            return

        active = [i for i, v in enumerate(s) if v == 1]
        error = sum(SENSOR_WEIGHTS[i] for i in active) / len(active)
        d_error = error - self.last_error
        self.last_error = error
        correction = (KP * error) + (KD * d_error)

        twist = Twist()
        twist.linear.x = LINEAR_FOLLOW
        twist.angular.z = -correction
        self._publish(twist)
        self._log_control(
            f'  follow  error={error:.2f}  d_error={d_error:.2f}  '
            f'correction={correction:.3f}'
        )

    def _run_align(self, sensors: LineSensors) -> None:
        """ALIGN: creep forward through cross until cleared → pause → SEARCH."""
        cleared = (sensors.fl == 0 and sensors.fr == 0) or (
            sensors.total_black <= 3
        )

        if cleared:
            if self.search_dir == ACTION_STRAIGHT:
                self._log_control('ALIGN done → straight, pause then FOLLOW')
                self._junction_cooldown_pending = True
                self._schedule_phase_pause(
                    PHASE_FOLLOW,
                    pause_sec=JUNCTION_PAUSE_AFTER_SEARCH,
                    label='line follow',
                )
            else:
                self._log_control(
                    f'ALIGN done → SEARCH next [{self.search_dir}]'
                )
                self._schedule_phase_pause(
                    PHASE_JUNCTION_SEARCH,
                    pause_sec=JUNCTION_PAUSE_AFTER_ALIGN,
                    label='search',
                )
            return

        twist = Twist()
        twist.linear.x = LINEAR_ALIGN
        twist.angular.z = 0.0
        self._publish(twist)
        self._log_control(
            f'  aligning – sensors={sensors.binary}  sum={sensors.total_black}'
        )

    def _run_search(self, sensors: LineSensors) -> None:
        """SEARCH: spin until branch acquired; cooldown defers line detection."""
        if self.search_line_allowed_after == 0.0:
            self._arm_junction_search()
            if self.phase != PHASE_JUNCTION_SEARCH:
                return

        ang = self._search_angular_velocity()
        centre_trio_on_line = (
            (sensors.cl == 1 and sensors.c == 1 and sensors.cr == 1)
            or (sensors.fl == 0 and sensors.fr == 0)
        )
        line_acquire_ok = time.monotonic() >= self.search_line_allowed_after

        if centre_trio_on_line and line_acquire_ok:
            self.search_line_allowed_after = 0.0
            if self.search_dir == ACTION_TURN_AROUND and not self._turn_around_second_leg_pending:
                self.world.turn_right()
                self._turn_around_second_leg_pending = True
                self._log_map()
                self._log_control(
                    f'SEARCH turn-around leg 1 done → pause then leg 2  {self.world.pose_str()}'
                )
                self._schedule_phase_pause(
                    PHASE_JUNCTION_SEARCH,
                    pause_sec=JUNCTION_PAUSE_AFTER_SEARCH,
                    label='search turn-around leg 2',
                )
                return

            if self.search_dir == ACTION_TURN_RIGHT:
                self.world.turn_right()
            elif self.search_dir == ACTION_TURN_LEFT:
                self.world.turn_left()
            elif self.search_dir == ACTION_TURN_AROUND:
                self.world.turn_right()
                self._turn_around_second_leg_pending = False
            self._log_map()
            self._log_control(
                f'SEARCH done – on line → FOLLOW  {self.world.pose_str()}'
            )
            self._junction_cooldown_pending = True
            self._schedule_phase_pause(
                PHASE_FOLLOW,
                pause_sec=JUNCTION_PAUSE_AFTER_SEARCH,
                label='line follow',
            )
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = ang
        self._publish(twist)

        if not line_acquire_ok:
            remaining = self.search_line_allowed_after - time.monotonic()
            self._log_control(
                f'  SEARCH [{self.search_dir}]  ignore line '
                f'{remaining:.2f}s left'
            )
        else:
            centre = sensors.binary[1:4]
            self._log_control(
                f'  SEARCH [{self.search_dir}]  centre={centre}  '
                f'sensors={sensors.binary}'
            )

    def _search_angular_velocity(self) -> float:
        """Signed angular.z for in-place SEARCH rotation."""
        if self.search_dir == ACTION_TURN_LEFT:
            return ANGULAR_SEARCH
        if self.search_dir in (ACTION_TURN_RIGHT, ACTION_TURN_AROUND):
            return -ANGULAR_SEARCH
        return 0.0

    def _publish(self, twist: Twist) -> None:
        self.pub.publish(twist)
        self._log_control(
            f'  CMD → lin={twist.linear.x:.3f}  ang={twist.angular.z:.3f}'
        )
        if self.phase == PHASE_FOLLOW:
            self.last_twist = twist


def main(args=None):
    rclpy.init(args=args)
    node = CrossHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()