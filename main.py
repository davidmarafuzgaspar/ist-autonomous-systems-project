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

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import random

# ──────────────────────────────────────────────────────────────────────────
# Sensors
# ──────────────────────────────────────────────────────────────────────────
WHITE_THRESHOLD = 700          # raw reading >= this → white (0)

# ──────────────────────────────────────────────────────────────────────────
# Motion  (cmd_vel: linear.x m/s, angular.z rad/s)
# ─────────────────────────────────────────────────────────────────────────
LINEAR_FOLLOW = 0.20           # line follow
LINEAR_ALIGN  = 0.15          # creep through cross bar
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
PHASE_FOLLOW         = 0
PHASE_JUNCTION_ALIGN = 1
PHASE_JUNCTION_SEARCH = 2
PHASE_PAUSE          = 3

PHASE_LABELS = {
    PHASE_FOLLOW:         'FOLLOW',
    PHASE_JUNCTION_ALIGN: 'ALIGN',
    PHASE_JUNCTION_SEARCH: 'SEARCH',
    PHASE_PAUSE:          'PAUSE',
}

# ──────────────────────────────────────────────────────────────────────────
# Junction
# ──────────────────────────────────────────────────────────────────────────
JUNCTION_PAUSE_AFTER_ALIGN  = 0.5   # s – stop after ALIGN, before SEARCH
JUNCTION_PAUSE_AFTER_SEARCH = 0.5   # s – stop after SEARCH, before FOLLOW
JUNCTION_COOLDOWN_SEARCH    = 0.5   # s – ignore [1,1,1] at start of SEARCH
JUNCTION_COOLDOWN_FORWARD   = 1.0   # s – ignore junction detect after follow resumes
JUNCTION_MIN_STRAIGHT_BLACK = 4     # consecutive blacks (not e.g. [1,1,0,1,1])


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

        # --- Inter-phase pause ---
        self.pause_until = 0.0
        self._after_pause_phase = PHASE_FOLLOW
        self._pending_twist_after_pause = None

        # --- ROS Controller Interfaces ---
        self.pub = self.create_publisher(
            Twist,
            '/alphabot2/cmd_vel',
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/line_sensors',
            self.sensor_callback,
            10,
        )

        # --- Logger Information ---
        self.get_logger().info('AlphaBot2 cross handler ready (line follow + junctions)')
        self.get_logger().info(
            f'Tuning: KP={KP} KD={KD} '
            f'LINEAR_FOLLOW={LINEAR_FOLLOW} LINEAR_ALIGN={LINEAR_ALIGN} '
            f'ANGULAR_SEARCH={ANGULAR_SEARCH} '
            f'JUNCTION_PAUSE_AFTER_ALIGN={JUNCTION_PAUSE_AFTER_ALIGN}s '
            f'JUNCTION_PAUSE_AFTER_SEARCH={JUNCTION_PAUSE_AFTER_SEARCH}s '
            f'JUNCTION_COOLDOWN_SEARCH={JUNCTION_COOLDOWN_SEARCH}s '
            f'JUNCTION_COOLDOWN_FORWARD={JUNCTION_COOLDOWN_FORWARD}s '
        )


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
        self.get_logger().info(f'PAUSE {pause_sec:.2f}s → {dest}')

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

    def _complete_phase_pause(self) -> None:
        """Leave PHASE_PAUSE: advance phase and run transition hooks."""
        self.phase = self._after_pause_phase

        pending = self._pending_twist_after_pause
        if pending is not None:
            self._publish(pending)
            self._pending_twist_after_pause = None

        self.get_logger().info(f'PAUSE done → {_state_label(self.phase)}')

        if self.phase == PHASE_JUNCTION_SEARCH:
            self._arm_junction_search()
            return

        if self.phase == PHASE_FOLLOW and self._junction_cooldown_pending:
            self.junction_ignore_until = (
                time.monotonic() + JUNCTION_COOLDOWN_FORWARD
            )
            self._junction_cooldown_pending = False
            self.get_logger().info(
                f'Junction cooldown forward {JUNCTION_COOLDOWN_FORWARD:.2f}s '
                f'(no re-detect)'
            )

    def _arm_junction_search(self) -> None:
        """Start SEARCH: spin in place; defer line-acquire until cooldown elapses."""
        self.search_line_allowed_after = (
            time.monotonic() + JUNCTION_COOLDOWN_SEARCH
        )
        ang = ANGULAR_SEARCH if self.search_dir == 'left' else -ANGULAR_SEARCH
        twist = Twist()
        twist.angular.z = ang
        self._publish(twist)
        self.get_logger().info(
            f'  SEARCH [{self.search_dir}] @ {ang:+.1f} rad/s '
            f'(ignore [1,1,1] for {JUNCTION_COOLDOWN_SEARCH}s)'
        )

    def sensor_callback(self, msg: Int32MultiArray):
        raw = list(msg.data)
        if len(raw) < 5:
            return

        s = [0 if v >= WHITE_THRESHOLD else 1 for v in raw]
        fl, cl, c, cr, fr = s
        total_black = sum(s)

        self.get_logger().info(
            f'{_state_label(self.phase)}  binary={s}  sum={total_black}')

        if self._handle_phase_pause():
            return

        twist = Twist()

        # ── FOLLOW: proportional line tracking ─────────────────────────────── #
        if self.phase == PHASE_FOLLOW:

            # Junction: 4+ in a row; skip briefly after completing a turn
            straight_black = max_consecutive_black(s)
            on_cooldown = time.monotonic() < self.junction_ignore_until
            junction = (
                not on_cooldown
                and straight_black >= JUNCTION_MIN_STRAIGHT_BLACK
            )

            if junction:
                self.last_twist = Twist()
                self.search_dir = random.choice(['right'])
                self.phase = PHASE_JUNCTION_ALIGN
                self.get_logger().info(
                    f'JUNCTION straight_black={straight_black} '
                    f'→ ALIGN (will search {self.search_dir})'
                )
                twist.linear.x = LINEAR_ALIGN
                twist.angular.z = 0.0
                self._publish(twist)
                return

            if total_black == 0:
                # Lost – persist last known correction instead of stopping
                self.pub.publish(self.last_twist)
                self.get_logger().warn(
                    f'Line lost – persisting last cmd '
                    f'lin={self.last_twist.linear.x:.3f}  '
                    f'ang={self.last_twist.angular.z:.3f}')
                return
           
            active = [i for i, v in enumerate(s) if v == 1]
            error  = sum(SENSOR_WEIGHTS[i] for i in active) / len(active)

            d_error = error - self.last_error   # rate of change of error
            self.last_error = error             # store for next callback

            correction = (KP * error) + (KD * d_error)

            twist.linear.x  = LINEAR_FOLLOW
            twist.angular.z = -correction
            self._publish(twist)
            self.get_logger().info(
                f'  follow  error={error:.2f}  d_error={d_error:.2f}  '
                f'correction={correction:.3f}')
            
            return

        # ── ALIGN: creep straight through cross bar ──────────────────────── #
        if self.phase == PHASE_JUNCTION_ALIGN:

            # Exit condition: outer sensors have cleared the bar AND
            # at most 2 sensors are black (only the vertical line remains)
            cleared = (fl == 0 and fr == 0) or (total_black <= 3)

            if cleared:
                self.get_logger().info(
                    f'ALIGN done → SEARCH next [{self.search_dir}]'
                )
                self._schedule_phase_pause(
                    PHASE_JUNCTION_SEARCH,
                    pause_sec=JUNCTION_PAUSE_AFTER_ALIGN,
                    label='search',
                )
            else:
                # Still crossing the horizontal bar – creep straight
                twist.linear.x  = LINEAR_ALIGN
                twist.angular.z = 0.0
                self._publish(twist)
                self.get_logger().info(
                    f'  aligning – sensors={s}  sum={total_black}')
            return

        # ── SEARCH: rotate until new branch centred (after cooldown) ───────── #
        if self.phase == PHASE_JUNCTION_SEARCH:
            if self.search_line_allowed_after == 0.0:
                self._arm_junction_search()

            ang = ANGULAR_SEARCH if self.search_dir == 'left' else -ANGULAR_SEARCH
            centre_trio_on_line = (cl == 1 and c == 1 and cr == 1) or (fl == 0 and fr == 0)
            line_acquire_ok = time.monotonic() >= self.search_line_allowed_after

            if centre_trio_on_line and line_acquire_ok:
                self.search_line_allowed_after = 0.0
                self.get_logger().info(
                    'SEARCH done – centre on line → FOLLOW'
                )
                self._junction_cooldown_pending = True
                self._schedule_phase_pause(
                    PHASE_FOLLOW,
                    pause_sec=JUNCTION_PAUSE_AFTER_SEARCH,
                    label='line follow',
                )
                return

            twist.linear.x = 0.0
            twist.angular.z = ang
            self._publish(twist)
            if not line_acquire_ok:
                remaining = self.search_line_allowed_after - time.monotonic()
                self.get_logger().info(
                    f'  SEARCH [{self.search_dir}]  ignore line '
                    f'{remaining:.2f}s left'
                )
            else:
                self.get_logger().info(
                    f'  SEARCH [{self.search_dir}]  centre={s[1:4]}  sensors={s}'
                )
            return
        
    def _publish(self, twist: Twist):
            self.pub.publish(twist)
            self.get_logger().info(
                f'  CMD → lin={twist.linear.x:.3f}  ang={twist.angular.z:.3f}')
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