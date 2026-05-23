#!/usr/bin/env python3
"""
AlphaBot2 – Robust Line Follower + Junction Handler
=====================================================
Track: solid black line on white background.
Sensors [fl, cl, c, cr, fr] → 1 = black, 0 = white.

State machine (junction + pause):
──────────────────────────────────────────────────────────────────────────────
  PHASE_FOLLOW        Proportional controller keeps robot centred on [0,1,1,1,0].
                      Transition → PHASE_JUNCTION_ALIGN
                      when  4+ consecutive sensors on black (not scattered 4)

  PHASE_JUNCTION_ALIGN  Creep through cross until cleared.
                        Transition → PHASE_JUNCTION_TURN

  PHASE_JUNCTION_TURN   Rotate until cl, c, cr all black; ignore that for the first
                        JUNCTION_COOLDOWN_SEARCH so the robot can leave the old line.
                        Transition → PHASE_FOLLOW

  PAUSE_BETWEEN_PHASES_SEC (0.5 s) stop between align→turn and turn→follow.
                        No pause on follow→align.

──────────────────────────────────────────────────────────────────────────────
Tune these constants:
  LINEAR_FOLLOW   – normal cruising speed
  LINEAR_ALIGN    – slow creep through the horizontal bar
  ANGULAR_TURN              – junction turn spin rate (rad/s)
  JUNCTION_COOLDOWN_SEARCH  – ignore centre [1,1,1] this long after turn starts
  JUNCTION_COOLDOWN_FORWARD – ignore new junction detect after resuming follow
  KP              – proportional gain for line-follow angular correction
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
# ──────────────────────────────────────────────────────────────────────────
LINEAR_FOLLOW = 0.20           # line follow
LINEAR_ALIGN  = 0.17          # creep through cross bar
ANGULAR_TURN  = 4.0            # junction turn in place

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
PHASE_JUNCTION_TURN  = 2
PHASE_PAUSE          = 4

PHASE_LABELS = {
    PHASE_FOLLOW:         'FOLLOW',
    PHASE_JUNCTION_ALIGN: 'ALIGN',
    PHASE_JUNCTION_TURN:  'TURN',
    PHASE_PAUSE:          'PAUSE',
}

# ──────────────────────────────────────────────────────────────────────────
# Junction
# ──────────────────────────────────────────────────────────────────────────
PAUSE_BETWEEN_PHASES_SEC    = 0.5
JUNCTION_MIN_STRAIGHT_BLACK = 4     # consecutive blacks (not e.g. [1,1,0,1,1])
JUNCTION_COOLDOWN_SEARCH    = 0.45   # s – ignore centre [1,1,1] after turn starts
JUNCTION_COOLDOWN_FORWARD   = 1.0   # s – ignore junction detect after follow resumes


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

    def __init__(self):
        super().__init__('alphabot')

        self.phase    = PHASE_FOLLOW
        self.turn_dir = None   
        self.last_error = 0.0
        self.last_twist = Twist()
        self.pause_until = 0.0
        self.turn_search_allowed_after = 0.0
        self.junction_ignore_until = 0.0
        self._junction_cooldown_pending = False
        self._after_pause_phase = PHASE_FOLLOW
        self._pending_twist_after_pause = None

        self.pub = self.create_publisher(Twist, '/alphabot2/cmd_vel', 10)
        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/line_sensors',
            self.sensor_callback,
            10
        )

        self.get_logger().info('AlphaBot2 started – line following active')
        self.get_logger().info(
            f'  KP={KP}  LINEAR_FOLLOW={LINEAR_FOLLOW}  '
            f'LINEAR_ALIGN={LINEAR_ALIGN}  ANGULAR_TURN={ANGULAR_TURN}  '
            f'COOLDOWN_SEARCH={JUNCTION_COOLDOWN_SEARCH}  '
            f'COOLDOWN_FORWARD={JUNCTION_COOLDOWN_FORWARD}')


    def _schedule_phase_pause(self, next_phase: int, *, twist_after=None, label: str = '') -> None:
        self.phase = PHASE_PAUSE
        self._after_pause_phase = next_phase
        self._pending_twist_after_pause = twist_after
        self.pause_until = time.monotonic() + PAUSE_BETWEEN_PHASES_SEC
        self.pub.publish(Twist())
        next_label = label or _state_label(next_phase)
        self.get_logger().info(
            f'PAUSE {PAUSE_BETWEEN_PHASES_SEC}s → {next_label}'
        )

    def _handle_phase_pause(self) -> bool:
        """Hold stop during pause. Return True if still pausing."""
        if self.phase != PHASE_PAUSE:
            return False
        self.pub.publish(Twist())
        if time.monotonic() < self.pause_until:
            return True
        self.phase = self._after_pause_phase
        if self._pending_twist_after_pause is not None:
            self._publish(self._pending_twist_after_pause)
            self._pending_twist_after_pause = None
        self.get_logger().info(f'PAUSE done → {_state_label(self.phase)}')
        if self.phase == PHASE_JUNCTION_TURN:
            self._arm_junction_turn()
        if self.phase == PHASE_FOLLOW and self._junction_cooldown_pending:
            self.junction_ignore_until = (
                time.monotonic() + JUNCTION_COOLDOWN_FORWARD
            )
            self._junction_cooldown_pending = False
            self.get_logger().info(
                f'Junction cooldown forward {JUNCTION_COOLDOWN_FORWARD}s '
                f'(no re-detect)'
            )
        return False

    def _arm_junction_turn(self) -> None:
        self.turn_search_allowed_after = (
            time.monotonic() + JUNCTION_COOLDOWN_SEARCH
        )
        ang = ANGULAR_TURN if self.turn_dir == 'left' else -ANGULAR_TURN
        twist = Twist()
        twist.angular.z = ang
        self._publish(twist)
        self.get_logger().info(
            f'  junction turn [{self.turn_dir}] @ {ang:+.1f} rad/s '
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

        # ── PHASE 0: Proportional line following ─────────────────────────── #
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
                self.turn_dir = random.choice(['right'])
                self.phase = PHASE_JUNCTION_ALIGN
                self.get_logger().info(
                    f'JUNCTION detected straight_black={straight_black} '
                    f'→ ALIGN (will turn {self.turn_dir})')
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

            # Weighted proportional error
            # error > 0 → line to the right → angular.z negative (turn right)
            # error < 0 → line to the left  → angular.z positive (turn left)
           
            # Replace the P controller block in PHASE_FOLLOW
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

        # ── PHASE 1: Junction align – creep straight through horizontal bar ─ #
        if self.phase == PHASE_JUNCTION_ALIGN:

            # Exit condition: outer sensors have cleared the bar AND
            # at most 2 sensors are black (only the vertical line remains)
            cleared = (fl == 0 and fr == 0) or (total_black <= 3)

            if cleared:
                self.get_logger().info(
                    f'Pivot over centre – junction turn next [{self.turn_dir}]'
                )
                self._schedule_phase_pause(
                    PHASE_JUNCTION_TURN, label='junction turn'
                )
            else:
                # Still crossing the horizontal bar – creep straight
                twist.linear.x  = LINEAR_ALIGN
                twist.angular.z = 0.0
                self._publish(twist)
                self.get_logger().info(
                    f'  aligning – sensors={s}  sum={total_black}')
            return

        # ── PHASE 2: Junction turn – spin until centre trio (after cooldown) ─ #
        if self.phase == PHASE_JUNCTION_TURN:
            if self.turn_search_allowed_after == 0.0:
                self._arm_junction_turn()

            ang = ANGULAR_TURN if self.turn_dir == 'left' else -ANGULAR_TURN
            centre_trio_on_line = cl == 1 and c == 1 and cr == 1 and (fl == 0 and fr == 0)
            search_allowed = time.monotonic() >= self.turn_search_allowed_after

            if centre_trio_on_line and search_allowed:
                self.turn_search_allowed_after = 0.0
                self.get_logger().info(
                    'Centre trio [1,1,1] – line follow next'
                )
                self._junction_cooldown_pending = True
                self._schedule_phase_pause(
                    PHASE_FOLLOW, label='line follow'
                )
                return

            twist.linear.x = 0.0
            twist.angular.z = ang
            self._publish(twist)
            if not search_allowed:
                remaining = self.turn_search_allowed_after - time.monotonic()
                self.get_logger().info(
                    f'  turn [{self.turn_dir}]  ignore line '
                    f'{remaining:.2f}s left')
            else:
                self.get_logger().info(
                    f'  turn [{self.turn_dir}]  centre={s[1:4]}  sensors={s}')
            return

    # ──────────────────────────────────────────────────────────────────────── #
        
    def _publish(self, twist: Twist):
            self.pub.publish(twist)
            self.get_logger().info(
                f'  CMD → lin={twist.linear.x:.3f}  ang={twist.angular.z:.3f}')
            if self.phase == PHASE_FOLLOW:
                self.last_twist = twist

# ─────────────────────────────────────────────────────────────────────────── #

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