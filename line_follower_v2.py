#!/usr/bin/env python3
"""
AlphaBot2 - Cross/Junction handler ONLY
=========================================
Run this to test and tune cross behaviour independently.

The robot will:
  1. Wait until all 5 sensors see black (cross detected)
  2. Stop and settle
  3. Creep forward until sensors read [0,1,1,1,0] (centred on cross)
  4. Stop and settle again
  5. Execute a sensor-guided 90° turn (randomly left or right)
     – turns until it LEAVES [0,1,1,1,0], then until it FINDS [0,1,1,1,0] again
  6. DONE – robot stopped

Tune ONLY these values:
  LINEAR_CREEP     – forward speed while centring on cross
  ANGULAR_TURN     – rotation speed during turn
  SETTLE_CALLBACKS – callbacks to stay stopped before each action
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import random

# ─────────────────────────────────────────────────────────────────────────── #
WHITE_THRESHOLD  = 500   # >= WHITE  /  < BLACK

LINEAR_CREEP     = 0.05   # m/s – speed while centring on cross
ANGULAR_TURN     = 2.0   # rad/s – rotation speed for 90° turn

LINEAR_FOLLOW    = 0.17  # m/s – normal forward speed
ANGULAR_CORRECT  = 0.85# rad/s – correction strength

SETTLE_CALLBACKS = 3     # callbacks to stay stopped (used for both settles)
# ─────────────────────────────────────────────────────────────────────────── #

PHASE_FOLLOW     = 0   # waiting for a cross to appear
PHASE_SETTLE   = 1   # stopped on cross, letting robot fully halt
PHASE_CENTRE   = 2   # creeping forward until [0,1,1,1,0]
PHASE_SETTLE_2 = 3   # stopped at centre, settling before turn
PHASE_TURN     = 4   # sensor-guided 90° turn
PHASE_DONE     = 5   # finished, robot stopped


class CrossHandler(Node):

    def __init__(self):
        super().__init__('alphabot2_cross_handler')

        self.phase          = PHASE_FOLLOW
        self.phase_count    = 0
        self.turn_dir       = None    # 'left' or 'right', chosen once in PHASE_SETTLE
        self.turn_left_line = False   # True once robot has left [0,1,1,1,0] during turn

        self.pub = self.create_publisher(Twist, '/alphabot2/cmd_vel', 10)
        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/line_sensors',
            self.sensor_callback,
            10
        )

        self.get_logger().info('Cross handler started – waiting for cross...')
        self.get_logger().info(
            f'  LINEAR_CREEP={LINEAR_CREEP}  ANGULAR_TURN={ANGULAR_TURN}  '
            f'SETTLE_CALLBACKS={SETTLE_CALLBACKS}')

    # ──────────────────────────────────────────────────────────────────────── #

    def sensor_callback(self, msg: Int32MultiArray):
        raw = list(msg.data)

        if len(raw) < 5:
            return

        s = [0 if v >= WHITE_THRESHOLD else 1 for v in raw]
        fl, cl, c, cr, fr = s
        total_black = sum(s)

        self.get_logger().info(
            f'phase={self.phase}  binary={s}  sum={total_black}  count={self.phase_count}')

        twist = Twist()

        if self.phase == PHASE_FOLLOW:
            if total_black == 5:
                self.phase       = PHASE_SETTLE
                self.phase_count = 0
                self._publish(Twist())
                return

            twist.linear.x = LINEAR_FOLLOW

           # Drive straight
            if s == [0, 1, 1, 1, 0] or s == [0, 0, 1, 0, 0]:
                twist.angular.z = 0.0

            # Turn RIGHT (Line is on the right side)
            elif s in [[0, 0, 1, 1, 0], [0, 0, 0, 1, 0],[0, 0, 0, 0, 1], [0, 0, 1, 1, 1], [0, 1, 1, 1, 1], [0, 0, 0, 1, 1], [1, 0, 0, 1, 1]]:
                twist.angular.z = -ANGULAR_CORRECT

            # Turn LEFT (Line is on the left side)
            elif s in [[0, 1, 1, 0, 0], [0, 1, 0, 0, 0],[1, 0, 0, 0, 0], [1, 1, 1, 0, 0], [1, 1, 1, 1, 0], [1, 1, 0, 0, 0], [1, 1, 0, 0, 1]]:
                twist.angular.z = ANGULAR_CORRECT

            # Stop ONLY if completely lost [0, 0, 0, 0, 0] or noise
            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0

            self._publish(twist)
            return
    
        # ── PHASE 1: Settle (full stop) ──────────────────────────────────── #
        if self.phase == PHASE_SETTLE:
            self._publish(Twist())
            self.phase_count += 1

            if self.phase_count >= SETTLE_CALLBACKS:
                # Choose turn direction once here and never again
                self.turn_dir = random.choice(['left', 'right'])
                self.get_logger().info(
                    f'Settled – will turn [{self.turn_dir}] – creeping to centre')
                self.phase       = PHASE_CENTRE
                self.phase_count = 0
            return

        # ── PHASE 2: Creep forward until [0,1,1,1,0] ────────────────────── #
        if self.phase == PHASE_CENTRE:
                    # As soon as the outer sensors clear the horizontal line, we've reached the center.
                    # We accept slightly misaligned states so we don't penalize a realistic approach angle.
                    centre_found = s in [
                        [0, 1, 1, 1, 0], 
                        [0, 1, 1, 0, 0], 
                        [0, 0, 1, 1, 0], 
                        [0, 0, 1, 0, 0]
                    ]

                    if centre_found:
                        self.phase       = PHASE_SETTLE_2
                        self.phase_count = 0
                        self._publish(Twist())
                    else:
                        twist.linear.x  = LINEAR_CREEP
                        twist.angular.z = 0.0
                        self._publish(twist)
                        self.phase_count += 1
                    return

        # ── PHASE 3: Settle before turn ──────────────────────────────────── #
        if self.phase == PHASE_SETTLE_2:
            self._publish(Twist())
            self.phase_count += 1
            self.get_logger().info(f'  settling before turn – count={self.phase_count}')

            if self.phase_count >= SETTLE_CALLBACKS:
                self.get_logger().info(f'Settled – starting 90° {self.turn_dir} turn')
                self.turn_left_line = False   # ensure clean state before turn
                self.phase          = PHASE_TURN
                self.phase_count    = 0
            return

        if self.phase == PHASE_TURN:
                    # See 'Additional Sensor-Logic Issues' below regarding this strict condition
                    # on_line = (fl == 0 and cl == 1 and c == 1 and cr == 1 and fr == 0)
                    # Returns True as long as the center is on the line and the extreme edges are off
                    on_line = (c == 1 and fl == 0 and fr == 0)
                    ang     = ANGULAR_TURN if self.turn_dir == 'left' else -ANGULAR_TURN

                    if not self.turn_left_line:
                        if not on_line:
                            self.turn_left_line = True

                        twist.linear.x  = 0.0
                        twist.angular.z = ang
                        self._publish(twist)

                    else:
                        if on_line:
                            # FIX 2: Skip PHASE_DONE. Go straight to FOLLOW so momentum isn't killed
                            self.turn_left_line = False
                            self.phase          = PHASE_FOLLOW
                            self.phase_count    = 0
                            # Do not publish Twist() here; let the next callback handle smooth forward motion
                            return 
                        else:
                            twist.linear.x  = 0.0
                            twist.angular.z = ang
                            self._publish(twist)

                    self.phase_count += 1
                    return

        
    # ──────────────────────────────────────────────────────────────────────── #

    def _publish(self, twist: Twist):
        self.pub.publish(twist)
        self.get_logger().debug(
            f'  CMD → lin={twist.linear.x:.3f}  ang={twist.angular.z:.3f}')


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
