#!/usr/bin/env python3
"""
AlphaBot2 Line Follower - simple strategy
=========================================

Behavior:
  - If the centre sensor sees the line, drive straight ahead.
  - Only apply a correction when the centre sensor stops seeing the line.
  - Use the side sensors to decide whether to turn left or right.
  - If only the centre sensor sees white, rotate in place to recover the line.
  - Stop only if no sensor sees the line.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


# Values to tune with real sensor data.
BLACK_THRESHOLD = 500

LINEAR_SPEED = 0.1
CORRECTION_SPEED = 0.15
ANGULAR_CORRECTION = 1.0


class LineFollowerSimple(Node):

    def __init__(self):
        super().__init__('alphabot2_line_follower_simple')
        self.last_action = None
        self.search_turn_sign = 1.0

        self.pub = self.create_publisher(Twist, '/alphabot2/cmd_vel', 10)
        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/line_sensors',
            self.sensor_callback,
            10,
        )

        self.get_logger().info('AlphaBot2 simple line follower started.')
        self.get_logger().info(f'  BLACK_THRESHOLD = {BLACK_THRESHOLD}')

    def sensor_callback(self, msg: Int32MultiArray):
        raw = list(msg.data)

        if len(raw) < 5:
            self.get_logger().warn(f'Expected 5 values, got {len(raw)}: {raw}')
            return

        # 1 = white / background, 0 = black / line
        s = [1 if value >= BLACK_THRESHOLD else 0 for value in raw]
        _, cl, c, cr, _ = s

        twist = Twist()

        if c == 0:
            # If the middle sensor sees black, do not correct: just go straight.
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = 0.0
            self._publish(twist, 'FORWARD')
            return

        # Only correct when the centre sensor is white, using the adjacent side sensors.
        if cl == 0 and c == 1 and cr == 0:
            # The line is under every sensor except the centre one.
            # Rotate in place until the centre sensor finds black again.
            twist.linear.x = 0.0
            twist.angular.z = self.search_turn_sign * ANGULAR_CORRECTION
            action = 'SPIN SEARCH'
        else:
            left_black = cl == 0
            right_black = cr == 0

            if left_black and not right_black:
                # The middle sensor lost the line, but the left side still sees it.
                twist.linear.x = CORRECTION_SPEED
                twist.angular.z = ANGULAR_CORRECTION
                self.search_turn_sign = 1.0
                action = 'CORRECT LEFT'
            elif right_black and not left_black:
                # The middle sensor lost the line, but the right side still sees it.
                twist.linear.x = CORRECTION_SPEED
                twist.angular.z = -ANGULAR_CORRECTION
                self.search_turn_sign = -1.0
                action = 'CORRECT RIGHT'
            elif left_black and right_black:
                # Both adjacent sensors still see black, so move slowly ahead.
                twist.linear.x = CORRECTION_SPEED
                twist.angular.z = 0.0
                action = 'SLOW FORWARD'
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                action = 'STOP'

        self._publish(twist, action)

    def _publish(self, twist: Twist, action: str):
        self.pub.publish(twist)
        if action != self.last_action:
            self.last_action = action
            self.get_logger().info(f'Action: {action}')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.get_logger().info('Stopped. Shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
