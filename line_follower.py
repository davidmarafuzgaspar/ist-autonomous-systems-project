#!/usr/bin/env python3
"""
AlphaBot2 Line Follower 
==================================
Topics:
  SUB  /alphabot2/line_sensors   → std_msgs/Int32MultiArray
  PUB  /alphabot2/cmd_vel        → geometry_msgs/Twist

Sensor layout (left → right):
  [0] FAR LEFT     – border, should always be on WHITE
  [1] CENTRE LEFT  – should be on BLACK (line)
  [2] CENTRE       – should be on BLACK (line)
  [3] CENTRE RIGHT – should be on BLACK (line)
  [4] FAR RIGHT    – border, should always be on WHITE

State machine:
  3 centre on black, 2 outer on white  → on track, go forward
  All 5 on black                        → junction / cross detected → stop
  Drifting (partial centre coverage)   → steer to correct
  All 5 on white                        → lost the track → stop

Calibration:
  Run:  ros2 topic echo /alphabot2/line_sensors
  - Note values when ON  the black line  (should be HIGH)
  - Note values when OFF the line        (should be LOW)
  - Set BLACK_THRESHOLD between the two
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import random


#  VALUES TO BE TUNED
BLACK_THRESHOLD = 500   # values >= this = WHITE (high reflection)

LINEAR_SPEED    = 0.7   # m/s   – cruising speed
ANGULAR_SLOW    = 0.50  # rad/s – gentle correction
ANGULAR_FAST    = 0.90  # rad/s – sharp correction
ANGULAR_HARD    = 1.20  # rad/s – recovery (outer sensor triggered)


class LineFollower(Node):

    def __init__(self):
        super().__init__('alphabot2_line_follower')

        self.at_junction = False   
        self.junction_action = None

        self.pub = self.create_publisher(Twist, '/alphabot2/cmd_vel', 10)
        self.create_subscription(
            Int32MultiArray,
            '/alphabot2/line_sensors',
            self.sensor_callback,
            10
        )
        self.get_logger().info('AlphaBot2 line follower v3 started.')
        self.get_logger().info(f'  BLACK_THRESHOLD = {BLACK_THRESHOLD}')


    def sensor_callback(self, msg: Int32MultiArray):
        raw = list(msg.data)

        if len(raw) < 5:
            self.get_logger().warn(f'Expected 5 values, got {len(raw)}: {raw}')
            return

        # Convert analog → binary  (1 = BLACK / on line,  0 = WHITE / off line)
        s = [0 if v >= BLACK_THRESHOLD else 1 for v in raw]
        fl, cl, c, cr, fr = s   # far_left, centre_left, centre, centre_right, far_right

        total_black = sum(s)

        self.get_logger().debug(f'raw={raw}  →  {s}  (sum={total_black})')

        twist = Twist()

        """
      Twist
        ├── linear
        │   ├── x  →  forward / backward  (the one we use)
        │   ├── y  →  sideways strafe      (not used on a 2-wheel robot)
        │   └── z  →  up / down           (not used on a ground robot)
        └── angular
            ├── x  →  roll                 (not used)
            ├── y  →  pitch                (not used)
            └── z  →  turn left / right   (the one we use)
        """

        # ── CASE 1: All 5 on black → junction detected ────────────────────── #
        if total_black == 5 and not self.at_junction:
            self.at_junction = True
            self.junction_action = random.choice(['forward', 'left', 'right'])
            self.get_logger().info(f'JUNCTION detected – chose: {self.junction_action}')

        # ── CASE 1b: Executing junction action until recentred ────────────── #
        if self.at_junction:
            if cl == 1 and c == 1 and cr == 1 and total_black == 3:
                # Centre sensors back on black → junction cleared
                self.get_logger().info('Junction cleared – resuming line follow')
                self.at_junction = False
                self.junction_action = None
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = 0.0
            else:
                # Keep executing the chosen action
                if self.junction_action == 'forward':
                    twist.linear.x  = LINEAR_SPEED
                    twist.angular.z = 0.0
                elif self.junction_action == 'left':
                    twist.linear.x  = 0.0
                    twist.angular.z =  ANGULAR_HARD   # positive = LEFT
                elif self.junction_action == 'right':
                    twist.linear.x  = 0.0
                    twist.angular.z = -ANGULAR_HARD   # negative = RIGHT

            self._publish(twist)
            return

        # ── CASE 2: All 5 on white → lost the track ───────────────────────── #
        if total_black == 0:
            self.get_logger().warn('LOST TRACK (all 5 on white) – stopping')
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self._publish(twist)
            return

        # ── CASE 3: Outer border sensor on black → drifting off track ─────── #
        # (Check AFTER the all-5-black junction case so we don't misread a cross)
        if fl == 1 and fr == 0:
            self.get_logger().warn('Far-left on black – hard RIGHT recovery')
            twist.linear.x  = 0.0
            twist.angular.z = -ANGULAR_HARD
            self._publish(twist)
            return

        if fr == 1 and fl == 0:
            self.get_logger().warn('Far-right on black – hard LEFT recovery')
            twist.linear.x  = 0.0
            twist.angular.z =  ANGULAR_HARD
            self._publish(twist)
            return

        # ── CASE 4: Normal line following on centre sensors ────────────────── #
        # Visual key:  ● = on black (1),  ○ = on white (0)
        #              [CL] [C] [CR]

        if   cl == 1 and c == 1 and cr == 1:
            # ● ● ●  Perfectly centred → full speed
            twist.linear.x  = LINEAR_SPEED
            twist.angular.z = 0.0

        elif cl == 1 and c == 1 and cr == 0:
            # ● ● ○  Right sensor lost line → line is left → turn LEFT
            twist.linear.x  = LINEAR_SPEED * 0.8
            twist.angular.z =  ANGULAR_SLOW    # positive = LEFT

        elif cl == 0 and c == 1 and cr == 1:
            # ○ ● ●  Left sensor lost line → line is right → turn RIGHT
            twist.linear.x  = LINEAR_SPEED * 0.8
            twist.angular.z = -ANGULAR_SLOW    # negative = RIGHT 

        elif cl == 0 and c == 1 and cr == 0:
            # ○ ● ○  Only centre on line – balanced, keep going
            twist.linear.x  = LINEAR_SPEED 
            twist.angular.z = 0.0

        elif cl == 1 and c == 0 and cr == 0:
            # ● ○ ○  Drifted far right → sharp left turn
            twist.linear.x  = LINEAR_SPEED * 0.3
            twist.angular.z =  ANGULAR_FAST

        elif cl == 0 and c == 0 and cr == 1:
            # ○ ○ ●  Drifted far left → sharp right turn
            twist.linear.x  = LINEAR_SPEED * 0.3
            twist.angular.z = -ANGULAR_FAST

        elif cl == 1 and c == 0 and cr == 1:
            # ● ○ ●  Wide line or T-junction → go straight, let next reading decide
            twist.linear.x  = LINEAR_SPEED * 0.5
            twist.angular.z = 0.0

        else:
            # Catch-all safety stop
            self.get_logger().warn(f'Unhandled sensor pattern {s} – stopping')
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        self._publish(twist)


    def _publish(self, twist: Twist):
        self.pub.publish(twist)
        self.get_logger().debug( f'  CMD → lin={twist.linear.x:.3f}  ang={twist.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
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
