import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray

import RPi.GPIO as GPIO

"""
AlphaBot2 is equipped with five ITR20001 reflective IR sensors for line tracking.
The sensors are read through the TLC1543 ADC (10-bit, serial output).
Each ADC value represents reflected IR intensity from the surface (reflectance)
"""

# TLC1543 / Raspberry Pi GPIO pin mapping (BCM numbering)
CS_PIN = 5        # Chip Select
CLOCK_PIN = 25    # Serial clock
ADDRESS_PIN = 24  # Address input
DATA_OUT_PIN = 23 # ADC serial data output

# ITR20001 channels on TLC1543 (left -> right).
# On many AlphaBot2 boards sensors are wired to even ADC channels.
LEFT_OUTER_CH = 0
LEFT_INNER_CH = 2
CENTER_CH = 4
RIGHT_INNER_CH = 6
RIGHT_OUTER_CH = 8

LINE_TRACKING_TOPIC = "line_tracking"
SPIN_TIMER_PERIOD_SEC = 0.025  # Timer callback period


def gpio_init():
    """
    Function that initializes GPIO.
    """
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def gpio_close():
    """
    Function that closes GPIO.
    """
    GPIO.cleanup()


def clip(value, minimum, maximum):
    """
    Function that clips a value between a minimum and a maximum.
    """
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


class TLC1543:
    """
    Class that represents the TLC1543 ADC.
    """
    def __init__(self, cs_pin=CS_PIN, clock_pin=CLOCK_PIN, address_pin=ADDRESS_PIN, data_out_pin=DATA_OUT_PIN):
        self.cs_pin = cs_pin
        self.clock_pin = clock_pin
        self.address_pin = address_pin
        self.data_out_pin = data_out_pin
        self.pulse_delay_sec = 0.000020

        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.clock_pin, GPIO.OUT)
        GPIO.setup(self.address_pin, GPIO.OUT)
        # DOUT is actively driven by ADC, so keep this input floating (no internal pull-up/down).
        GPIO.setup(self.data_out_pin, GPIO.IN)

        GPIO.output(self.cs_pin, GPIO.HIGH)
        GPIO.output(self.clock_pin, GPIO.LOW)
        GPIO.output(self.address_pin, GPIO.LOW)

    def _clock_pulse(self):
        GPIO.output(self.clock_pin, GPIO.HIGH)
        time.sleep(self.pulse_delay_sec)
        GPIO.output(self.clock_pin, GPIO.LOW)
        time.sleep(self.pulse_delay_sec)

    def _read_channel_once(self, channel):
        """
        Function that reads one TLC1543 channel once and returns a 10-bit raw reflectance value (0..1023).
        """
        channel = int(clip(channel, 0, 10))

        GPIO.output(self.cs_pin, GPIO.LOW)
        time.sleep(self.pulse_delay_sec)

        # Send 4-bit channel address (MSB first)
        for shift in (3, 2, 1, 0):
            bit = (channel >> shift) & 0x01
            GPIO.output(self.address_pin, GPIO.HIGH if bit else GPIO.LOW)
            self._clock_pulse()

        # Dummy pulse for alignment
        self._clock_pulse()

        value = 0
        for _ in range(10):
            GPIO.output(self.clock_pin, GPIO.HIGH)
            time.sleep(self.pulse_delay_sec)
            value = (value << 1) | GPIO.input(self.data_out_pin)
            GPIO.output(self.clock_pin, GPIO.LOW)
            time.sleep(self.pulse_delay_sec)

        GPIO.output(self.cs_pin, GPIO.HIGH)
        return int(clip(value, 0, 1023))

    def read_channel(self, channel):
        """
        Function that reads one TLC1543 channel and returns a raw 10-bit reflectance value (0..1023).
        First read is discarded as dummy read (ADC settling), second read is returned.
        """
        self._read_channel_once(channel)  # dummy read
        return self._read_channel_once(channel)


class IRLineTrackingSensors(Node):
    """
    ROS2 node that controls AlphaBot2's IR line tracking sensors (ITR20001 + TLC1543).
    It publishes Int32MultiArray messages on the "line_tracking" topic.
    Data order: [left_outer, left_inner, center, right_inner, right_outer]
    """
    def __init__(self):
        super().__init__("IR_line_tracking_sensors")

        self.get_logger().info("Node init ...")

        self.tlc1543 = TLC1543()

        # Create the timer (called by rclpy.spin()) with its callback function
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.line_tracking_pub_callback)

        # Topic publisher
        self.line_tracking_pub = self.create_publisher(Int32MultiArray, LINE_TRACKING_TOPIC, 10)

        # Internal status
        self.left_outer = 0
        self.left_inner = 0
        self.center = 0
        self.right_inner = 0
        self.right_outer = 0

        self.get_logger().info("Node init complete.")

    def check_line_sensors(self):
        """
        Function that reads all line tracking sensors and updates the internal status.
        Values are raw reflectance measurements from the ADC.
        """
        self.left_outer = self.tlc1543.read_channel(LEFT_OUTER_CH)
        self.left_inner = self.tlc1543.read_channel(LEFT_INNER_CH)
        self.center = self.tlc1543.read_channel(CENTER_CH)
        self.right_inner = self.tlc1543.read_channel(RIGHT_INNER_CH)
        self.right_outer = self.tlc1543.read_channel(RIGHT_OUTER_CH)

    def line_tracking_pub_callback(self):
        """
        Function called to publish a new line tracking message on "line_tracking" topic.
        """
        self.check_line_sensors()

        # Create the Int32MultiArray message
        line_tracking_msg = Int32MultiArray()
        line_tracking_msg.data = [
            self.left_outer,
            self.left_inner,
            self.center,
            self.right_inner,
            self.right_outer
        ]

        # Publish the message
        self.line_tracking_pub.publish(line_tracking_msg)
        self.get_logger().info(
            f"Publishing >> "
            f"left_outer_reflectance={self.left_outer}, left_inner_reflectance={self.left_inner}, "
            f"center_reflectance={self.center}, right_inner_reflectance={self.right_inner}, "
            f"right_outer_reflectance={self.right_outer}"
        )


def main(args=None):
    """
    Main function of the IR_line_tracking_sensors node.
    """
    rclpy.init(args=args)
    gpio_init()

    # Create, spin and destroy the node
    ir_line_tracking_sensors = IRLineTrackingSensors()
    rclpy.spin(ir_line_tracking_sensors)
    ir_line_tracking_sensors.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
