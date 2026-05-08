import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray

import RPi.GPIO as GPIO

"""
AlphaBot2 is equipped with five ITR20001 reflective IR sensors for line sensing.
The sensors are read through the TLC1543 ADC (10-bit, serial output).
Each ADC value represents reflected IR intensity from the surface (reflectance).
"""

# TLC1543 / Raspberry Pi GPIO pin mapping (BCM numbering)
CS_PIN = 5         # Chip Select
CLOCK_PIN = 25     # Serial clock
ADDRESS_PIN = 24   # Address input
DATA_OUT_PIN = 23  # ADC serial data output

LINE_SENSORS_TOPIC = "line_sensors"
SPIN_TIMER_PERIOD_SEC = 0.025  # Timer callback period
ADC_PULSE_DELAY_SEC = 0.0001   # 100 us clock pulse delay


def gpio_init():
    """
    Initialize GPIO.
    """
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def gpio_close():
    """
    Close GPIO.
    """
    GPIO.cleanup()


def clip(value, minimum, maximum):
    """
    Clip value between minimum and maximum.
    """
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


class TLC1543:
    """
    TLC1543 ADC interface.
    """

    def __init__(self, cs_pin=CS_PIN, clock_pin=CLOCK_PIN, address_pin=ADDRESS_PIN, data_out_pin=DATA_OUT_PIN):
        self.cs_pin = cs_pin
        self.clock_pin = clock_pin
        self.address_pin = address_pin
        self.data_out_pin = data_out_pin
        self.pulse_delay_sec = ADC_PULSE_DELAY_SEC

        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.clock_pin, GPIO.OUT)
        GPIO.setup(self.address_pin, GPIO.OUT)
        GPIO.setup(self.data_out_pin, GPIO.IN)

        GPIO.output(self.cs_pin, GPIO.HIGH)
        GPIO.output(self.clock_pin, GPIO.LOW)
        GPIO.output(self.address_pin, GPIO.LOW)

    def read_analog_values(self, num_sensors=5):
        """
        ADC serial read sequence for TLC1543.
        Returns num_sensors values from channels [1..num_sensors].
        """
        num_sensors = int(clip(num_sensors, 1, 10))
        values = [0] * (num_sensors + 1)

        for channel in range(0, num_sensors + 1):
            GPIO.output(self.cs_pin, GPIO.LOW)
            for bit_idx in range(0, 8):
                # Send 8-bit address (4 address bits + 4 zeros)
                if bit_idx < 4:
                    if ((channel >> (3 - bit_idx)) & 0x01):
                        GPIO.output(self.address_pin, GPIO.HIGH)
                    else:
                        GPIO.output(self.address_pin, GPIO.LOW)
                else:
                    GPIO.output(self.address_pin, GPIO.LOW)

                values[channel] <<= 1
                if GPIO.input(self.data_out_pin):
                    values[channel] |= 0x01
                GPIO.output(self.clock_pin, GPIO.HIGH)
                time.sleep(self.pulse_delay_sec)
                GPIO.output(self.clock_pin, GPIO.LOW)
                time.sleep(self.pulse_delay_sec)

            for _ in range(0, 4):
                values[channel] <<= 1
                if GPIO.input(self.data_out_pin):
                    values[channel] |= 0x01
                GPIO.output(self.clock_pin, GPIO.HIGH)
                time.sleep(self.pulse_delay_sec)
                GPIO.output(self.clock_pin, GPIO.LOW)
                time.sleep(self.pulse_delay_sec)

            time.sleep(self.pulse_delay_sec)
            GPIO.output(self.cs_pin, GPIO.HIGH)
            time.sleep(self.pulse_delay_sec)

        for i in range(0, num_sensors + 1):
            values[i] >>= 2
        return values[1:]


class LineSensors(Node):
    """
    ROS2 node that publishes raw line sensor values as an Int32MultiArray.
    Data order: [sensor1, sensor2, sensor3, sensor4, sensor5]
    """

    def __init__(self):
        super().__init__("line_sensors")
        self.get_logger().info("Node init ...")

        self.tlc1543 = TLC1543()
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.line_sensors_pub_callback)
        self.line_sensors_pub = self.create_publisher(Int32MultiArray, LINE_SENSORS_TOPIC, 10)
        self.sensor_values = [0, 0, 0, 0, 0]

        self.get_logger().info("Node init complete.")

    def read_line_sensors(self):
        """
        Read 5 raw line sensor values.
        """
        self.sensor_values = self.tlc1543.read_analog_values(num_sensors=5)

    def line_sensors_pub_callback(self):
        """
        Publish a new line sensor message.
        """
        self.read_line_sensors()

        line_sensors_msg = Int32MultiArray()
        line_sensors_msg.data = self.sensor_values

        self.line_sensors_pub.publish(line_sensors_msg)
        self.get_logger().info(f"line_sensors={self.sensor_values}")


def main(args=None):
    """
    Main function of the line_sensors node.
    """
    rclpy.init(args=args)
    gpio_init()

    line_sensors = LineSensors()
    rclpy.spin(line_sensors)
    line_sensors.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
