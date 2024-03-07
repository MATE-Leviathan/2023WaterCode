"""
Author(s): Everett Tucker
Creation Date: 01/09/2024
Description: Tests getting controller inputs from the joy topic and writing to motors
"""


import rclpy
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from rclpy.node import Node
from sensor_msgs.msg import Joy

class MinimalSubscriber(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)

        # Creating the servo
        i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = 450

        # Sets the duty cycle for the LED pin to match the servo
        self.pca.channels[0].duty_cycle = 0xFFFF

        # Turns the LED off
        # self.pca.channels[0].duty_cycle = 0x0000

        self.thruster = servo.ContinuousServo(self.pca.channels[8], min_pulse=1100, max_pulse=1900)

        self.thruster.throttle = 0
        self.get_logger().info("Thruster Initializing...")
        time.sleep(7)

    def listener_callback(self, msg):
        """
        msg.buttons contains the button inputs.
        Each index below returns 1 if pressed and 0 if not
        A: 0
        B: 1
        X: 2
        Y: 3
        L Bump: 4
        R Bump: 5
        View Button (Small Left): 6
        Menu Button (Small Right): 7
        Power/Xbox Button: 8
        Left Joystick Press: 9
        Right Joystick Press: 10

        msg.axes contains the float inputs
        Each index below returns a float in [-1, 1]

        Left Stick Horizontal: 0 - [left, right] -> [1, -1]
        Left Stick Vertical: 1 - [up, down] -> [1, -1]
        Left Trigger: 2 - [pressed, not pressed] -> [-1, 1]
        Right Stick Horizontal: 3 - [left, right] -> [1, -1]
        Right Stick Vertical: 4 - [up, down] -> [1, -1]
        Right Trigger: 5 - [pressed, not pressed] -> [-1, 1]
        Pad Horizontal: 6 - left -> 1, right -> -1
        Pad Vertical: 7 - up -> 1, down -> -1
        """

        """
        In my testing, I found that an angle of about 110 resulted in no motion
        Whereas an angle of 90 resulted in slow forward motion.
        """

        # Angle is limited to [45, 135]
        self.thruster.throttle = msg.axes[1]

        # This time roughly matches the frequency of the pwm - 450 Hz
        time.sleep(0.002)

        self.get_logger().info(f'Throttle is at: {msg.axes[1]}')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()