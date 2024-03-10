"""
Author(s): Everett Tucker
Date Created: March 6, 2024
Description: Controls the MATE ROV with a twist message
"""

import rclpy
import time
import math
import busio
from board import SCL, SDA
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from geometry_msgs.msg import Twist

# Constants
NUM_THRUSTERS = 6
START_PIN = 8
ONEOVERROOTTWO = 1 / math.sqrt(2)
CONTROLLER_DEADZONE = 0.01

class DriveRunner(Node):
    def __init__(self):
        # Creating the node and subscriber
        super().__init__("drive_runner")
        self.subscription = self.create_subscription(Twist, "twist", self.twist_callback, 10)

        # Initializing the PCA Board
        i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = 450

        # Initializing the thrusters
        self.thrusters = []
        for i in range(START_PIN, START_PIN + NUM_THRUSTERS):
            self.thrusters.append(servo.Servo(self.pca.channel[START_PIN + i], min_pulse=1141, max_pulse=1871))
    

    def drivetrainInit(self):
        # Setting thrusters to initialization angles for 7 seconds
        for thruster in self.thrusters:
            thruster.angle = 90
        time.sleep(7)

    
    def twist_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        x_rotation = msg.angular.x
        z_rotation = msg.angular.z
        ### Horizontal Motor Writing
        if abs(x) > CONTROLLER_DEADZONE or abs(y) > CONTROLLER_DEADZONE:  # Linear Movement in XY
            x *= -1  # Reversal to compensate for the non-standard basis
            set_thruster(5, -1 * ONEOVERROOTTWO * (x + y))
            set_thruster(3, ONEOVERROOTTWO * (x - y))
            set_thruster(0, ONEOVERROOTTWO * (y - x))
            set_thruster(1, ONEOVERROOTTWO * (y + x))
        elif abs(z_rotation) > CONTROLLER_DEADZONE:  # Yaw (Spin)
            set_thruster(5, -z_rotation)
            set_thruster(3, z_rotation)
            set_thruster(1, -z_rotation)
            set_thruster(0, z_rotation)

        ### Vertical Motor Writing
        if abs(z) > CONTROLLER_DEADZONE:  # Linear Movement in Z
            set_thruster(2, z)
            set_thruster(4, z)
        elif abs(x_rotation) > CONTROLLER_DEADZONE:  # Roll
            set_thruster(2, x_rotation)
            set_thruster(4, -x_rotation)
    

    def set_thruster(self, index, value):
        self.thrusters[index] = value
