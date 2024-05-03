"""
Author(s): Everett Tucker
Date Created: March 6, 2024
Description: Controls the MATE ROV with a twist message
Subscribers: Subscribes to a twist message and an IMU message
Publishers: None
"""

import rclpy
import time
import math
import busio
from board import SCL, SDA
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

# Final Global Variables
THRUSTER_PINS = [1, 2, 3, 13, 14, 15]
ONEOVERROOTTWO = 1 / math.sqrt(2)
CONTROLLER_DEADZONE = 0.01
THRUST_SCALE_FACTOR = 0.83375

# Dynamic Global Variables
imu_init = False
orientation = Quaternion()
linear_acceleration = Vector3()
angular_velocity = Vector3()

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
        for pin in THRUSTER_PINS:
            self.thrusters.append(servo.Servo(self.pca.channels[pin], min_pulse=1141, max_pulse=1971))

        self.drivetrainInit()


    def drivetrainInit(self):
        # Setting thrusters to initialization angles for 7 seconds
        print("Initializing Thrusters... Spam the Controller!")
        for i in range(6):
            self.set_thruster(i, 0.0)
        time.sleep(7)
        print("Ready!")

    def set_thruster(self, index, value):
        value = min(max(value, -1), 1)  # Keeping it in bounds
        value = value if value < 0 else value * THRUST_SCALE_FACTOR
        self.thrusters[index].angle = 90 * value + 90

    """
    Current Mapping 4/13/24:
    LF: 1
    LU: 2
    LB: 3
    RF: 15
    RU: 14
    RB: 13
    """
    
    def twist_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        x_rotation = msg.angular.x
        z_rotation = msg.angular.z
        ### Horizontal Motor Writing
        if abs(x) > CONTROLLER_DEADZONE or abs(y) > CONTROLLER_DEADZONE: # Linear Movement in XY
            self.set_thruster(5, ONEOVERROOTTWO * (x - y))
            self.set_thruster(0, ONEOVERROOTTWO * (x + y))
            self.set_thruster(3, ONEOVERROOTTWO * (-y - x))
            self.set_thruster(2, ONEOVERROOTTWO * (y - x))
        elif abs(z_rotation) > CONTROLLER_DEADZONE:  # Yaw (Spin)
            self.set_thruster(5, -z_rotation)
            self.set_thruster(0, z_rotation)
            self.set_thruster(3, z_rotation)
            self.set_thruster(2, -z_rotation)
        else:
            self.set_thruster(5, 0.0)
            self.set_thruster(0, 0.0)
            self.set_thruster(3, 0.0)
            self.set_thruster(2, 0.0)

        ### Vertical Motor Writing
        if abs(z) > CONTROLLER_DEADZONE:  # Linear Movement in Z
            self.set_thruster(1, -z)
            self.set_thruster(4, -z)
        elif abs(x_rotation) > CONTROLLER_DEADZONE:  # Roll
            self.set_thruster(1, x_rotation)
            self.set_thruster(4, -x_rotation)
        else:
            self.set_thruster(1, 0.0)
            self.set_thruster(4, 0.0)


class IMUSub(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(Imu, 'IMUData', self.imu_callback, 10)

    def imu_callback(self, msg):
        global orientation, linear_acceleration, angular_velocity, imu_init
        orientation = msg.orientation
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity
        imu_init = True


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    drive_runner = DriveRunner()
    imu_sub = IMUSub()
    executor.add_node(drive_runner)
    executor.add_node(imu_sub)

    # Starting the execution loop
    executor.spin()

    # Destroying Nodes
    drive_runner.destroy_node()
    imu_sub.destroy_node()

    # Shutting down the program
    rclpy.shutdown()


if __name__ == '__main__':
    main()

