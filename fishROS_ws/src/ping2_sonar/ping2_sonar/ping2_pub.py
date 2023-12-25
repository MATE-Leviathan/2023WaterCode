"""
Author(s): Christopher Holley
Creation Date: 09/07/2023
Description: Handles the interfacing with the Ping2 sonar and publishing the data to the topic 'Data'
"""
"test by larry to try pushing to github"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np
import adafruit_bno055
import board
import tf_transformations

# shouldn't need to change this
BAUD_RATE = 115200
# @TODO Make this use a launch file
DEVICE = "/dev/ttyUSB0"


class ImuPub(Node):
    """
    The Ping2SonarPub object represents the publisher node for the ping2 sonar. 
    It handles interfacing with the ping2 sonar and publishing the data to the topic 'Data'

    Args:
        None

    Attributes:
        publisher: ROS publisher object, publishes Ping2 messages to the topic 'Data'
        ping_sonar: Ping1D object, used to get data from the ping2 sonar
        timer: ROS timer object, used to call the read_and_publish_data function at exactly 50hz

    Special Cases:
        None
    """
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Imu, 'Data', 10)

        i2c = board.I2C()

        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
        print(self.sensor.temperature)
        timer_period = 0.02 # seconds, should be 50hz
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)            

    """
    Reads in the most recent data from the ping2 sonar and publishes it to the topic 'Data'

    Args:
        None

    Returns:
        None

    Raises:
        Does not raise but will log error if unable to read frame from ping2
    """
    def read_and_publish_data(self):
        data = self.sensor.euler
        if not data:
            self.get_logger().error("Failed reading IMU data")

        imu_msg = Imu()
        imu_msg.orientation = tf_transformations.quaternion_from_euler(data)
        self.publisher.publish(imu_msg)        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ImuPub()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()