"""
Author(s): Christopher Holley
Creation Date: 09/07/2023
Description: Handles the interfacing with the Ping2 sonar and publishing the data to the topic 'Data'
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from brping import Ping1D
from ping2_sonar_msgs.msg import Ping2

# shouldn't need to change this
BAUD_RATE = 115200
# @TODO Make this use a launch file
DEVICE = "/dev/ttyUSB0"


class Ping2SonarPub(Node):
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
        self.publisher = self.create_publisher(Ping2, 'Data', 10)
        self.ping_sonar = Ping1D()
        
        self.ping_sonar.connect_serial(DEVICE, BAUD_RATE)
        MAX_RETRIES = 3

        for i in range(1, MAX_RETRIES+1):
            if self.ping_sonar.initialize():
                break
            self.get_logger().error(f"Failed to initialize Ping2! Trying {MAX_RETRIES-i} more times")
            if i == MAX_RETRIES:
                self.get_logger().error("Unable to start ping2 sonar, EXITING")
                rclpy.shutdown()
                exit()
        
        self.get_logger().info("Ping2 Sonar Initalized Sucessfully")
        self.get_logger().info("Ping2 Sonar Initalized Sucessfully")

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
        data = self.ping_sonar.get_distance()
        if not data:
            self.get_logger().error("Failed reading sonar data")

        sonar_msg = Ping2()
        sonar_msg.distance = data["distance"] / 1000 # convert mm to m
        sonar_msg.confidence = float(data["confidence"])
        self.publisher.publish(sonar_msg)        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Ping2SonarPub()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()