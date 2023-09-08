import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from brping import Ping1D
from ping2_sonar_msgs.msg import Ping2
import time

# shouldn't need to change this
BAUD_RATE = 115200
# @TODO Make this use a launch file
DEVICE = "/dev/ttyUSB0"


class Ping2SonarPub(Node):
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
        
        self.get_logger().info("Ping2 Sonar Initalized Sucessfully")
        
        timer_period = 0.02 # seconds, should be 50hz
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)            

    def read_and_publish_data(self):
        #d = myPing.get_profile()
        #print(myPing.get_speed_of_sound())
        data = self.ping_sonar.get_distance()
        if not data:
            self.get_logger().info("Failed reading sonar data")

        sonar_msg = Ping2()
        sonar_msg.distance = data["distance"] / 1000 # convert mm to m
        sonar_msg.confidence = data["confidence"]
        self.publisher.publish(sonar_msg)        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Ping2SonarPub()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()