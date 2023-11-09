"""
Author(s): Christopher Holley
Creation Date: 09/09/2023
Description: Handles interfacing with the thrusters
"""

import rclpy
from rclpy.node import Node
# START adafruit imports, won't be in container 
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
# END adafruit imports
    
NUM_THRUSTERS = 6
# thrusters should be wired sequentially on the breakout board, so if you start at 8, the next one is 9, etc.
START_THRUSTER = 8 

INITALIZE_THRUSTER_ANGLE = 90 # angle needed to send 1500 pulse width to ESC to initialize

class ThrusterInterface(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #self.get_logger().info(self.get_node_names_and_namespaces())

        # Create the I2C bus interface.
        self.i2c_bus = busio.I2C(SCL, SDA)

        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(self.i2c_bus)

        # Set the PWM frequency to 60hz.
        self.pca.frequency = 450

        # Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
        # but the PCA9685 will only actually give 12 bits of resolution.
        self.pca.channels[0].duty_cycle = 0xFFFF

        self.thrusters = []
        for i in range(NUM_THRUSTERS):
            # idea with the servo is writing to the angle will set the duty cycle
            self.thrusters.append(servo.Servo(self.pca.channels[START_THRUSTER + i], min_pulse=1100, max_pulse=1900))
        # log that thrusters are ready
        self.get_logger().info(f"Created thrust interface with {NUM_THRUSTERS} thrusters")
        
        for propeller in self.thrusters:
            propeller.angle = INITALIZE_THRUSTER_ANGLE    
    

    def disable_thrusters(self):
        self.get_logger().error("DISABLING THRUSTERS")
        for propeller in self.thrusters:
            propeller.angle = INITALIZE_THRUSTER_ANGLE

    # takes 
    def map_speed_to_angle(self, speed):
        

def main(args=None):
    rclpy.init(args=args)
    thruster_interface = ThrusterInterface()
    rclpy.spin(thruster_interface)
    rclpy.shutdown()


if __name__ == '__main__':
   main()