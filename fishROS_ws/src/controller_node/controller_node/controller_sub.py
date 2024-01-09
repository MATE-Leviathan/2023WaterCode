"""
Author(s): Everett Tucker
Creation Date: 01/09/2024
Description: Tests getting controller inputs from the joy topic
"""

import rclpy
from rclpy.node import Node
import busio
from sensor_msgs.msg import Joy

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
    
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
        Pad Horizontal: 6 - [left, right] -> [1, -1]
        Pad Vertical: 7 - [up, down] -> [1, -1]
        """
        
        self.get_logger().info(f'Press the A Button: {msg.buttons[0]}')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Tests getting controller inputs from the joy topic. 
