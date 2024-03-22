"""
Author(s): Everett Tucker
Creation Date: 01/09/2024
Description: Gets controller input from the joy publisher and send a twist message
Subscribers: Joy
Publishers: Twist
"""


import rclpy
import time
import busio
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# Global dynamic variables
controller_init = false
axes = []
buttons = []


class ControllerSub(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('controller_subscriber')
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
        Pad Horizontal: 6 - left -> 1, right -> -1
        Pad Vertical: 7 - up -> 1, down -> -1
        """

        """
        In my testing, I found that an angle of about 110 resulted in no motion
        Whereas an angle of 90 resulted in slow forward motion.
        """

        global controller_init, axes, buttons
        axes = msg.axes
        buttons = msg.buttons
        controller_init = True



class TwistPub(Node):
    def __init__(self):
        # Creating the publisher
        super().__init__("twist_publisher")
        self.publisher = self.create_publisher(Twist, 'twist', 10)

    def publishTwist(self):
        if controller_init:
            twist_message = Twist()

            # Modify twist_message using global variables axes and buttons

            # Linear Motion - (x, y, z)
            twist_message.linear.x = axes[1]
            twist_message.linear.y = axes[0]
            if axes[2] < axes[5]:
                linear_z = (axes[2] - 1) / 2
            else:
                linear_z = -(axes[5] - 1) / 2
            twist_message.linear.z = linear_z

            # Angular Motion - Just yaw for now
            twist_message.angular.x = 0
            twist_message.angular.y = 0
            twist_message.angular.z = axes[3]

            self.publisher.publish(twist_message)
        


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    executor.add_node(ControllerSub())
    executor.add_node(TwistPub())

    # Launching the executor
    executor.spin()

    # Shutting down the program
    rclpy.shutdown()


if __name__ == '__main__':
    main()
