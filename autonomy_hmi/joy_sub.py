import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
import pygame
from pygame.locals import *


class JoystickSub(Node):
    def __init__(self):
        super().__init__('joy_sub')     # Initilize the node with proper name (does not have to be the same as the class name)
        # Creating a subscriber that outputs joystick messages from publisher into /joy_input using the listener function. 
        # The limit for queued messgaes is still 1 
        self.subscription = self.create_subscription(Joy, '/joy_input', self.listener_callback, 1)
        self.subscription       # Just here to prevent unused variable warning, not needed otherwise

    def listener_callback(self, msg):       # listens for published data then outputs it in arrays
        # The formatiing of this outputs a long string of arrays with some labels before them.
        self.get_logger().info('Time: "%s"  Axis and buttons: "%s" , "%s"' % (msg.header.stamp, msg.axes, msg.buttons))

def main(args=None):
    rclpy.init(args=args)       # Initilizes rclpy before everything else
    joy_sub = JoystickSub()     # Calls subscriber
    rclpy.spin(joy_sub)         # Runs subscriber without timeout limit until keyboard interruption
    joy_sub.destroy_node()      # Destroys node when done
    rclpy.shutdown()            # Shuts down rclpy

if __name__ == '__main__':
    main()
    