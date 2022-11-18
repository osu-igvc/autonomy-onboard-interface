import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

import pygame
from pygame.locals import *
       

class JoystickPub(Node):
    def __init__(self):
        super().__init__('joy_pub')     # Initilize the node with proper name (does not have to be the same as the class name)

        # Initilizing pygame's joystick module
        pygame.joystick.init()     
        # Connecting to joystick (if there is more than 1 you would change it from 0th). 
        self.stick = pygame.joystick.Joystick(0)    
        
        # Initilize the connected joystick
        self.stick.init()          
        # Creating timer that updates every 0.05 seconds
        self.timer = self.create_timer(0.05, self.update)       
        
        # Creating publisher that publishes standard Joy messages (imported from std_mssgs), the topic is /joy_input, 
        # and queued messages limit (if subscriber is not fast enough) is 1:
        self.publisher = self.create_publisher(Joy, '/joy_input', 1) 
         # Message type is set to joystick, a package built into ros2 made for controllers and joysticks:
        self.msg = Joy()       
         # Reports axis, must be in float32 format:
        self.msg.axes = [float(0)]*self.stick.get_numaxes()

        # Reports buttons, must be int32 format:
        self.msg.buttons = [0]*(self.stick.get_numbuttons())  

    def updateHeader(self):
         # Sending timestamp with each update 
        self.msg.header.stamp = self.get_clock().now().to_msg()    
    
    
     # Every time an event occurs this loop will determine if it is a button or axis motion 
     # and will assign values according to its state
    def update(self):      
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:      
                self.msg.buttons[event.button] = 1
            elif event.type == pygame.JOYBUTTONUP:
                self.msg.buttons[event.button] = 0
                    
            elif event.type == pygame.JOYAXISMOTION:
                self.msg.axes[event.axis] = event.value
        self.updateHeader()    #Updates the header each time update runs and outputs 
        self.publisher.publish(self.msg)    # Publishes values each time update runs


def main():
    pygame.init()       # Initilize pygame before anything else
    rclpy.init()        # Initilize rclpy 
    joy_pub = JoystickPub()

    # Spin node; runs node until keyboard interrupts such as crtl + c
    # This section needs to be changed to allow timeouts to shutdown the program 
    # Ideally there will be a shutdown sequence to allow the car to come to a stop safely
    rclpy.spin(joy_pub) 
    joy_pub.destroy_node()  # Destroy node at the end
    rclpy.shutdown()        # Shutdown rclpy at end


if __name__ == "__main__":
    main()
    
    