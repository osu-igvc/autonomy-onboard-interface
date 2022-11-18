from xml.etree.ElementTree import PI
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
import pygame
from pygame.locals import *
from math import pi


class JoystickSub(Node):
    def __init__(self):
        super().__init__('joy_sub')     # Initilize the node with proper name (does not have to be the same as the class name)
        # Creating a subscriber that outputs joystick messages from publisher into /joy_input using the listener function. 
        # The limit for queued messgaes is still 1 
        self.subscription = self.create_subscription(Joy, '/joy_input', self.map_input_cb, 1)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.cmd_vel = TwistStamped()

        # Publish new commands at 100Hz
        self.publishTimer = self.create_timer(1.0/100.0, self.publishCMDVel)




        self.VEL_LIN_MAX = 3
        self.VEL_LIN_MIN = -2
        self.VEL_LPF_ALPHA = 0.3
        self.VEL_DEADZONE = 0.7
        self.vel_lin_prev = 0


        self.VEL_ANG_MAX = pi
        self.VEL_ANG_MIN = -pi
        self.VEL_ANG_LPF_ALPHA = 0.3
        self.VEL_ANG_DEADZONE = 0.1
        self.vel_ang_prev = 0

        # +V is down on analog stick. -V is up.
        self.JOY_V_MAX = 1
        self.JOY_V_MIN = -1
        self.JOY_V_AXIS = 4
        self.JOY_V_DEADZONE = 0.05

        # +H is right on analog stick. -H is left.
        self.JOY_H_MAX = 1
        self.JOY_H_MIN = -1
        self.JOY_H_AXIS = 3
        self.JOY_H_DEADZONE = 0.15

    def publishCMDVel(self):
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def updateCMDVel(self, linearX: float, angularZ: float):
        self.cmd_vel.twist.linear.x = float(linearX)
        self.cmd_vel.twist.angular.z = float(angularZ)

        self.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel.header.frame_id = 'base_link'


    def map_input_cb(self, msg):       # listens for published data then outputs it in arrays
        # The formatiing of this outputs a long string of arrays with some labels before them.
        #self.get_logger().info('Time: "%s"  Axis and buttons: "%s" , "%s"' % (msg.header.stamp, msg.axes, msg.buttons))

        analogV_input = msg.axes[self.JOY_V_AXIS]
        analogH_input = msg.axes[self.JOY_H_AXIS]

        #self.get_logger().warning("H in: " + str(analogH_input) + ", V in: " + str(analogV_input))

        aV_in_trunc = - round(analogV_input, 3) # Truncate and invert so that +V is forward in robot frame
        aH_in_trunc = - round(analogH_input, 3) # Similar to above

        # Nullify negligible inputs to reduce noise
        if abs(aV_in_trunc) < self.JOY_V_DEADZONE: aV_in_trunc = 0
        if abs(aH_in_trunc) < self.JOY_H_DEADZONE: aH_in_trunc = 0

        cmd_linvel = self.mapLinearVelocity(aV_in_trunc)
        cmd_angvel = self.mapSteeringAngle(aH_in_trunc)

        cmd_linvel_filtered = self.vel_lin_prev * self.VEL_LPF_ALPHA + cmd_linvel * (1-self.VEL_LPF_ALPHA)
        cmd_angvel_filtered = self.vel_ang_prev * self.VEL_ANG_LPF_ALPHA + cmd_angvel * (1-self.VEL_ANG_LPF_ALPHA)

        # Nullify negligible inputs to reduce noise
        if abs(cmd_linvel_filtered) < self.VEL_DEADZONE: cmd_linvel_filtered = 0
        if abs(cmd_angvel_filtered) < self.VEL_ANG_DEADZONE: cmd_angvel_filtered = 0

        self.vel_lin_prev = cmd_linvel_filtered
        self.vel_ang_prev = cmd_angvel_filtered

        self.updateCMDVel(round(cmd_linvel_filtered, 5), round(cmd_angvel_filtered, 5))

        #self.get_logger().warning("LinVel: " + str(cmd_linvel_filtered) + ", StrAng: " + str(cmd_angvel_filtered))



    def mapLinearVelocity(self, analogInput):
        
        linVel = self.linearIshMap(analogInput, self.JOY_V_MIN, self.JOY_V_MAX, self.VEL_LIN_MIN, self.VEL_LIN_MAX)
        return linVel

    def mapSteeringAngle(self, analogInput):
        strAng = self.linearIshMap(analogInput, self.JOY_H_MIN, self.JOY_V_MAX, self.VEL_ANG_MIN, self.VEL_ANG_MAX)
        return strAng


    # Performs linear map. Due to potentially uneven bounds, simply "bottoms out" control input.
    # IE when reversing, will reach maximum speed before analog stick fully moved.
    def linearIshMap(self, value, inMin, inMax, outMin, outMax):
        #outRange = outMax - outMin
        #inRange = inMax - inMin

        #valScale = float(value - inMin) / inRange


        # Symmetrical range
        outRange = 2 * outMax
        inRange = 2 * inMax

        valScale = float(value + inMax) / inRange
        
        out = -outMax + (valScale * outRange)
        #self.get_logger().info('vs: ' + str(valScale) + ', out: ' + str(out))
        if out < outMin: out = outMin
        elif out > outMax: out = outMax

        return out

def main(args=None):
    rclpy.init(args=args)       # Initilizes rclpy before everything else
    joy_sub = JoystickSub()     # Calls subscriber
    rclpy.spin(joy_sub)         # Runs subscriber without timeout limit until keyboard interruption
    joy_sub.destroy_node()      # Destroys node when done
    rclpy.shutdown()            # Shuts down rclpy

if __name__ == '__main__':
    main()
    