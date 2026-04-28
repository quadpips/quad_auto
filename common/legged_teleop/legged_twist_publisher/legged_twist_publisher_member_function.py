#!/usr/bin/env python3

from __future__ import print_function

import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

import termios
import tty

import numpy as np


TwistMsg = Twist

class LeggedTwistPublisher(Node):
    def __init__(self):
        super().__init__('legged_twist_publisher')
        self.publisher = self.create_publisher(TwistMsg, '/cmd_vel', 10)
        self.vel_linear_x = 0.0
        self.vel_linear_y = 0.0
        self.linear_increment = 0.1
        self.angular_increment = 0.1
        self.vel_angular_z = 0.0
        self.speed_limit = 1.0
        self.turn_limit = 1.0
        self.key_timeout = 0.5

        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.current_speed = np.array([0.0, 0.0, 0.0])
        self.bindings = {
                            # (key) : (x, y, omega)
                            'w' : np.array([1.0, 0.0, 0.0]),
                            'a' : np.array([0.0, -1.0, 0.0]),
                            's' : np.array([-1.0, 0.0, 0.0]),
                            'd' : np.array([0.0, 1.0, 0.0]),
                            'q' : np.array([0.0, 0.0, 1.0]),
                            'e' : np.array([0.0, 0.0, -1.0]),
                            'x' : np.array([0.0, 0.0, 0.0])
                        }

        self.settings = self.saveTerminalSettings()


    def publish(self, twist_msg):
        self.publisher.publish(twist_msg)

    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.getKey()

        if key in self.bindings.keys():
            if (key == 'x'):
                self.current_speed = np.array([0.0, 0.0, 0.0]) # bindings[key]
            else:
                self.current_speed += self.bindings[key]

            vel_linear_x = min(self.speed_limit, max(self.linear_increment * self.current_speed[0], -self.speed_limit))
            vel_linear_y = min(self.speed_limit, max(self.linear_increment * self.current_speed[1], -self.speed_limit))
            vel_angular_z = min(self.turn_limit, max(self.angular_increment * self.current_speed[2], -self.turn_limit))

            print("Current speed - linear_x: ", vel_linear_x, ", linear_y: ", vel_linear_y, ", angular_z: ", vel_angular_z)

            # print('vel_linear_x: ', vel_linear_x)
            # print('vel_linear_y: ' , vel_linear_y)
            # print('vel_angular_z: ' , vel_angular_z)

            twist_msg = TwistMsg()
            #twist_msg.header.stamp = rospy.Time.now()
            # twist_msg.header.frame_id = twist_frame
            twist_msg.linear.x = vel_linear_x
            twist_msg.linear.y = vel_linear_y
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = vel_angular_z

            self.publisher.publish(twist_msg)
        elif key == 'z':
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    legged_twist_publisher = LeggedTwistPublisher()

    rclpy.spin(legged_twist_publisher)

    legged_twist_publisher.destroy_node()
    rclpy.shutdown()