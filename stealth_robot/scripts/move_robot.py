#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pi import Pi


class RobotMover:
    def __init__(self):
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def get_start_time(self):
        """Get initial time t0 in seconds"""
        t0 = 0
        while not t0:
            t0 = rospy.Time.now().to_sec()
        return t0

    def get_current_time(self):
        """Get current time in seconds"""
        return rospy.Time.now().to_sec()

    def x_forward(self, forward=True):
        """Set up velocity message for moving straight ahead"""
        vel_msg = Twist()

        # set direction according to forward
        if forward:
            vel_msg.linear.x = 1
        else:
            vel_msg.linear.x = -1

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        return vel_msg

    def z_angular(self, clockwise=True):
        """Set up message with angular speed (used for rotation)"""
        if clockwise:
            angular_speed = -1
        else:
            angular_speed = 1

        vel_msg = Twist()

        # no linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # set angular speed
        vel_msg.angular.z = angular_speed

        return vel_msg

    def stop_message(self):
        """Set up message with no speed set in order to stop the robot"""
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        return vel_msg

    def publish(self, msg):
        """Publish a given message to /cmd_vel topic"""
        self.vel_publisher.publish(msg)

    def stop_robot(self):
        """Publish stop message in order to stop the robot"""
        self.vel_publisher.publish(self.stop_message())

    def move(self, distance):
        """Move a given distance forward"""

        # create velocity message object
        vel_msg = Twist()

        if distance < 0:
            # move backwards for negative distance
            vel_msg = self.x_forward(forward=False)
        else:
            vel_msg = self.x_forward()

        t0 = self.get_start_time()

        current_distance = 0
        # set distance to absolute value for comparisons
        distance = abs(distance)

        # move robot a specified distance
        while current_distance < distance:
            # publish velocity
            self.publish(vel_msg)
            t1 = self.get_current_time()
            current_distance = t1 - t0

        self.stop_robot()

    def rotate(self, angle, clockwise=True):
        """Rotate by a given (positive) angle"""
        if not clockwise:
            vel_msg = self.z_angular(clockwise=False)
        else:
            vel_msg = self.z_angular()

        t0 = self.get_start_time()
        current_angle = 0
        # set angle to absolute value for comparison

        while current_angle < angle:
            self.publish(vel_msg)
            t1 = self.get_current_time()
            current_angle = t1 - t0

        self.stop_robot()
