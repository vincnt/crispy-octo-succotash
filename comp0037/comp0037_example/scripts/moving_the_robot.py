#!/usr/bin/env python

import rospy
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi, sin, cos
import math
from PyKDL import Rotation
import sys
import os
import numpy as np

class stdr_controller():

    def __init__(self, waypoint_file_location):
        #Creating our node,publisher and subscriber
        rospy.init_node('stdr_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_pose_callback)
        self.current_pose = Odometry()
        self.distance_tolerance = 0.01
        self.waypoint_file_location = waypoint_file_location
        with open(self.waypoint_file_location, "r") as w:
            self.waypoints = [line.strip() for line in w.readlines()]
        print('waypoints', self.waypoints)
        self.current_goal = [float(x) for x in self.waypoints[0].split()]
 
    def current_pose_callback(self, data):
        self.current_pose = data
        pose = self.current_pose.pose.pose
        self.current_position_x = pose.position.x
        self.current_position_y = pose.position.y
        orientation = pose.orientation
        theta = 2 * atan2(orientation.z, orientation.w) * 180 / pi
        self.current_theta = theta * pi/180
        # Show the output
        rospy.loginfo('Current position, x: {}, y:{}, theta:{}'.format(self.current_position_x,
            self.current_position_y, self.current_theta))


    def run(self):
        # Sleep for 1s before starting. This gives time for all the parts of stdr to start up
        rospy.sleep(1.0)
        self.vel_msg = Twist()
        for point in self.waypoints:
            curr = [float(x) for x in point.split()]
            rospy.loginfo('New Waypoint, x: {}, y:{}, theta:{}'.format(curr[0],curr[1],curr[2]))
            self.move_to_point(curr[0],curr[1],curr[2]*pi/180)
            rospy.sleep(1.0)
        self.stop()

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(1)

        
    def move_to_point(self, x_goal, y_goal, theta_goal):
        x_diff = x_goal - self.current_position_x
        y_diff = y_goal - self.current_position_y
        theta_diff = atan2(y_diff, x_diff)
        print('theta_diff', theta_diff)
        r = rospy.Rate(10)

        # Orientate
        while not rospy.is_shutdown():
            if abs(self.current_theta - theta_diff) < 0.01:
                break
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = theta_diff - self.current_theta 
            self.velocity_publisher.publish(self.vel_msg)
            r.sleep()
        self.stop()
        rospy.loginfo('Orientating done')

        while not rospy.is_shutdown():
            x_diff = x_goal - self.current_position_x
            y_diff = y_goal - self.current_position_y
            theta_diff = atan2(y_diff, x_diff)
            distance = sqrt(x_diff**2 + y_diff**2)
            print('distance left', distance)
            if abs(distance) < 0.1:
                break
            self.vel_msg.linear.x = distance
            self.vel_msg.angular.z = theta_diff - self.current_theta
            self.velocity_publisher.publish(self.vel_msg)
            r.sleep()
        self.stop()
        rospy.loginfo('Moving done')

        # Orientate to goal
        while not rospy.is_shutdown():
            if abs(self.current_theta - theta_goal) < 0.087:
                break
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = theta_goal - self.current_theta
            self.velocity_publisher.publish(self.vel_msg)
            r.sleep()
        self.stop()
        rospy.loginfo('Orientated to goal')


if __name__ == '__main__':
    print("Waypoint arg location: {}".format(sys.argv[1]))
    try:
        #Testing our function
        x = stdr_controller(sys.argv[1])
        x.run()

    except rospy.ROSInterruptException: pass
