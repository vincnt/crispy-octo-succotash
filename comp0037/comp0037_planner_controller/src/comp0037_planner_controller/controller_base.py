#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi
from planned_path import PlannedPath
import time
import math
import pickle as pkl
import numpy as np


# This is the base class of the controller which moves the robot to its goal.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid

        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

        self.distanceTravelled = 0
        self.angleTurned = 0
        self.prevX = 0
        self.prevY = 0
        self.prevTheta = 0
        self.firstTime = True
        self.notTeleportGoal = False

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation

        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose
        if self.notTeleportGoal:
            if self.firstTime:
                self.prevX = pose.x
                self.prevY = pose.y
                self.prevTheta = pose.theta
                self.firstTime = False
            dDistance = sqrt((pose.x - self.prevX) ** 2 + (pose.y - self.prevY) ** 2)
            self.distanceTravelled += dDistance
            self.angleTurned += (abs(pose.theta - self.prevTheta))
            self.prevX = pose.x
            self.prevY = pose.y
            self.prevTheta = pose.theta
            # print('distance Travelled: {}  angle turned: {}'.format(self.distanceTravelled, self.angleTurned))

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        self.plannerDrawer = plannerDrawer
        self.notTeleportGoal = True

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')

        new_waypoints =  self.reduceWaypoints(path.waypoints)
        print "new waypoints: ", new_waypoints
        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(new_waypoints)):
            cell = new_waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

    def reduceWaypoints(self, waypoints):
        new_waypoints = [waypoints[0]]
        ox,oy = waypoints[0].coords
        prevAngle = 0
        print('old waypoints', len(waypoints))

        for cell in list(waypoints)[1:]:
            x,y = cell.coords
            dx, dy = x- ox, y - oy
            angle = atan2(dy, dx) 
            if angle != prevAngle:
                new_waypoints.append(cell)
            prevAngle = angle
            ox,oy = x,y
        new_waypoints.append(list(waypoints)[-1])
        return new_waypoints 