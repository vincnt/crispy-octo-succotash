#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from math import pow, atan2, sqrt
from comp0037_planner_controller.planned_path import PlannedPath
from comp0037_planner_controller.controller_base import ControllerBase
import math
import angles


# This sample controller works a fairly simple way. It figures out
# where the goal is. It first turns the robot until it's roughly in
# the correct direction and then keeps driving. It monitors the
# angular error and trims it as it goes.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)

        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)

        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))

        # PID stuff
        overshooot = 0.6
        self.distancePErrorGain = 0.3  #
        self.distanceIErrorGain = 0.1
        self.distanceDErrorGain = 1  # increasing this means slowing down response time

        self.anglePErrorGain = 1
        self.angleIErrorGain = 0.5
        self.angleDErrorGain = 1

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0 * math.pi
        elif (delta > math.pi):
            delta = delta - 2.0 * math.pi
        return delta

    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))

        while (distanceError >= self.distanceErrorTolerance) & (not rospy.is_shutdown()):
            # print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
            #                                                                           self.pose.theta, waypoint[0],
            #                                                                           waypoint[1]))
            # print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))

            # Proportional Controller
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * distanceError, 10.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

            # print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()

            self.rate.sleep()

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Make sure the robot is stopped once we reach the destination.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

    def rotateToGoalOrientation(self, goalOrientation):
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        while (math.fabs(angleError) >= self.goalAngleErrorTolerance) & (not rospy.is_shutdown()):
            # print 'Angular Error: ' + str(angleError)

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()

            self.rate.sleep()
            angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        # Stop movement once finished
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

    def driveToWaypointPID(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))

        # needed for PID
        self.angleErrors = [angleError]
        self.distanceErrors = [distanceError]

        while (distanceError >= self.distanceErrorTolerance) & (not rospy.is_shutdown()):
            # print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
            #                                                                           self.pose.theta, waypoint[0],
            #                                                                           waypoint[1]))
            # print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))

            # Proportional Controller
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                pidValueDistance = self.getPIDValueDistance(distanceError)
                print "PID VALUE DISTANCE: ", pidValueDistance
                vel_msg.linear.x = max(0.0, min(pidValueDistance, 4.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            pidValueAngle = self.getPIDValueAngle(angleError)
            print "PID VALUE ANGLE: ", pidValueAngle
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(pidValueAngle, 5.0))

            # print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()

            self.rate.sleep()

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Make sure the robot is stopped once we reach the destination.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

    def getPIDValueDistance(self, distanceError):

        PValue = self.distancePErrorGain * distanceError

        # n = 15
        # if len(self.distanceErrors) > n:
        #     IValue = self.distanceIErrorGain * sum(self.distanceErrors[-n:]) / n
        # else:
        IValue = self.distanceIErrorGain * sum(self.distanceErrors) / len(self.distanceErrors)

        if len(self.distanceErrors) >= 1:
            gradient = self.distanceErrors[-1] - distanceError
        else:
            gradient = 0

        DValue = self.distanceDErrorGain * gradient

        self.distanceErrors.append(distanceError)
        print "P: %f; I: %f; D: %f" % (PValue, IValue, DValue)
        return PValue + IValue + DValue

    def getPIDValueAngle(self, angleError):
        PValue = self.anglePErrorGain * angleError
        IValue = sum(self.angleErrors) / len(self.angleErrors)

        if len(self.angleErrors) >= 1:
            gradient = self.angleErrors[-1] - angleError
        else:
            gradient = 0

        DValue = self.angleDErrorGain * gradient

        self.angleErrors.append(angleError)

        return PValue + IValue + DValue
