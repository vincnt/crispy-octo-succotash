# -*- coding: utf-8 -*-

# The planned path the robot will take. This consists of a set of waypoints.
from collections import deque
from math import pi, atan2


class PlannedPath(object):

    # Construct a new planner object and set defaults.
    def __init__(self):
        # Does the path actually reach the goal or not?
        self.goalReached = False

        # The list of waypoints, from start to finish, which make up the path.
        # The type of data stored here depends on the 
        self.waypoints = deque()

        # Performance information - number of waypoints, and the
        # travel cost of the path.
        self.numberOfWaypoints = 0
        self.travelCost = 0
        # (!) total angle
        self.totalAngle = 0

    def calculateTotalAngle(self):
        q = list(self.waypoints)
        ox, oy = q[0].coords
        prevAngle = 0  # assuming the robot always starts out looking the same way
        # the last waypoint is
        for waypoint in q:
            x, y = waypoint.coords
            dx, dy = x - ox, y - oy
            angle = atan2(dy, dx) * 180 / pi

            delta = abs(angle - prevAngle)

            if delta > 180:
                delta = 360 - delta

            self.totalAngle += delta

            prevAngle = angle
            ox, oy = x, y
