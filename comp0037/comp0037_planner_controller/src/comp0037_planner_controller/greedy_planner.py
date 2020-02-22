# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop
from math import sqrt

# add comments

class GreedyPlanner(CellBasedForwardSearch):
    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = []

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        cellCoords = cell.coords
        goalCoords = self.goal.coords
        distance = sqrt((cellCoords[0] - goalCoords[0])**2 + (cellCoords[1] - goalCoords[1])**2)
        heappush(self.greedyQueue, (distance, cell))

    # (!) new method that returns the length of the queue
    def getQueueLength(self):
        return len(self.greedyQueue)

    # Check the queue size is zeros
    def isQueueEmpty(self):
        return not self.greedyQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.greedyQueue)
        return cell[1]

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
