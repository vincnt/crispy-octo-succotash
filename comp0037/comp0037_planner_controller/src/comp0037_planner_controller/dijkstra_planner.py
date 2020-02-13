# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop, heapify
from math import sqrt
from cell import *
# add comments

class DijkstraPlanner(CellBasedForwardSearch):
    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijkstraQueue = []

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        heappush(self.dijkstraQueue, (cell.pathCost, cell))

    # Check the queue size is zeros
    def isQueueEmpty(self):
        return not self.dijkstraQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.dijkstraQueue)
        return cell[1]

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        cumulative_cost = parentCell.pathCost
        next_cost = self.computeLStageAdditiveCost(parentCell, cell)
        total_cost = cumulative_cost + next_cost
        if total_cost < cell.pathCost:
            cell.parent = parentCell
            cell.pathCost = total_cost
            # update queue with new priority value of x 
            for idx, i_cell in enumerate(self.dijkstraQueue):
                if i_cell[1].coords == cell.coords:
                    self.dijkstraQueue[idx] = (cell.pathCost, cell)
                    break
            # re-sort heap based on newly added value
            heapify(self.dijkstraQueue)

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        if parentCell is None:
            total_cost = 0
        else:
            cumulative_cost = parentCell.pathCost
            next_cost = self.computeLStageAdditiveCost(parentCell, cell)
            total_cost = cumulative_cost + next_cost
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell      
        cell.pathCost = total_cost