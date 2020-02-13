# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop, heapify
from math import sqrt
from cell import *
# add comments

class AstarPlanner(CellBasedForwardSearch):
    # Construct the new planner object
    def __init__(self, title, occupancyGrid, heuristic_type, heuristic_constant = 1):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijkstraQueue = []
        self.heuristic_constant = heuristic_constant
        self.heuristic_type = heuristic_type
        
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

    def calculateCost(self, cell, parentCell):
        cumulative_cost = parentCell.pathCost
        next_cost = self.computeLStageAdditiveCost(parentCell, cell)
        total_cost = cumulative_cost + next_cost 

        if self.heuristic_type == 'constant':
            total_cost += self.heuristic_constant
        
        elif self.heuristic_type == 'euclidean':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = sqrt(dx**2 + dy**2)
            total_cost += self.heuristic_constant * heuristic_cost
        
        elif self.heuristic_type == 'octile':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = max(dx,dy) + (sqrt(2) - 1) * min(dx,dy)
            total_cost += self.heuristic_constant * heuristic_cost

        elif self.heuristic_type == 'manhattan':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = dx + dy
            total_cost += self.heuristic_constant * heuristic_cost

        return total_cost

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        total_cost = self.calculateCost(cell, parentCell)
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
            total_cost = self.calculateCost(cell, parentCell)
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell      
        cell.pathCost = total_cost