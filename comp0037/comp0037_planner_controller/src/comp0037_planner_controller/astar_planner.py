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
        self.astarQueue = []
        self.heuristic_constant = heuristic_constant
        self.heuristic_type = heuristic_type
        
    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        heuristic_cost = self.calculateHeuristic(cell) + cell.pathCost
        heappush(self.astarQueue, (heuristic_cost, cell))
        
    # (!) new method that returns the length of the queue
    def getQueueLength(self):
        return len(self.astarQueue)

    # Check the queue size is zeros
    def isQueueEmpty(self):
        return not self.astarQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.astarQueue)
        return cell[1]

    def calculateHeuristic(self, cell):
        if self.heuristic_type == 'constant':
            heuristic_cost = 1
        
        elif self.heuristic_type == 'euclidean':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = sqrt(dx**2 + dy**2)
        
        elif self.heuristic_type == 'octile':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = max(dx,dy) + (sqrt(2) - 1) * min(dx,dy)

        elif self.heuristic_type == 'manhattan':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = dx + dy

        elif self.heuristic_type == 'octile_scaled':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = max(dx,dy) + (sqrt(2) - 1) * min(dx,dy)
            heuristic_cost *= (1 + 1/150)

        elif self.heuristic_type == 'chebyshev':
            dx = abs(cell.coords[0] - self.goal.coords[0])
            dy = abs(cell.coords[1] - self.goal.coords[1])
            heuristic_cost = max(dx,dy) 
            
        return heuristic_cost * self.heuristic_constant

    def calculateCost(self, cell, parentCell):
        cumulative_cost = parentCell.pathCost
        next_cost = self.computeLStageAdditiveCost(parentCell, cell)
        total_cost = cumulative_cost + next_cost 
        cost_with_heuristic = total_cost + self.calculateHeuristic(cell)
        return total_cost, cost_with_heuristic

    def resolveDuplicate(self, cell, parentCell):
        total_cost, cost_with_heuristic = self.calculateCost(cell, parentCell)

        # update queue with new priority value of x 
        for idx, i_cell in enumerate(self.astarQueue):
            if i_cell[1].coords == cell.coords:
                if cost_with_heuristic < i_cell[0]:
                    cell.parent = parentCell
                    cell.pathCost = total_cost
                    self.astarQueue[idx] = (cost_with_heuristic, cell)
                break
        # re-sort heap based on newly added value
        heapify(self.astarQueue)

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        if parentCell is None:
            total_cost = 0
        else:
            total_cost, _ = self.calculateCost(cell, parentCell)
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell      
        cell.pathCost = total_cost