import numpy as np
import math

'''Class of Node to store state, parent, move made, and cost of each move'''

class Node():
    def __init__(self, state, parent, move, cost): 
        # creating tuple with cost to come, index, parent node index=0 and coordinate values (x,y) 
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = math.inf
        
    def getState(self):
        return self.state
		
    def getParent(self):
        return self.parent

    def getParentState(self):
        if self.getParent() is None:
            return None
        return self.getParent().getState()
		
    def getMove(self):
	    return self.move
		
    def getCost(self):
        return self.cost

    def getFullPath(self):
        
        moves = []
        nodes = []
        current_node = self
        while(current_node.getMove() is not None):

            moves.append(current_node.getMove())
            nodes.append(current_node)
            current_node = current_node.getParent()

        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        
        return moves, nodes