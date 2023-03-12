import os
import collections
import copy
import numpy as np

#Class to define a tree node contents
class node:

    def __init__(self, state, hash, cost_to_come, cost_to_go) -> None:
        """Constructor for node object

        Args:
            state (numpy array): state of the environemtn
        """

        # an unique id is generated for the given state by taking the 
        # numpy array ordered elements in sequence
        self.Node_hash = hash

        #node's parent id
        # contains "None" if the node is top of the tree
        self.Parent_Node_hash = None

        #Contains state that this node represents
        self.Node_State = state
        
        #Contains action taken by agent to reach the state
        self.TRANSITION_ACTION = "START"

        #Contains cost-to-come to the node
        self.Cost_to_Come = cost_to_come

        #Contains cost-to-Go to the goal
        self.Cost_to_Go = cost_to_go

    #Funciton to help heapq for comparing two node objects
    def __lt__(self, other):
        return self.Cost_to_Come < other.Cost_to_Come