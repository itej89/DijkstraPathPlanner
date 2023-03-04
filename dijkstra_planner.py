import os
import collections
import copy
import numpy as np

from node import *
from node_manager import *

from queue import PriorityQueue, Empty

class dijkstra_planner:

    def explore_actions(self, parent_node) -> bool:
        """Function takes an environmet state and runs 
        possible actions for that state and extract 
        possible unique future states and stores them in the que

        Args:
            parent_node (node): node representing a state of the
            environment

        Returns:
            bool: returns True if goal state is found, False otherwise
        """

        #get the position of the agent location
        # this represents the (x,y) position on the grid
        agents_postion = parent_node.Node_State

        #move the input node into explored list
        parent_node.IsExplored = True

        NewNode = None
        Status = False

        #iterate through all possible actions
        for action in self.action_list:
            #make a copy of the current agents position
            simulated_position = copy.deepcopy(agents_postion)

            #Compute agents possible furture position
            if action == "LEFT":
                Status ,NewNode = self.ActionMoveLeft(parent_node)
            elif action == "RIGHT":
                Status ,NewNode = self.ActionMoveRight(parent_node)
            elif action == "UP":
                Status ,NewNode = self.ActionMoveUp(parent_node)
            elif action == "DOWN":
                Status ,NewNode = self.ActionMoveDown(parent_node)

            #If the future position is in the bounds of the environemnt
            if Status:

                #Found an unique state that needs to be pushed in to the que
                NewNode.TRANSITION_ACTION = action
                self.pending_state_que.append(NewNode)

                #Check if the computed stae is the goal state
                # if True return
                if np.array_equal(NewNode.Node_State_i, self.goal_state):
                    return True
                    
            
        return False
    
    def find_goal_node(self, start_state, goal_state) -> bool:
        """Takes start stat and goal state for the environement 
        and computes the tree using _Breadth first search algorithm till the goal state is reached 

        Args:
            start_state (2D numpy array): Start state for the environment
            goal_state (2D numpy array): Goal state for the environment
        """

        #Call this fuction for auto id generation of nodes for a new tree
        node_manager.initialize()

        #initailize states
        self.goal_state = goal_state
        self.initial_state = start_state

        #Initialize search que. This stores the nodes that needs to be explored
        self.pending_state_que = PriorityQueue()
        self.pending_state_que.put(node_manager.make_node(self.initial_state))

        #Perform search till a goal state is reached or maximum number of iterations reached
        print("Please wait while searching for the goal state")
        while not self.explore_actions(self.pending_state_que.popleft()):
            print('\r'+f"Number of states explored : {len(self.explored_state_que)}", end='')
            continue

        print("\nFound goal state!!!")
        return True


