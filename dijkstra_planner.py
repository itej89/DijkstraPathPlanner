import os
import collections
import copy
import numpy as np

from node import *
from node_manager import *

# from queue import PriorityQueue, Empty

import heapq

from environment import *


class ActionHandler:

    def __init__(self, env) -> None:
        self.env = env

        self.ActionHandlers = {
            "LEFT" : self.ActionMoveLeft,
            "RIGHT" : self.ActionMoveRight,
            "UP" : self.ActionMoveUp,
            "DOWN" : self.ActionMoveDown,
            "UP_LEFT" : self.ActionMoveUpLeft,
            "UP_RIGHT" : self.ActionMoveUpRight,
            "DOWN_RIGHT" : self.ActionMoveDownLeft,
            "DOWN_LEFT" : self.ActionMoveDownRight,
        }

    def position_to_node(self, parent_node, simulated_position, action_cost):

        if not self.env.is_valid_position(simulated_position):
            return False, None
        
        return True, (simulated_position, action_cost)
        
  

    def PerformAction(self, parent_node, action):
        agents_postion = parent_node.Node_State
        simulated_position = (agents_postion[0] + self.env.ActionValues[action][0], 
                                agents_postion[1] + self.env.ActionValues[action][1])
        action_cost = parent_node.Cost_to_Come + self.env.ActionCost[action]
        return self.position_to_node(parent_node, simulated_position, action_cost)

    def ActionMoveLeft(self, parent_node):
        return self.PerformAction(parent_node, "LEFT")
        
    def ActionMoveRight(self, parent_node):
        return self.PerformAction(parent_node, "RIGHT")
    
    def ActionMoveUp(self, parent_node):
        return self.PerformAction(parent_node, "UP")
    
    def ActionMoveDown(self, parent_node):
        return self.PerformAction(parent_node, "DOWN")
    
    def ActionMoveUpLeft(self, parent_node):
        return self.PerformAction(parent_node, "UP_LEFT")
    
    def ActionMoveUpRight(self, parent_node):
        return self.PerformAction(parent_node, "UP_RIGHT")
    
    def ActionMoveDownLeft(self, parent_node):
        return self.PerformAction(parent_node, "DOWN_LEFT")
    
    def ActionMoveDownRight(self, parent_node):
        return self.PerformAction(parent_node, "DOWN_RIGHT")


class dijkstra_planner:

    def __init__(self, env, action_handler) -> None:
        self.env = env
        self.action_handler = action_handler
        
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
        self.visited_node_list.add(agents_postion)
        self.env.update_map(agents_postion)

        Sim_Pose = None
        Status = False

        #iterate through all possible actions
        for action in self.env.Actions:
            #make a copy of the current agents position

            #Compute agents possible furture position
            if action == "LEFT":
                Status ,Sim_Pose = self.action_handler.ActionMoveLeft(parent_node)
            elif action == "RIGHT":
                Status ,Sim_Pose = self.action_handler.ActionMoveRight(parent_node)
            elif action == "UP":
                Status ,Sim_Pose = self.action_handler.ActionMoveUp(parent_node)
            elif action == "DOWN":
                Status ,Sim_Pose = self.action_handler.ActionMoveDown(parent_node)
            elif action == "UP_LEFT":
                Status ,Sim_Pose = self.action_handler.ActionMoveUpLeft(parent_node)
            elif action == "UP_RIGHT":
                Status ,Sim_Pose = self.action_handler.ActionMoveUpRight(parent_node)
            elif action == "DOWN_LEFT":
                Status ,Sim_Pose = self.action_handler.ActionMoveDownLeft(parent_node)
            elif action == "DOWN_RIGHT":
                Status ,Sim_Pose = self.action_handler.ActionMoveDownRight(parent_node)

            #If the future position is in the bounds of the environemnt
            if Status:
                
                NewNode = None

                simulated_position , Total_Cost_To_Come = Sim_Pose
                if simulated_position not in node_manager.global_node_directory:
                    NewNode = node_manager.make_node(simulated_position, Total_Cost_To_Come)
                
                elif simulated_position in self.visited_node_list:
                    NewNode = None
                
                else:
                    if node_manager.global_node_directory[simulated_position].Cost_to_Come > Total_Cost_To_Come:
                        node_manager.global_node_directory[simulated_position].Cost_to_Come = Total_Cost_To_Come
                        node_manager.global_node_directory[simulated_position].Parent_Node_hash = parent_node.Node_hash
                        NewNode = node_manager.global_node_directory[simulated_position]

                if NewNode != None:
                    #Found an unique state that needs to be pushed in to the que
                    NewNode.TRANSITION_ACTION = action
                    heapq.heappush(self.pending_state_que, NewNode)

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

        if not self.env.is_valid_position(start_state):
            print(f"Invalid start state.")
            return

        if not self.env.is_valid_position(goal_state):
            print(f"Invalid goal state.")
            return

        #Initialize search que. This stores the nodes that needs to be explored
        start_node = node_manager.make_node(self.initial_state)
        self.pending_state_que = [start_node]
        heapq.heapify(self.pending_state_que)

        self.visited_node_list = set()

        self.env.show_map()
        #Perform search till a goal state is reached or maximum number of iterations reached
        print("Please wait while searching for the goal state...")
        
        while True:
            
            next_item = heapq.heappop(self.pending_state_que)

            if next_item!= None:
                next_node = next_item

                # if next_node.Node_State[0] == 142 and next_node.Node_State[1] == 595:
                #     i =0

                # self.env.save_image()
                print(f"Pending : {len(self.pending_state_que)}")
                print(f"Priority: {next_node.Cost_to_Come}, Node : {next_node.Node_State}")

                if next_node.Node_State == self.goal_state:
                    print("\nFound the goal state!!!")

                    self.env.save_image()
                    return True

                if next_node.Node_State in self.visited_node_list:
                    continue
                else:
                    self.explore_actions(next_node)
            else:
                print("\nUnable to find the goal state!!!")
                return False



if __name__ == "__main__":
    _environment = environment(250, 600)
    _environment.create_map()
    _actionHandler = ActionHandler(_environment)
    _dijkstra_planner = dijkstra_planner(_environment, _actionHandler)
    _dijkstra_planner.find_goal_node((20,20), (137,590))
