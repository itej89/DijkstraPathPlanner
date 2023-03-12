import os
import collections
import copy
import numpy as np

import heapq

from node import *
from node_manager import *
from environment import *
from action_handler import *




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
        self.visited_node_list[agents_postion] = None

        Sim_Pose = None
        Status = False

        #iterate through all possible actions
        for action in self.env.Actions:
            #make a copy of the current agents position

            #Compute agents possible furture position
            Status ,Sim_Pose = self.action_handler.ActionHandlers[action](parent_node)

            #If the future position is in the bounds of the environemnt
            if Status:
                
                NewNode = None

                simulated_position , Total_Cost_To_Come = Sim_Pose
                if simulated_position not in node_manager.global_node_directory:
                    NewNode = node_manager.make_node(simulated_position, Total_Cost_To_Come)
                    NewNode.Parent_Node_hash = parent_node.Node_hash
                
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
        self.Final_Node = None

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

        self.visited_node_list = {}

        #Perform search till a goal state is reached or maximum number of iterations reached
        print("Please wait while searching for the goal state...")
        
        while True:

            next_item = heapq.heappop(self.pending_state_que)

            if next_item!= None:
                next_node = next_item

                if next_node.Node_State == self.goal_state:
                    self.Final_Node = next_node
                    print("\nFound the goal state!!!")
                    return True

                if next_node.Node_State in self.visited_node_list:
                    continue
                else:
                    self.explore_actions(next_node)
            else:
                print("\nUnable to find the goal state!!!")
                return False

    def back_track(self):
        if self.Final_Node != None:
            last_node = self.Final_Node
            trajectory = [last_node.Node_hash]
            while last_node.Parent_Node_hash != None:
                trajectory.append(last_node.Parent_Node_hash)
                last_node = node_manager.global_node_directory[last_node.Parent_Node_hash]

            return trajectory
        return None
    
    def show_exploration(self):
        self.env.highlight_state(start_state)
        self.env.highlight_state(goal_state)
        self.env.refresh_map()
        cv.waitKey(1)
        update_count = 0
        for position in self.visited_node_list:
            self.env.update_map(position)
            self.env.highlight_state(self.initial_state)
            self.env.highlight_state(self.goal_state)
            _environment.refresh_map()

            update_count +=1
            if update_count == 300:
                update_count = 0
                cv.waitKey(1)

    def show_trajectory(self, trajectory):
        for point in trajectory:
            _environment.highlight_point(point)
        cv.waitKey(1)

        _environment.refresh_map()

if __name__ == "__main__":
    start_state = (20, 20)
    goal_state  = (126, 412)
    # goal_state  = (30, 70)

    _environment = environment(250, 600)
    _environment.create_map()


    _actionHandler = action_handler(_environment)

    _dijkstra_planner = dijkstra_planner(_environment, _actionHandler)
    _status =  _dijkstra_planner.find_goal_node(start_state, goal_state)
    if _status:
        trajectory = _dijkstra_planner.back_track()
        if trajectory != None:
            _dijkstra_planner.show_exploration()
            _dijkstra_planner.show_trajectory(trajectory)
            _environment.save_image("Solution.png")
            cv.waitKey(0)