
from node import *

class node_manager:

    #do not change this value explicitely
    # (for internal id generation purpose only)
    global_node_directory = {}


    #call this funciton before starting the search to reset id
    def initialize():
        node_manager.global_node_directory.clear()


    def make_hash(state):
        return state


    def make_node(state, cost_to_come = 0, cost_to_go = 0):
        hash = node_manager.make_hash(state)
        node_manager.global_node_directory[hash] = node(state, hash, cost_to_come, cost_to_go)
        return node_manager.global_node_directory[hash]

