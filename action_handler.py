class action_handler:
#Class to handle the action supported by the environment


    def __init__(self, env) -> None:
        """Initilaize action handler parameters

        Args:
            env (_type_): _description_
        """
        self.env = env

        #Define action handler dictionary for each type of action supported
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