import unittest
import numpy as np

class obstacle_model:

    def __init__(self, _obstacles) -> None:
        self.obstacles = _obstacles 

    def inflate_obstacles(self, scale):
        for verticies in self.obstacles:
            mean  = np.mean(verticies, axis=0)
            for i in range(len(verticies)):
                verticies[0] = mean[0] + scale*(verticies[0] - mean[0])
                verticies[1] = mean[1] + scale*(verticies[1] - mean[1])
    
    def check_obstacle(self, point):
        
        for verticies in self.obstacles:
            is_inside_obstacle = True
            for i in range(len(verticies)):
                if is_inside_obstacle:

                    if i == len(verticies) - 1:
                        x2, y2 = verticies[i][0], verticies[i][1]
                        x1, y1 = verticies[0][0], verticies[0][1]
                    else:
                        x2, y2 = verticies[i][0], verticies[i][1]
                        x1, y1 = verticies[i+1][0], verticies[i+1][1]

                    a = (y1 - y2)
                    b = (x1 - x2) * -1
                    c = x1*y2 - x2*y1
                    
                    l = a*point[0] + b*point[1] + c

                    if  l > 0:
                        is_inside_obstacle = False
                        break

            
            if is_inside_obstacle:
                return True
        
        return False



class obstacle_model_test(unittest.TestCase):
    def setUp(self):
        self.obstacleModels = obstacle_model([ [(60, 0), (60, 50), (50,50), (50,0) ] ])

    def test_free_space(self):
        self.assertEqual(self.obstacleModels.check_obstacle((70,70)), False, "free_space error!!")
    
    def test_obstacle(self):
        self.assertEqual(self.obstacleModels.check_obstacle((55,40)), True, "obstacle test failed!!")
    
    def tearDown(self):
        pass



if __name__ == '__main__':
    unittest.main()