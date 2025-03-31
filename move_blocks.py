import robotic as ry
from high_level_funcs_old import RobotEnviroment
from utils import relative_yaw
import numpy as np

class BlockMover():
    
    def __init__(self, C: ry.Config, block_list: list[str]):
        self.C = C
        self.block_list = block_list
        self.feasible_motion = False

    def moveTo(self, C):
        x_off = np.linspace(-.2, .2, 30)
        y_off = np.linspace(.2, .3, 30)
        feasibleMotions = []

        for xoff in x_off:
            for yoff in y_off:
                C_cp = ry.Config()
                C_cp.addConfigurationCopy(self.C)
                
                motionFeasible = True
                for block in self.block_list:
                    RoboEnv = RobotEnviroment(C_cp, False , 0, True, False, False, False, False)
                    pickFeasible = RoboEnv.pick(block)
                    placeFeasible = RoboEnv.place3(C, block, xoff, yoff)

                    if not (pickFeasible and placeFeasible):
                        motionFeasible = False
                        break

                if motionFeasible:
                    C_cp.view(True)
                    feasibleMotions.append((xoff, yoff))
                del C_cp
        
        print(feasibleMotions)
        

    def testYaw(self):
        for i in range(8):
            C_cp = ry.Config()
            C_cp.addConfigurationCopy(self.C)

            RoboEnv = RobotEnviroment(C_cp, True , 0, True, False, False, False, True)
            RoboEnv.pick(self.block_list[0])
            #RoboEnv.place2(0, 0, .35, i*np.pi/4- np.pi/2)
            RoboEnv.place3(self.C, "block_2")

            del C_cp

