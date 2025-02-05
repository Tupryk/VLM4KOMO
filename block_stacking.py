import cma
import numpy as np
import robotic as ry
from high_level_funcs_old import RobotEnviroment

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")


# Objects
for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1
    if i > 2:
        color[(i+1)%3] = 1   

    C.addFrame(f"block_{i}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .71]) \
        .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)
    

C_copy = ry.Config()
C_copy.addConfigurationCopy(C)

def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=False)
    # Get object parameters
    red_block = env.getObj("block_0")
    green_block = env.getObj("block_1")
    blue_block = env.getObj("block_2")
    
    # Determine positions for vertical support blocks
    support1_x = red_block.pos.x
    support_y = red_block.pos.y
    support_z = red_block.size.z
    
    env.pick("block_0")
    env.place(support1_x, support_y, support_z+.1, rotated=True)
    
    env.pick("block_1")
    env.place(support1_x, support_y, support_z+.15, rotated=True)
    
    env.pick("block_2")
    env.place(support1_x, support_y, support_z+.3, rotated=True)

build_bridge()

C_copy.view(True)