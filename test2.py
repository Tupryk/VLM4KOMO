import cma
import numpy as np
import robotic as ry
from high_level_funcs_old import RobotEnviroment

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")

names = ["red", "green", "blue"]

# Objects
for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1.
    C.addFrame(f"block_{names[i]}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .71]) \
        .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)
    

C_copy = ry.Config()
C_copy.addConfigurationCopy(C)

def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=True)
    # Get object parameters
    red_block = env.getObj("block_red")
    green_block = env.getObj("block_green")
    blue_block = env.getObj("block_blue")
    
    # Determine positions for vertical support blocks
    support_x_offset = blue_block.size.x / 2 + red_block.size.x / 2 + 0.021653775891553203
    support1_x = blue_block.pos.x - support_x_offset
    support2_x = blue_block.pos.x + support_x_offset
    support_y = blue_block.pos.y + 0.21266437153304402
    support_z = red_block.size.z / 2  # Place directly on the table
    
    # Place vertical support blocks
    env.pick("block_red")
    env.place(support1_x, support_y, support_z)
    
    env.pick("block_green")
    env.place(support2_x, support_y, support_z)
    
    # Place horizontal block on top
    bridge_z = support_z + red_block.size.z / 2 + blue_block.size.z / 2 + -0.0419929878854249
    env.pick("block_blue")
    env.place(blue_block.pos.x, support_y, bridge_z, rotated=True, yaw=0.09093764421326257)

build_bridge()

C_copy.view(True)