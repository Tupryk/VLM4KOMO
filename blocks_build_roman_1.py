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
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=True)

    # Get block objects
    red_block = env.getObj("block_0")
    green_block = env.getObj("block_1")
    blue_block = env.getObj("block_2")
    
    # Compute placement positions based on object sizes
    center_x, center_y = red_block.pos.x, red_block.pos.y
    
    # Placeholder optimization values
    horizontal_offset = red_block.size.x * 1
    vertical_offset = green_block.size.z * 1
    
    # Position for the bottom horizontal block
    bottom_x, bottom_y = center_x, center_y
    
    # Position for the vertical block
    vertical_x, vertical_y = center_x, center_y
    vertical_z = red_block.size.z + vertical_offset  # Adjust placement height
    
    # Position for the top horizontal block
    top_x, top_y = center_x, center_y
    top_z = vertical_z + green_block.size.z + horizontal_offset  # Adjust top block height
    
    # Pick and place the bottom horizontal block
    env.pick("block_0")
    env.place(bottom_x, bottom_y, rotated=True)
    
    # Pick and place the vertical block
    env.pick("block_1")
    env.place(vertical_x, vertical_y, z=vertical_z, rotated=False)
    
    # Pick and place the top horizontal block
    env.pick("block_2")
    env.place(top_x, top_y, z=top_z, rotated=True)



build_bridge()

C_copy.view(True)