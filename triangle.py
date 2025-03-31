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
        .setPosition([(i%3)*.15, (i//3)*.15+.1, .71]) \
        .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)
    

C_copy = ry.Config()
C_copy.addConfigurationCopy(C)
qHome = C_copy.getJointState()
C_copy.view(True)



def build_horizontal_triangle():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=False)

    # Get objects
    red_block = env.getObj("block_0")
    green_block = env.getObj("block_1")
    blue_block = env.getObj("block_2")
    
    # Compute positions
    center_x = (red_block.pos.x + green_block.pos.x + blue_block.pos.x) / 3
    center_y = (red_block.pos.y + green_block.pos.y + blue_block.pos.y) / 3
    
    # Define triangle corner placements
    place_positions = [
        (red_block.pos.x, red_block.pos.y , red_block.pos.z, 0.0),
        (green_block.pos.x, green_block.pos.y, green_block.pos.z, 2.094),
        (blue_block.pos.x, blue_block.pos.y, blue_block.pos.z, -2.094)
    ]
    
    # Pick and place the blocks
    for block, (x, y, z, yaw) in zip(["block_0", "block_1", "block_2"], place_positions):
        env.pick(block)
        env.place(x, y, z, rotated=True, yaw=yaw)


build_horizontal_triangle()

C_copy.view(True)