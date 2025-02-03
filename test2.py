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
    env = RobotEnviroment(C_copy, visuals=False, verbose=1, compute_collisions=False)
    # Get object properties
    red_block = env.getObj('block_red')
    green_block = env.getObj('block_green')
    blue_block = env.getObj('block_blue')

    # Determine the blocks to use as vertical supports and the horizontal block
    vertical_block1 = red_block
    vertical_block2 = green_block
    horizontal_block = blue_block

    # Calculate the distance for the vertical blocks
    vertical_spacing = horizontal_block.size.x + -0.15999622037570585  # Add a small buffer (_FLOAT_)

    # Calculate positions for the vertical blocks
    vertical1_x = vertical_block1.pos.x
    vertical1_y = vertical_block1.pos.y

    vertical2_x = vertical1_x + vertical_spacing
    vertical2_y = vertical1_y

    # Calculate position for the horizontal block
    horizontal_x = (vertical1_x + vertical2_x) / 2  # Centered between vertical blocks
    horizontal_y = vertical1_y
    horizontal_z = vertical_block1.size.z + horizontal_block.size.z / 2 + -0.04000220803365792  # On top of vertical blocks

    # Build the bridge
    # Place first vertical block
    env.pick('block_red')
    env.place(vertical1_x, vertical1_y, z=vertical_block1.size.z / 2)

    # Place second vertical block
    env.pick('block_green')
    env.place(vertical2_x, vertical2_y, z=vertical_block2.size.z / 2)

    # Place horizontal block
    env.pick('block_blue')
    env.place(horizontal_x, horizontal_y, z=horizontal_z, rotated=True)

build_bridge()

C_copy.view(True)