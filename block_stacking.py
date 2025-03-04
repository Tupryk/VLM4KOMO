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


def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=False)

    red_block = env.getObj("block_0")

    support1_x = red_block.pos.x
    support_y = red_block.pos.y
    support_z = red_block.size.z
    
    a=-0.
    b=0.
    
    env.pick(f"block_0")
    env.place(support1_x, support_y, support_z, rotated=True)
    
    env.pick(f"block_1")
    env.place(a+support1_x, support_y, support_z+.07, rotated=True)
    
    env.pick(f"block_2")
    env.place(b+support1_x, support_y, support_z+.12, rotated=True)


build_bridge()

C_copy.view(True)