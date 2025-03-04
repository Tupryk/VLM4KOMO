import cma
import numpy as np
import robotic as ry
from high_level_funcs_old import RobotEnviroment

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")


C_copy = ry.Config()
C_copy.addConfigurationCopy(C)
qHome = C_copy.getJointState()

def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=True)
    
    # Determine positions for vertical support blocks
    support1_x = -.1
    support_y = .15
    support_z = .71
    

    for i in range(5):
        env.C.addFrame(f"block_{i}") \
        .setPosition([.2, .15, .71]) \
        .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
        .setColor([0,1,1]) \
        .setContact(1) \
        .setMass(.1)
        env.pick(f"block_{i}")
        env.place(support1_x-0.03517805, support_y+0.21879074, support_z+.02+.05*i, rotated=True)
        env.C.setJointState(qHome)

build_bridge()

C_copy.view(True)