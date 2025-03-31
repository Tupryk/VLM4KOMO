import robotic as ry
import numpy as np
from move_blocks import BlockMover
import numpy as np

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")



for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1.
    C.addFrame(f"block_{i}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .68]) \
        .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)

C.view(True)

C.addFrame("block_4") \
    .setPosition([0, .2, .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0, 1, 0, .3]) \
    .setContact(0) \
    .setMass(.1) \
    .setQuaternion([1/2, 0, 0, np.sqrt(3)/2])


M = BlockMover(C, ["block_0"])
M.testYaw()