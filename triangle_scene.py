import robotic as ry
import numpy as np
from move_blocks import BlockMover

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

C2 = ry.Config()
C2.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C2.delFrame("panda_collCameraWrist")

#x_off = -.25
#y_off = .1


x_off = 0 
y_off = 0

C2.addFrame(f"block_0") \
    .setPosition([0.2085052+x_off, 0.03640654+y_off, .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([1,0,0]) \
    .setContact(1) \
    .setMass(.1) \

C2.addFrame(f"block_1") \
    .setPosition([0.12056464+x_off, 0.08773839+y_off, .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0,1,0]) \
    .setContact(1) \
    .setMass(.1) \
    .setQuaternion([1/2, 0, 0, np.sqrt(3)/2])

C2.addFrame(f"block_2") \
    .setPosition([ 0.12039511+x_off, -0.01489812+y_off,  .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0,0,1]) \
    .setContact(1) \
    .setMass(.1) \
    .setQuaternion([1/2, 0, 0, -np.sqrt(3)/2])

C2.view(True)

M = BlockMover(C, ["block_0", "block_1", "block_2"])
M.moveTo(C2)