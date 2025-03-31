import numpy as np
import rowan
import robotic as ry



C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")


C.addFrame("block_0") \
    .setPosition([0.2085052, 0.03640654, .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([1,0,0]) \
    .setContact(1) \
    .setMass(.1) \

C.addFrame("block_1") \
    .setPosition([0.12056464, 0.08773839, .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0,1,0]) \
    .setContact(1) \
    .setMass(.1) \
    .setQuaternion([1/2, 0, 0, np.sqrt(3)/2])

C.addFrame("block_2") \
    .setPosition([ 0.12039511, -0.01489812,  .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0,0,1]) \
    .setContact(1) \
    .setMass(.1) \
    .setQuaternion([1/2, 0, 0, -np.sqrt(3)/2])



# Given quaternion (x, y, z, w)
quat = np.array([1/2, 0, 0, np.sqrt(3)/2])  # Example quaternion

# Define a quaternion representing a 180-degree rotation around Z-axis
angle = np.pi  # 180 degrees in radians
rotation_quat = rowan.from_axis_angle([0, 0, 1], angle)

# Rotate the given quaternion
rotated_quat = rowan.multiply(rotation_quat, quat)

print("Original Quaternion:", quat)
print("Rotated Quaternion:", rotated_quat)

C.addFrame("block_3") \
    .setPosition([ 0.32039511, -0.01489812,  .68]) \
    .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
    .setColor([0,1,1]) \
    .setContact(1) \
    .setMass(.1) \
    .setQuaternion(rotated_quat)

C.view(True)

print(C.eval(ry.FS.quaternionDiff, ["block_1", "block_3"])[0])