import robotic as ry
import numpy as np
import time
from robotic import SimulationEngine

C = ry.Config()
C.addFile(ry.raiPath("./scenarios/pandaSingle.g"))

midpoint = np.array([-0.105, 0.2, 0.745])

C.addFrame("target") \
    .setPosition(midpoint+np.array([-.22,.2,0])) \
    .setShape(ry.ST.box, size=[0.21, .36, .15]) \
    .setColor([28/255, 18/255, 210/255, .3]) \

def sample_rectangular_arena(width=0.4, height=0.4, z_coord=0.745, center_point=[0, 0]):
    x = center_point[0] + np.random.uniform(-width / 2, width / 2)
    y = center_point[1] + np.random.uniform(-height / 2, height / 2)
    return [x, y, z_coord]

midpoint = sample_rectangular_arena(width=.68, height=.6, center_point=[.19, .32])

base_quat = [-1/np.sqrt(2), 1/np.sqrt(2), 0 ,0 ]
rel_quat = rowan.from_axis_angle([0,1,0], np.random.uniform(0, 2*np.pi))

C.addFrame("blob") \
    .setPosition(midpoint) \
    .setShape(ry.ST.capsule, size=[.08, .07]) \
    .setColor([106/255, 24/255, 79/255]) \
    .setQuaternion(rowan.multiply(base_quat, rel_quat)) \
    .setContact(1)