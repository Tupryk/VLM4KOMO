import robotic as ry
import rowan
import numpy as np 
import time

C = ry.Config()
C.addFile(ry.raiPath("./scenarios/push_blob.g"))

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
    .setQuaternion(rowan.multiply(base_quat, rel_quat))
C.view(True)




directionVector = C.getFrame("target").getPosition()-C.getFrame("blob").getPosition()/np.linalg.norm(C.getFrame("target").getPosition()-C.getFrame("blob").getPosition())


komo = ry.KOMO()
komo.setConfig(C, True)

komo.setTiming(2, 64, 1., 2)

komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)

komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

# 
komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', "blob"], ry.OT.eq, [1e1])

# ensure the gripper to parallel to the z-axis
komo.addObjective([1,2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], [0,0,1])
# ensure the gripper to be at target position in the final timestep (timestep 2)
komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', "target"], ry.OT.eq, [1e1])


ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
bot = ry.BotOp(C, False)

path = komo.getPath()
duration = 5
for t in range(path.shape[0]):
    if(t==63):
        C.attach("l_gripper", "blob")
    C.setJointState(path[t])
    C.view(False)
    time.sleep(duration/path.shape[0])

    print(np.linalg.norm(C.getFrame("blob").getPosition()-C.getFrame("target").getPosition()))