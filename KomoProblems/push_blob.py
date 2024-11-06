import robotic as ry
import numpy as np

C = ry.Config()
C.addFile("scenarios/push_blob.g")
C.view(True)

startPoint=C.getFrame("blob").getPosition()
endPoint=C.getFrame("target").getPosition() 

directionVector = endPoint-startPoint/np.linalg.norm(endPoint-startPoint)   # define and normalize direction vector for initial position of gripper

# define inital point where to start the push, which is infront of the object to be pushed
C.addFrame("initialPushPoint").setShape(ry.ST.marker, size=[.1]).setPosition(startPoint-.01*directionVector)

komo = ry.KOMO()
komo.setConfig(C, True)

komo.setTiming(2, 20, 1., 2)


komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)

komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

# 
komo.addObjective([1], ry.FS.positionDiff, ['gripper', "initialPushPoint"], ry.OT.eq, [1e1])

# ensure being in a straight line in the push between timestep 1 and 2
komo.addObjective([1,2], ry.FS.positionDiff, ['gripper', "target"], ry.OT.eq, (np.eye(3)-np.outer(directionVector,directionVector)))
# ensure the gripper to parallel to the z-axis
komo.addObjective([1,2], ry.FS.vectorZ, ['gripper'], ry.OT.eq, [1], [0,0,1])
# ensure the gripper to be at target position in the final timestep (timestep 2)
komo.addObjective([2], ry.FS.positionDiff, ['gripper', "target"], ry.OT.eq, [1e1])


ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
bot = ry.BotOp(C, False)

path = komo.getPath()

print(path)
bot.moveAutoTimed(path,1.)

while (bot.getTimeToEnd()>0):
    bot.sync(C)