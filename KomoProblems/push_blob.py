import robotic as ry
import numpy as np

C = ry.Config()
C.addFile("scenarios/push_blob.g")
C.view(True)

startPoint=C.getFrame("blob").getPosition()
endPoint=C.getFrame("target").getPosition() 

directionVector = endPoint-startPoint/np.linalg.norm(endPoint-startPoint)   # define and normalize direction vector for initial position of gripper

# define inital point where to start the push, which is infront of the object to be pushed
C.addFrame("initialPushPoint").setShape(ry.ST.marker, size=[.1]).setPosition(startPoint-.1*directionVector)

komo = ry.KOMO()
komo.setConfig(C, True)

komo.setTiming(3, 20, 1., 2)


komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)


komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

komo.addObjective([1], ry.FS.positionDiff, ['gripper', "initialPushPoint"], ry.OT.eq, [1e1])
komo.addObjective([2], ry.FS.positionDiff, ['gripper', "initialPushPoint"], ry.OT.eq, [1e1])
komo.addObjective([3], ry.FS.positionDiff, ['gripper', "target"], ry.OT.eq, [1e1])

# komo.addObjective([3], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1)   # no motion in the last timestep

ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
bot = ry.BotOp(C, False)

path = komo.getPath()

print(path)
bot.move(path,[5.])

while (bot.getTimeToEnd()>0):
    bot.sync(C)