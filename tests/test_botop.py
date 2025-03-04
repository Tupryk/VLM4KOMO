import robotic as ry
import numpy as np

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
    

path = np.load("path0.npy")

bot = ry.BotOp(C, False)
bot.sync(C, .1)
bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("path1.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)



path = np.load("path2.npy")

bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("path3.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)



path = np.load("path4.npy")


bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("path5.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)
