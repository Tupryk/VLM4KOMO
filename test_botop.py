import robotic as ry
import numpy as np

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")

names = ["red", "green", "blue"]

# Objects
# for i in range(3):
#     color = [0., 0., 0.]
#     color[i%3] = 1.
#     C.addFrame(f"block_{names[i]}") \
#         .setPosition([(i%3)*.15, (i//3)*.1+.1, .71]) \
#         .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
#         .setColor(color) \
#         .setContact(1) \
#         .setMass(.1)
    
for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1.
    C.addFrame(f"block_{i}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .68]) \
        .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)

path = np.load("paths/path_block_0_pick.npy")

bot = ry.BotOp(C, False)
bot.sync(C, .1)
bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("paths/path_block_0_place.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)



path = np.load("paths/path_block_1_pick.npy")

bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("paths/path_block_1_place.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)



path = np.load("paths/path_block_2_pick.npy")


bot.move(path, [3.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

path = np.load("paths/path_block_2_place.npy")
bot.move(path, [3.])

while bot.getTimeToEnd() > 0:
    bot.sync(C)

bot.gripperMove(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

C.view(True)