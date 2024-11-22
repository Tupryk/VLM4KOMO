import robotic as ry
import numpy as np
import time
import utils
import cma

C = ry.Config()

C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
C.addFrame('refTarget'). setShape(ry.ST.marker, [.4]) .setPosition([.4, .4, .9])

sampled_vector = np.random.uniform([-.3, 1, .7], [.3, 2, 1.5])

range_x = [-.2, .6]
range_y = [-.1, .8]
range_z = [.7, 1.5]

utils.draw_uniform_block(C, range_x, range_y, range_z)


def l2_dist_to_target(target, komo):
    return np.linalg.norm(np.asarray(target)-np.asarray(komo), 2)

def runKomo(target):

    komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
    komo.addControlObjective([], 0, 1e-1)
    komo.addControlObjective([], 1, 1e0)
    komo.addObjective([1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], target=target)

    ret = ry.NLP_Solver(komo.nlp(), verbose=2) .solve()
    q = komo.getPath()

    C.setJointState([q[-1]])
    time.sleep(.02)
    C.view(False)

    loss = l2_dist_to_target(C.getFrame("refTarget").getPosition(), C.getFrame("l_gripper").getPosition())
    print(loss)
    
    return loss

x = np.random.uniform(*range_x)
y = np.random.uniform(*range_y)
z = np.random.uniform(*range_z)

pos = np.array([x, y, z])
options = {
    'popsize': 7,
    'maxiter': 50,
    'maxfevals': 5000,
    'tolfun': 1e-4,
    'tolx': 1e-5
}

result = cma.fmin(runKomo, pos, .1, options=options)
print (result)