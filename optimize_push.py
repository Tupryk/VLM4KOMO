import ast
import numpy as np
import robotic as ry
import cma
from blackBoxKomoProblem import BlackBoxKomoProblem
import rowan

def str_to_np_array(text: str) -> np.ndarray:
    return np.array(ast.literal_eval(text), dtype=np.float32)



if __name__ == "__main__":
    C = ry.Config()
    C.addFile(ry.raiPath("./scenarios/pandaSingle.g"))

    midpoint = np.array([-0.105, 0.2, 0.745])

    C.addFrame("target") \
        .setPosition(midpoint+np.array([-.22,.2,0])) \
        .setShape(ry.ST.box, size=[0.21, .36, .15]) \
        .setColor([28/255, 18/255, 210/255, .3]) \


    C.addFrame("box") \
        .setPosition([.19, .32, .745]) \
        .setShape(ry.ST.box, size=[.1, .1, .1]) \
        .setColor([106/255, 24/255, 79/255]) \
        .setContact(1) \
        .setMass(.1) 

    komo_text = """
komo = ry.KOMO(C, 2, 64, 2, True)

komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)

komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', "box"], ry.OT.eq, [1e1], [.12, 0, 0])

komo.addObjective([1,2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], [0,0,1])
komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', "target"], ry.OT.eq, [1e1])
"""

    bbk = BlackBoxKomoProblem(C, komo_text, targets=True, verbose=3)
    

    action, observation = bbk.reset()
    print(action)

    options = {
    'popsize': 7,
    'maxiter': 50,
    'maxfevals': 5000,
    }

    result = cma.fmin(bbk.step, action, sigma0=.1, options=options)
    