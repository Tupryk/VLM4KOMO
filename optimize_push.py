import ast
import numpy as np
import robotic as ry
import cma
from blackBoxKomoProblemNew import BlackBoxKomoProblem
import rowan

def str_to_np_array(text: str) -> np.ndarray:
    return np.array(ast.literal_eval(text), dtype=np.float32)



if __name__ == "__main__":
    komo_text = """
komo = ry.KOMO(C, 2, 64, 2, True)

komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)

komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', "blob"], ry.OT.eq, [1e1], [.1, .1, .1])

komo.addObjective([1,2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], [0,0,1])
komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', "target"], ry.OT.eq, [1e1],  [.2, .2, .2])
"""
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

    bbk = BlackBoxKomoProblem(C, komo_text, targets=True, verbose=3)
    

    action, observation = bbk.reset()
    print(action)

    options = {
    'popsize': 7,
    'maxiter': 50,
    'maxfevals': 5000,
    }

    result = cma.fmin(bbk.step, action, sigma0=.1, options=options)
    