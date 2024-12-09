import numpy as np
import robotic as ry
from time import sleep


class RobotEnviroment:
    def __init__(self, C: ry.Config, visuals: bool=False, verbose: int=0):
        self.C = C
        self.visuals = visuals
        self.verbose = verbose
        self.grabbed_frame = ""


    def basicKomo(self, C: ry.Config, phases: int=1) -> ry.KOMO:
        komo = ry.KOMO(C, phases, 32, 2, True)
        
        komo.addControlObjective([], 0, 1e-2)
        komo.addControlObjective([], 1, 1e-1)
        komo.addControlObjective([], 2, 1e-2)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1e1])

        return komo

    def solve_and_run(self, komo: ry.KOMO, C: ry.Config, visuals: bool=False) -> bool:
        sol = ry.NLP_Solver()
        sol.setProblem(komo.nlp())
        sol.setOptions(damping=1e-1, verbose=0, stopTolerance=1e-3, maxLambda=100., stopInners=20, stopEvals=200)
        ret = sol.solve()

        if not ret.feasible:
            return False
        
        path = komo.getPath()
        if visuals:
            for q in path:
                C.setJointState(q)
                C.view()
                sleep(.1)
        else:
            C.setJointState(path[-1])
                
        return True

    def pick(self, frame: str) -> bool:

        komo = self.basicKomo(self.C)
        komo.addObjective([1.], ry.FS.negDistance, ["l_gripper", frame], ry.OT.ineq, [-1e1], [.01])
        komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])

        solved = self.solve_and_run(komo, self.C, self.visuals)
        if not solved:
            if self.verbose:
                print("Pick was unsuccessful!")
            return False
        
        self.C.attach("l_gripper", frame)
        self.grabbed_frame = frame
        return True

    def place(self, x: float, y: float, z: float=.69) -> bool:
         
        komo = self.basicKomo(self.C)
        komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e1], [x, y, z])
        komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])

        solved = self.solve_and_run(komo, self.C, self.visuals)
        if not solved:
            if self.verbose:
                print("Place was unsuccessful!")
            return False
        
        if self.grabbed_frame:
            self.grabbed_frame = ""
            self.C.attach("table", self.grabbed_frame)
        return True

    def push(self, frame: str, relative_x: float, relative_y: float) -> bool:
        pass


if __name__ == "__main__":

    ### Create Enviroment ###
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
    C.addFrame("box") \
        .setPosition([.3, 0.05, 0.72]) \
        .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.04, 0.001]) \
        .setColor([1., 0., 0.]) \
        .setContact(1) \
        .setMass(.1)
    C.view()

    env = RobotEnviroment(C, visuals=True, verbose=1)


    ### Test High Level Functions ###
    env.pick("box")

    x = np.random.uniform(-.1, .1) - .105
    y = np.random.uniform(-.1, .1) + .4
    env.place(x, y)

    relative_x = np.random.uniform(-.1, .1) - .105
    relative_y = np.random.uniform(-.1, .1) + .4
    env.push("box", relative_x, relative_y)
