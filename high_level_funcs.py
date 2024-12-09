import numpy as np
import robotic as ry
from time import sleep

import Robotic_Manipulation.manipulation as manip


class RobotEnviroment:
    def __init__(self, C: ry.Config, visuals: bool=False, verbose: int=0):
        self.C = C
        self.visuals = visuals
        self.verbose = verbose
        self.grabbed_frame = ""
        self.path = np.array([])

    def pick(self, frame: str) -> bool:

        graspDirection = np.random.choice(['x', 'y'])
        gripper = "l_gripper"
        table = "table"
        palm = "l_palm"
        box = frame
        M = manip.ManipulationModelling()
        M.setup_sequence(self.C, 1)
        M.grasp_box(1., gripper, box, palm, graspDirection)
        M.no_collisions([], [palm, table])
        ways = M.solve()
        if not M.feasible:
            return False

        M2 = M.sub_motion(0)
        M2.no_collisions([.3,.7], [palm, box], margin=.05)
        M2.retract([.0, .2], gripper)
        M2.approach([.8, 1.], gripper)
        M2.solve()
        if not M2.feasible:
            return False

        M2.play(C)
        self.grabbed_frame = frame
        self.C.attach(gripper, frame)
        return True

    def place(self, x: float, y: float, z: float=.69) -> bool:
        assert self.grabbed_frame != ""
        placeDirection = np.random.choice(['x', 'y', 'z', 'xNeg', 'yNeg', 'zNeg'])
        table = "table"
        palm = "l_palm"

        box = self.grabbed_frame
        M = manip.ManipulationModelling()
        M.setup_sequence(self.C, 1)
        M.place_box(1., box, table, palm, placeDirection)
        M.no_collisions([], [palm, table])
        M.target_relative_xy_position(1., box, table, [x, y])
        ways = M.solve()
        if not M.feasible:
            return False

        M3 = M.sub_motion(0)
        M3.no_collisions([], [table, box])
        M3.solve()
        if not M3.ret.feasible:
            return False

        M3.play(C)
        C.attach(table, box)
        self.grabbed_frame = ""
        return True

    def push(self, frame: str, relative_x: float, relative_y: float) -> bool:

        obj_pos = self.C.getFrame(frame).getPosition()
        target_pos = obj_pos + np.array([relative_x, relative_y, 0.])
        
        gripper = "l_gripper"
        table = "table"
        obj = frame
        M = manip.ManipulationModelling()
        M.setup_pick_and_place_waypoints(self.C, gripper, obj, 1e-1, accumulated_collisions=False)
        pushStart = M.straight_push([1.,2.], obj, gripper, table)
        M.komo.addObjective([2.], ry.FS.position, [obj], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), target_pos)
        M.solve()
        if not M.ret.feasible:
            return False

        M1 = M.sub_motion(0, accumulated_collisions=False)
        M1.retractPush([.0, .15], gripper, .03)
        M1.approachPush([.85, 1.], gripper, .03)
        M1.no_collisions([.15,.85], [obj, 'l_finger1'], .02)
        M1.no_collisions([.15,.85], [obj, 'l_finger2'], .02)
        M1.no_collisions([.15,.85], [obj, 'l_palm'], .02)
        M1.no_collisions([], [table, 'l_finger1'], .0)
        M1.no_collisions([], [table, 'l_finger2'], .0)
        M1.solve()
        if not M1.ret.feasible:
            return False

        M2 = M.sub_motion(1, accumulated_collisions=False)
        M2.komo.addObjective([], ry.FS.positionRel, [gripper, pushStart], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
        M2.solve()
        if not M2.ret.feasible:
            return False

        M1.play(C, 1.)
        C.attach(gripper, obj)
        M2.play(C, 1.)
        C.attach(table, obj)

        return True


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
