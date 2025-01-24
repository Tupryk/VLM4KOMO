import numpy as np
import robotic as ry

from utils import generate_blocks_scene
import Robotic_Manipulation.manipulation as manip


class RAIVec:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class RAIObj:
    def __init__(self, C: ry.Config, name: str):
        self.C = C
        self.name = name

    @property
    def pos(self) -> RAIVec:
        nums = self.C.getFrame(self.name).getPosition()
        vec = RAIVec(*nums)
        return vec
    
    @property
    def size(self) -> RAIVec:
        real_size = self.C.getFrame(self.name).getSize()[:3]

        rel_size = RAIVec(0, 0, 0)
        # Height
        rel_size.z = np.argmax([
            np.abs(self.C.eval(ry.FS.scalarProductXZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductYZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductZZ, [self.name, "table"])[0][0])
            ])
        # Height
        rel_size.z = np.argmax([
            np.abs(self.C.eval(ry.FS.scalarProductXZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductYZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductZZ, [self.name, "table"])[0][0])
            ])
        # Height
        rel_size.z = np.argmax([
            np.abs(self.C.eval(ry.FS.scalarProductXZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductYZ, [self.name, "table"])[0][0]),
            np.abs(self.C.eval(ry.FS.scalarProductZZ, [self.name, "table"])[0][0])
            ])
        return RAIVec(*real_size)


class RobotEnviroment:
    def __init__(self,
                 C: ry.Config,
                 visuals: bool=False,
                 verbose: int=0,
                 compute_collisions: bool=True):
        self.C = C
        self.visuals = visuals
        self.verbose = verbose
        self.grabbed_frame = ""
        self.path = np.array([])
        self.compute_collisions = compute_collisions

    def pick(self, frame: str) -> bool:
        assert self.grabbed_frame == ""

        graspDirection = np.random.choice(['x', 'y'])
        gripper = "l_gripper"
        table = "table"
        palm = "l_palm"

        M = manip.ManipulationModelling()
        M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions)
        M.grasp_box(1., gripper, frame, palm, graspDirection)
        M.no_collisions([], [palm, table])
        M.solve()
        if not M.feasible:
            return False

        M2 = M.sub_motion(0, accumulated_collisions=self.compute_collisions)
        M2.no_collisions([.3,.7], [palm, frame], margin=.05)
        M2.retract([.0, .2], gripper)
        M2.approach([.8, 1.], gripper)
        self.path = M2.solve()
        if not M2.feasible:
            return False

        if self.visuals:
            M2.play(self.C)
            self.C.attach(gripper, frame)
        self.grabbed_frame = frame
        return True

    def place(self, x: float, y: float, z: float=.0, up_vec: str="z", yaw: float=None) -> bool:
        assert self.grabbed_frame != ""

        table = "table"
        palm = "l_palm"
        table_frame = self.C.getFrame("table")

        M = manip.ManipulationModelling()
        M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions)
        
        if not z:
            M.place_box(1., self.grabbed_frame, table, palm, up_vec)
            M.no_collisions([], [palm, table])
            M.target_relative_xy_position(1., self.grabbed_frame, table, [x, y])
        else:
            z += table_frame.getPosition()[2] + table_frame.getSize()[2]*.5
            M.place_box(1., self.grabbed_frame, table, palm, up_vec, on_table=False)
            M.target_position(1., self.grabbed_frame, [x, y, z])

        if yaw != None:
            yaw = np.deg2rad(yaw)
            scalar_prod = np.cos(yaw)
            if up_vec == "x":
                feature = ry.FS.scalarProductXZ
            elif up_vec == "y":
                feature = ry.FS.scalarProductXZ
            elif up_vec == "z":
                feature = ry.FS.scalarProductXX
            else:
                raise Exception(f"'{up_vec}' is not a valid up vector for a place motion!")
            
            M.komo.addObjective([1.], feature, [table, self.grabbed_frame], ry.OT.eq, [1e1], scalar_prod)

        M.solve()
        if not M.feasible:
            return False

        M3 = M.sub_motion(0, accumulated_collisions=self.compute_collisions)
        M3.no_collisions([], [table, self.grabbed_frame])
        self.path = M3.solve()
        if not M3.ret.feasible:
            return False

        if self.visuals:
            M3.play(self.C)
            self.C.attach(table, self.grabbed_frame)
        self.grabbed_frame = ""
        return True

    def push(self, frame: str, relative_x: float, relative_y: float) -> bool:

        obj_pos = self.C.getFrame(frame).getPosition()
        target_pos = obj_pos + np.array([relative_x, relative_y, 0.])
        
        gripper = "l_gripper"
        table = "table"
        
        M = manip.ManipulationModelling()
        M.setup_pick_and_place_waypoints(self.C, gripper, frame, 1e-1, accumulated_collisions=False)
        pushStart = M.straight_push([1.,2.], frame, gripper, table)
        M.target_xy_position(2., frame, target_pos)
        M.solve()
        if not M.ret.feasible:
            return False

        M1 = M.sub_motion(0, accumulated_collisions=False)
        M1.retractPush([.0, .15], gripper, .03)
        M1.approachPush([.85, 1.], gripper, .03)
        M1.no_collisions([.15,.85], [frame, 'l_finger1'], .02)
        M1.no_collisions([.15,.85], [frame, 'l_finger2'], .02)
        M1.no_collisions([.15,.85], [frame, 'l_palm'], .02)
        M1.no_collisions([], [table, 'l_finger1'], .0)
        M1.no_collisions([], [table, 'l_finger2'], .0)
        path1 = M1.solve()
        if not M1.ret.feasible:
            return False

        M2 = M.sub_motion(1, accumulated_collisions=False)
        M2.komo.addObjective([], ry.FS.positionRel, [gripper, pushStart], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
        path2 = M2.solve()
        if not M2.ret.feasible:
            return False

        if self.visuals:
            M1.play(self.C, 1.)
            self.C.attach(gripper, frame)
            M2.play(self.C, 1.)
            self.C.attach(table, frame)

        self.path = np.concatenate((path1, path2))

        return True
    
    def getObj(self, object_name: str) -> RAIObj:
        obj = RAIObj(self.C, object_name)
        return obj


if __name__ == "__main__":

    ### Create Enviroment ###
    C = generate_blocks_scene()

    env = RobotEnviroment(C, visuals=True, verbose=1, compute_collisions=False)


    ### Test High Level Functions ###
    # env.pick("block_red")

    # x = np.random.uniform(-.1, .1) - .105
    # y = np.random.uniform(-.1, .1) + .4
    # env.place(x, y)

    relative_x = np.random.uniform(-.05, .05) - .105
    relative_y = np.random.uniform(-.05, .05) + .4
    env.push("block_red", relative_x, relative_y)
