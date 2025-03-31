import numpy as np
import robotic as ry
from simulator import Simulator
from utils import generate_blocks_scene, rotate_quat_pi_yaw
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
                 compute_collisions: bool=True,
                 on_real: bool=False,
                 use_botop: bool=False,
                 use_sim: bool=True,
                 sim_rt: bool=False
                 ):
        self.C = C
        if on_real:
            self.bot = ry.BotOp(self.C, on_real)
            self.bot.home(self.C)
        self.visuals = visuals
        self.verbose = verbose
        self.grabbed_frame = ""
        self.grasp_direction = ""
        self.path = np.array([])
        self.compute_collisions = compute_collisions
        self.use_botop = use_botop
        if use_botop:
            self.bot = ry.BotOp(C, False)
        self.use_sim = use_sim
        self.feasible = True
        self.sim_rt = sim_rt
        self.i = 0

    def pick(self, frame: str) -> bool:

        if not self.feasible:
            return self.feasible
        
        assert self.grabbed_frame == ""

        graspDirections = ['x', 'y']
        gripper = "l_gripper"
        palm = "l_palm"

        for gd in graspDirections:

            M = manip.ManipulationModelling()
            M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions)
            M.grasp_box(1., gripper, frame, palm, gd)
            M.solve(verbose=self.verbose)
            if not M.feasible:
                continue

            M2 = M.sub_motion(0, accumulated_collisions=self.compute_collisions)
            M2.no_collisions([.3,.7], [palm, frame], margin=.05)
            M2.retract([.0, .2], gripper)
            M2.approach([.8, 1.], gripper)
            self.path = M2.solve(verbose=self.verbose)
            if not M2.feasible:
                continue
            np.save(f"paths/path_{frame}_pick.npy", self.path)
            self.frame = frame

            self.i += 1
            if self.visuals:
                M2.play(self.C)
                self.C.attach(gripper, frame)
            
            elif self.use_botop:

                self.bot.sync(self.C, .1)
                self.bot.move(self.path, [3.])
                while self.bot.getTimeToEnd() > 0:
                    self.bot.sync(self.C, .1)
                self.bot.gripperClose(ry._left)
                while not self.bot.gripperDone(ry._left):
                    self.bot.sync(self.C)
                self.C.attach(gripper, frame)

            elif self.use_sim:
                C2 = ry.Config()
                C2.addConfigurationCopy(self.C)
                sim = Simulator(C2)
                xs, qs, xdots, qdots = sim.run_trajectory(self.path, 2, real_time=self.sim_rt)

                sim._sim.closeGripper("l_gripper")
                self.C.setJointState(qs[-1])
                
                self.C.attach(gripper, frame)



            else:
                qt = self.path[-1]
                self.C.setJointState(qt)
                self.C.attach(gripper, frame)

            self.grabbed_frame = frame
            self.grasp_direction = gd
            return True


        print("PICK INFEASIBLE")
        self.feasible = False
        return False

    def place(self, x: float, y: float, z: float=.0, rotated: bool=False, yaw: float=None) -> bool:
        if not self.feasible:
            return False
        
        assert self.grabbed_frame != ""

        table = "table"
        palm = "l_palm"
        table_frame = self.C.getFrame("table")
        
        if rotated and self.grasp_direction == 'x':
            place_direction = ['y', 'yNeg']
        elif rotated and self.grasp_direction == 'y':
            place_direction = ['x', 'xNeg']
        elif not rotated:
            place_direction = ['z', 'zNeg']

        feasible = False
        Ms = []
        for i, direction in enumerate(place_direction):
            M = manip.ManipulationModelling()
            M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions, homing_scale=.1)
            
            if not z:
                M.place_box(1., self.grabbed_frame, table, palm, direction)
                M.target_relative_xy_position(1., self.grabbed_frame, table, [x, y])
            else:
                table_offset = table_frame.getPosition()[2] + table_frame.getSize()[2]*.5
                if z < table_offset:
                    z += table_offset
                M.place_box(1., self.grabbed_frame, table, palm, direction, on_table=False)
                M.target_position(1., self.grabbed_frame, [x, y, z])

            if yaw != None:

                if direction == "x" or direction == "xNeg":
                    feature = ry.FS.scalarProductXZ
                elif direction == "y" or direction == "yNeg":
                    feature = ry.FS.scalarProductXX
                elif direction == "z" or direction == "zNeg":
                    feature = ry.FS.scalarProductXX
                else:
                    raise Exception(f"'{place_direction}' is not a valid up vector for a place motion!")
                
                M.komo.addObjective([.8, 1.], feature, [table, self.grabbed_frame], ry.OT.eq, [1e1], yaw)

            M.solve(verbose=self.verbose)
            Ms.append((M, M.ret.sos + M.ret.eq))
            if M.feasible:    
                feasible = True

        Ms.sort(key=lambda x: x[1])  # Sort by cost (index 1)
        if not feasible:    
            print("PLACE INFEASIBLE 1")

            self.feasible = False
            return False

        M = Ms[0][0]

        M3 = M.sub_motion(0, accumulated_collisions=self.compute_collisions)
        self.path = M3.solve(verbose=self.verbose)
        if not M3.ret.feasible:
            print("PLACE INFEASIBLE 1")

            self.feasible = False
            return False

        np.save(f"paths/path_{self.frame}_place.npy", self.path)
        self.i += 1
        if self.visuals:
            M3.play(self.C)
            self.C.attach(table, self.grabbed_frame)
        
        elif self.use_botop:
            self.bot.move(self.path, [3.])
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C)
            self.bot.gripperMove(ry._left)
            while not self.bot.gripperDone(ry._left):
                self.bot.sync(self.C)
            self.C.attach(table, self.grabbed_frame)

        elif self.use_sim:
            self.C.attach(table, self.grabbed_frame)

            C2 = ry.Config()
            C2.addConfigurationCopy(self.C)
            sim = Simulator(C2)
            xs, qs, xdots, qdots = sim.run_trajectory(self.path, 2, real_time=self.sim_rt, close_gripper=True)
            
            self.C.setJointState(qs[-1])
            self.C.setFrameState(xs[-1])
        
        else:
            qt = self.path[-1]
            self.C.setJointState(qt)
            self.C.attach(table, self.grabbed_frame)

        self.grabbed_frame = ""
        self.grasp_direction = ""
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
        M.solve(verbose=self.verbose)
        if not M.ret.feasible:
            self.feasible = False
            return False

        M1 = M.sub_motion(0, accumulated_collisions=False)
        M1.retractPush([.0, .15], gripper, .03)
        M1.approachPush([.85, 1.], gripper, .03)
        M1.no_collisions([.15,.85], [frame, 'l_finger1'], .02)
        M1.no_collisions([.15,.85], [frame, 'l_finger2'], .02)
        M1.no_collisions([.15,.85], [frame, 'l_palm'], .02)
        M1.no_collisions([], [table, 'l_finger1'], .0)
        M1.no_collisions([], [table, 'l_finger2'], .0)
        path1 = M1.solve(verbose=self.verbose)
        if not M1.ret.feasible:
            self.feasible = False
            return False

        M2 = M.sub_motion(1, accumulated_collisions=False)
        M2.komo.addObjective([], ry.FS.positionRel, [gripper, pushStart], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
        path2 = M2.solve(verbose=self.verbose)
        if not M2.ret.feasible:
            self.feasible = False
            return False

        if self.visuals:
            M1.play(self.C, 1.)
            self.C.attach(gripper, frame)
            M2.play(self.C, 1.)
            self.C.attach(table, frame)
        
        elif self.use_sim:
            #TODO
            pass

        else:
            self.bot.move(path1, [3.])
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C)
            self.bot.move(path2, [3.])
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C)

        self.path = np.concatenate((path1, path2))

        return True
    
    def getObj(self, object_name: str) -> RAIObj:
        obj = RAIObj(self.C, object_name)
        return obj

    def place2(self, x: float, y: float, z: float=.0, yaw: float=None) -> bool:
        if not self.feasible:
            print("NOT FEASIBLE")
            return False
        
        assert self.grabbed_frame != ""

        table = "table"
        palm = "l_palm"
        table_frame = self.C.getFrame("table")
        
        place_direction = ['z', 'zNeg']

        feasible = False
        Ms = []
        for i, direction in enumerate(place_direction):
            M = manip.ManipulationModelling()
            M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions, homing_scale=.1)
            
            if not z:
                M.place_box(1., self.grabbed_frame, table, palm, direction)
                M.target_relative_xy_position(1., self.grabbed_frame, table, [x, y])
            else:
                table_offset = table_frame.getPosition()[2] + table_frame.getSize()[2]*.5
                if z < table_offset:
                    z += table_offset
                M.place_box(1., self.grabbed_frame, table, palm, direction, on_table=False)
                M.target_position(1., self.grabbed_frame, [x, y, z])

            if yaw != None:
                #M.komo.addObjective([.8, 1.], ry.FS.quaternionDiff, [table, self.grabbed_frame], ry.OT.eq, [1e1], [np.cos(yaw)])
                M.komo.addObjective([.8, 1.], ry.FS.quaternionDiff, [self.grabbed_frame, yaw], ry.OT.eq, [1e1])

            M.solve(verbose=self.verbose)
            Ms.append((M, M.ret.sos + M.ret.eq))
            if M.feasible:    
                feasible = True

        Ms.sort(key=lambda x: x[1])  # Sort by cost (index 1)
        if not feasible:    
            print("PLACE INFEASIBLE 1")

            self.feasible = False
            return False

        M = Ms[0][0]

        M3 = M.sub_motion(0, accumulated_collisions=self.compute_collisions)
        self.path = M3.solve(verbose=self.verbose)
        if not M3.ret.feasible:
            print("PLACE INFEASIBLE 2")

            self.feasible = False
            return False

        if self.visuals:
            M3.play(self.C)
            self.C.attach(table, self.grabbed_frame)
        
        elif self.use_botop:
            self.bot.move(self.path, [3.])
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C)
            self.bot.gripperMove(ry._left)
            while not self.bot.gripperDone(ry._left):
                self.bot.sync(self.C)
            self.C.attach(table, self.grabbed_frame)

        elif self.use_sim:
            self.C.attach(table, self.grabbed_frame)

            C2 = ry.Config()
            C2.addConfigurationCopy(self.C)
            sim = Simulator(C2)
            xs, qs, xdots, qdots = sim.run_trajectory(self.path, 2, real_time=self.sim_rt, close_gripper=True)
            
            self.C.setJointState(qs[-1])
            self.C.setFrameState(xs[-1])
        
        else:
            qt = self.path[-1]
            self.C.setJointState(qt)
            self.C.attach(table, self.grabbed_frame)

        self.grabbed_frame = ""
        self.grasp_direction = ""
        return True

    def place3(self, C, frame, x_off=0, y_off=0) -> bool:
        if not self.feasible:
            print("NOT FEASIBLE")
            return False
        
        assert self.grabbed_frame != ""

        table = "table"
        palm = "l_palm"
        table_frame = self.C.getFrame("table")
        
        place_direction = ['z', 'zNeg']

        Ms = []
        self.C.addFrame("frameTo").setQuaternion(C.getFrame(frame).getQuaternion()).setPosition(C.getFrame(frame).getPosition())

        self.C.addFrame("frame_rotated").setQuaternion(rotate_quat_pi_yaw(self.C.getFrame("frameTo").getQuaternion()))

        feasible = False
        for i, direction in enumerate(place_direction):
            for j in range(2):
                M = manip.ManipulationModelling()
                M.setup_sequence(self.C, 1, accumulated_collisions=self.compute_collisions, homing_scale=.1)
                
                M.place_box(1., self.grabbed_frame, table, palm, direction, on_table=False)
                M.target_position(1., self.grabbed_frame, self.C.getFrame("frameTo").getPosition()+np.asarray([x_off, y_off, .02]))
                if j == 0:
                    M.komo.addObjective([1.], ry.FS.quaternionDiff, [self.grabbed_frame, "frameTo"], ry.OT.eq, [1e1])
                else:
                    M.komo.addObjective([1.], ry.FS.quaternionDiff, [self.grabbed_frame, "frame_rotated"], ry.OT.eq, [1e1])


                M.solve(verbose=self.verbose)
                Ms.append((M, M.feasible, M.ret.eq))

                print(f"Dir {i} Quat {j} Feasible: {M.feasible}")

        Ms.sort(key=lambda x: x[2])  # Sort by cost (index 1)

        for score in Ms:
            if score[1]:
                self.feasible = True
                break
            self.feasible = False
            return False
        
            
        M3 = Ms[0][0].sub_motion(0, accumulated_collisions=self.compute_collisions)
        self.path = M3.solve(verbose=self.verbose)
        if not M3.ret.feasible:
            print("PLACE INFEASIBLE 2")

            self.feasible = False
            return False
        
        np.save(f"paths/path_{frame}_place.npy", self.path)
        self.i += 1

        if self.visuals:
            M3.play(self.C)
            self.C.attach(table, self.grabbed_frame)
        
        elif self.use_botop:
            self.bot.move(self.path, [3.])
            while self.bot.getTimeToEnd() > 0:
                self.bot.sync(self.C)
            self.bot.gripperMove(ry._left)
            while not self.bot.gripperDone(ry._left):
                self.bot.sync(self.C)
            self.C.attach(table, self.grabbed_frame)

        elif self.use_sim:
            self.C.attach(table, self.grabbed_frame)

            C2 = ry.Config()
            C2.addConfigurationCopy(self.C)
            sim = Simulator(C2)
            xs, qs, xdots, qdots = sim.run_trajectory(self.path, 2, real_time=self.sim_rt, close_gripper=True)
            
            self.C.setJointState(qs[-1])
            self.C.setFrameState(xs[-1])
        
        else:
            qt = self.path[-1]
            self.C.setJointState(qt)
            self.C.attach(table, self.grabbed_frame)

        self.grabbed_frame = ""
        self.grasp_direction = ""
        return True


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
