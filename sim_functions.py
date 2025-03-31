import numpy as np
import robotic as ry
from simulator import Simulator
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
                 visuals = False,
                 verbose: int=0,
                 compute_collisions=False
                 ):
        self.C = C
        self.verbose = verbose

    def place(self, block: str,  x: float, y: float, z: float=.0, yaw: float=None) -> bool:
        self.C.getFrame(block) \
            .setPosition([x, y, self.C.getFrame(block).getPosition()[2]]) \
            .setQuaternion([np.cos(yaw/2), 0, 0, np.sin(yaw/2)])

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
