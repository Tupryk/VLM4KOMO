import re
import rowan
import numpy as np
import robotic as ry


def sample_rectangular_arena(width: float=0.4,
                            height: float=0.4,
                            z_coord: float=0.69,
                            center_point: np.ndarray=[0, 0]) -> np.ndarray:
    x = center_point[0] + np.random.uniform(-width * .5, width * .5)
    y = center_point[1] + np.random.uniform(-height * .5, height * .5)
    return [x, y, z_coord]


def cleanup_highlvl_func(original_func: str, compute_collisions: bool=True, visuals: bool=False) -> str:

    high_funcs = ["pick", "place", "push", "getObj"]

    lines = original_func.split('\n')
    
    new_line = f"    env = RobotEnviroment(C, visuals={visuals}, verbose=0, compute_collisions={compute_collisions})"
    lines.insert(1, new_line)

    execute_command = lines[0].replace("def ", "").replace(":", "")
    lines.append(execute_command)

    new_func = '\n'.join(lines)

    for f in high_funcs:
        new_func = new_func.replace(f"{f}(", f"env.{f}(")
    
    return new_func

def generate_blocks_scene() -> ry.Config:

    valid = False
    colors = ["red", "green", "blue"]

    while not valid:
    
        C = ry.Config()
        C.addFile(ry.raiPath("./scenarios/push_blob.g"))

        for i in range(3):

            midpoint = sample_rectangular_arena(width=.68, height=.6, center_point=[.19, .32])
            color = [0., 0., 0.]
            color[i] = 1.

            rand_z = np.random.uniform(0., np.pi*2.)
            rot = rowan.from_euler(rand_z, 0, 0)

            C.addFrame(f"block_{colors[i]}") \
                .setShape(ry.ST.ssBox, size=[.06, .06, .06, .001]) \
                .setPosition(midpoint) \
                .setQuaternion(rot) \
                .setColor(color) \
                .setContact(1)

        col = C.eval(ry.FS.accumulatedCollisions, [])[0][0]
        valid = col <= 0
    
    return C

def generate_demo_scene() -> ry.Config:

    valid = False
    colors = ["red", "green"]

    while not valid:
    
        C = ry.Config()
        C.addFile(ry.raiPath("./scenarios/push_blob.g"))

        for i in range(3):

            midpoint = sample_rectangular_arena(width=.68, height=.6, center_point=[.19, .32])
            color = [0., 0., 0.]
            color[i] = 1.

            rand_z = np.random.uniform(0., np.pi*2.)
            rot = rowan.from_euler(rand_z, 0, 0)
            
            if i == 2:
                C.addFrame("goal_area") \
                    .setShape(ry.ST.ssBox, size=[.06, .06, .001, .001]) \
                    .setPosition(midpoint) \
                    .setColor([0., 0., 1]) \
                    .setContact(1)
                
            else:
                C.addFrame(f"block_{colors[i]}") \
                    .setShape(ry.ST.ssBox, size=[.06, .06, .06, .001]) \
                    .setPosition(midpoint) \
                    .setQuaternion(rot) \
                    .setColor(color) \
                    .setContact(1)
            

            

        col = C.eval(ry.FS.accumulatedCollisions, [])[0][0]
        valid = col <= 0



    return C


def grasping_within_komo_definition(komo_definition: str) -> list[float]:

    komo_lines = komo_definition.splitlines()
    second_line = komo_lines[1]
    slices = int(re.findall(r'-?\b\d+\.\d+|-?\b\d+\b', second_line)[1])

    lines_with_prefix = [line for line in komo_lines if line.startswith("komo.addModeSwitch")]
    
    indices = []
    for l in lines_with_prefix:
        first_number = re.findall(r'-?\b\d+\.\d+|-?\b\d+\b', l)[0]
        indices.append(int(float(first_number) * slices))

    return indices


if __name__ == "__main__":

    komo_definition = """hello
komo.setTiming(2.5, 1, 5., 2)
#grasp
komo.addModeSwitch([1, 2.], ry.SY.stable, ["gripper", "box"])
komo.addObjective([1.], ry.FS.positionDiff, ["gripper", "box"], ry.OT.eq, [1e2])
komo.addObjective([1.], ry.FS.scalarProductXX, ["gripper", "box"], ry.OT.eq, [1e2], [0.])
komo.addObjective([1.], ry.FS.vectorZ, ["gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

#slow - down - up
komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [], [], 1)
komo.addObjective([.9,1.1], ry.FS.position, ["gripper"], ry.OT.eq, [], [0.,0.,.1], 2)

#place
komo.addModeSwitch([2., -1.], ry.SY.stable, ["table", "box"])
komo.addObjective([2.], ry.FS.positionDiff, ["box", "table"], ry.OT.eq, [1e2], [0,0,.08])
komo.addObjective([2.], ry.FS.vectorZ, ["gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

#slow - down - up
komo.addObjective([2.], ry.FS.qItself, [], ry.OT.eq, [], [], 1)
komo.addObjective([1.9,2.2], ry.FS.position, ["gripper"], ry.OT.eq, [], [0.,0.,.1], 2)
"""

    indices = grasping_within_komo_definition(komo_definition)
    print(indices)
