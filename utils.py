import re


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
