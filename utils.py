import re


def grasping_within_komo_definition(komo_definition: str) -> list[float]:

    komo_lines = komo_definition.splitlines()
    komo_creation = komo_lines[0]
    slices = int(re.findall(r'-?\b\d+\.\d+|-?\b\d+\b', komo_creation)[1])

    lines_with_prefix = [line for line in komo_lines if line.startswith("komo.addModeSwitch")]
    
    indices = []
    for l in lines_with_prefix:
        first_number = re.findall(r'-?\b\d+\.\d+|-?\b\d+\b', l)[0]
        indices.append(int(float(first_number) * slices))

    return indices


def clean_komo(komo_definition: str) -> str:
    komo_definition = [line for line in komo_definition.splitlines() if line.strip() not in ("```python", "```")]
    komo_definition = "\n".join(komo_definition)
    komo_definition_clean = komo_definition.splitlines()
    komo_definition_clean = [line for line in komo_definition_clean if not line.startswith("C ") and not line.startswith("C.")]
    komo_definition_clean = "\n".join(komo_definition_clean)
    return komo_definition_clean


if __name__ == "__main__":

    komo_definition = """komo = ry.KOMO(C, 2, 10, 2, True)

# Grasp the blob
komo.addModeSwitch([1.0, 2.0], ry.SY.stable, ["l_gripper", "blob"], False)
komo.addObjective([1.0], ry.FS.positionDiff, ["l_gripper", "blob"], ry.OT.eq, [1e2])
komo.addObjective([1.0], ry.FS.scalarProductXX, ["l_gripper", "blob"], ry.OT.eq, [1e2], [0.])
komo.addObjective([1.0], ry.FS.scalarProductXZ, ["l_gripper", "blob"], ry.OT.eq, [1e2], [0.])

# Move to the bin
komo.addObjective([1.5], ry.FS.positionDiff, ["blob", "bin"], ry.OT.eq, [1e1])

# Place the blob in the bin
komo.addModeSwitch([2.0, -1.0], ry.SY.stable, ["bin", "blob"], False)
komo.addObjective([2.0], ry.FS.positionDiff, ["blob", "bin"], ry.OT.eq, [1e2], [0, 0, 0.08])
komo.addObjective([2.0], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

# Add smooth motion objectives
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 2, 1e0)

# Ensure zero velocity at the end
komo.addObjective([2.0], ry.FS.jointState, [], ry.OT.eq, [1e1], [], order=1)

"""

    indices = grasping_within_komo_definition(komo_definition)
    print(indices)
