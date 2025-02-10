# joint_dim = len(C.getJointState())
joint_dim = 10

# Feature name, frame count, dimensionality
features = {
    "position": [1, 3],
    "positionDiff": [2, 3],
    "positionRel": [2, 3],
    "quaternion": [1, 4],
    "quaternionDiff": [2, 4],
    "quaternionRel": [2, 4],
    "pose": [1, 7],
    "poseDiff": [2, 7],
    "poseRel": [2, 7],
    "vectorX": [1, 3],
    "vectorXDiff": [2, 3],
    "vectorXRel": [2, 3],
    "vectorY": [1, 3],
    "vectorYDiff": [2, 3],
    "vectorYRel": [2, 3],
    "vectorZ": [1, 3],
    "vectorZDiff": [2, 3],
    "vectorZRel": [2, 3],
    "scalarProductXX": [2, 1],
    "scalarProductXY": [2, 1],
    "scalarProductXZ": [2, 1],
    "scalarProductYY": [2, 1],
    "scalarProductYZ": [2, 1],
    "scalarProductZZ": [2, 1],
    "angularVel": [1, 3],
    "negDistance": [2, 1],
    "qItself": [0, joint_dim],
    "aboveBox": [2, 4],
    "insideBox": [2, 6]
}

contraint_types = [
    "ineq",
    "eq",
    "sos"
]

frame_names = [
    "l_gripper",
    "box1",
    "box2",
    "handle"
]
