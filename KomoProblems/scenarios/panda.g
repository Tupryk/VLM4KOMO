# Create panda and its base
base: {
 X: [0, 0, 0.],
 shape: ssBox, size: [1.1, 1.2, .02, .005], color: [.8],
 contact: 1, logical: { },
 friction: .5
}
#Include </home/denis/miniconda3/envs/myenv/lib/python3.8/site-packages/robotic/rai-robotModels/scenarios/panda_fixGripper.g>
Include </home/eckart/miniconda3/envs/project/lib/python3.10/site-packages/robotic/rai-robotModels/scenarios/panda_fixGripper.g>

Edit panda_base (base): { Q: "t(0 .33 .01) d(-90 0 0 1)" joint:rigid }

Edit panda_joint2: { q: -.5 }
Edit panda_joint4: { q: -2 }
Edit panda_joint7: { q: -.5 }
Edit panda_finger_joint1: { q: 0. }
