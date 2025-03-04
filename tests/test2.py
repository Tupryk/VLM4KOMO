import cma
import numpy as np
import robotic as ry
from high_level_funcs_old import RobotEnviroment
import os 

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")

names = ["red", "green", "blue"]

# Objects
for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1.
    C.addFrame(f"block_{names[i]}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .71]) \
        .setShape(ry.ST.ssBox, size=[.04, .04, .12, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)
    



results = []

# # Read and execute each file, storing results along with filename
# for filename in os.listdir("outputs"):
#     if filename.endswith(".txt"):
#         file_path = os.path.join("outputs", filename)
#         with open(file_path, "r", encoding="utf-8") as file:
#             code = file.read()
#             try:
#                 C_copy = ry.Config()
#                 C_copy.addConfigurationCopy(C)
#                 exec(code)  # Execute the code from the text file
#                 result_value = C_copy.eval(ry.FS.positionDiff, ["l_gripper", "table"])[0][2]
#                 results.append((result_value, filename, C_copy))  # Store result and filename

#                 del C_copy
#             except Exception as e:
#                 print(f"Error executing {filename}: {e}")

# # Sort results by the result_value
# sorted_results = sorted(results, key=lambda x: x[0])

# # Output the sorted results
# for result_value, filename, C_copy in sorted_results:
#     print(f"{filename}: {result_value}")
#     C_copy.view(True)  # View the configuration after sorting

C_copy = ry.Config()
C_copy.addConfigurationCopy(C)
def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=True)
    # Get object parameters
    red_block = env.getObj("block_red")
    green_block = env.getObj("block_green")
    blue_block = env.getObj("block_blue")
    
    # Determine positions for vertical support blocks
    support_x_offset = blue_block.size.x / 2 + red_block.size.x / 2 + -0.10549055654747053
    support1_x = blue_block.pos.x - support_x_offset
    support2_x = blue_block.pos.x + support_x_offset
    support_y = blue_block.pos.y + 0.2527571584373817
    support_z = red_block.size.z / 2  # Place directly on the table
    
    # Place vertical support blocks
    env.pick("block_red")
    env.place(support1_x, support_y, support_z)
    
    env.pick("block_green")
    env.place(support2_x, support_y, support_z)
    
    # Place horizontal block on top
    bridge_z = support_z + red_block.size.z / 2 + blue_block.size.z / 2 + -0.0153674888522006
    env.pick("block_blue")
    env.place(blue_block.pos.x, support_y, bridge_z, rotated=True, yaw=0.1443180794199308)

build_bridge()

C_copy.view(True)
# print(C_copy.eval(ry.FS.position, ["l_gripper"])[0][2])
# print(C_copy.eval(ry.FS.position, ["table"])[0][2])
# print(C_copy.eval(ry.FS.positionDiff, ["l_gripper", "table"])[0][2])
# print(C_copy.eval(ry.FS.position, ["block_blue"])[0][2])
# print(C_copy.eval(ry.FS.position, ["block_red"])[0][2])