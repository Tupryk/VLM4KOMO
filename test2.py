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
    C.addFrame(f"block_{i}") \
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
    # Get block objects
    red_block = env.getObj("block_0")
    green_block = env.getObj("block_1")
    blue_block = env.getObj("block_2")
    
    # Compute placement positions based on object sizes
    center_x, center_y = red_block.pos.x, red_block.pos.y
    
    # Placeholder optimization values
    horizontal_offset = 0.014996240515723465
    vertical_offset = 0.02311625919409065
    
    # Position for the bottom horizontal block
    bottom_x, bottom_y = center_x, center_y
    
    # Position for the vertical block
    vertical_x, vertical_y = center_x, center_y
    vertical_z = red_block.size.z + vertical_offset  # Adjust placement height
    
    # Position for the top horizontal block
    top_x, top_y = center_x, center_y
    top_z = vertical_z + green_block.size.z + horizontal_offset  # Adjust top block height
    
    # Pick and place the bottom horizontal block
    env.pick("block_0")
    env.place(bottom_x, bottom_y, rotated=True)
    
    # Pick and place the vertical block
    env.pick("block_1")
    env.place(vertical_x, vertical_y, z=vertical_z, rotated=False)
    
    # Pick and place the top horizontal block
    env.pick("block_2")
    env.place(top_x, top_y, z=top_z, rotated=True)


build_bridge()

C_copy.view(True)
# print(C_copy.eval(ry.FS.position, ["l_gripper"])[0][2])
# print(C_copy.eval(ry.FS.position, ["table"])[0][2])
# print(C_copy.eval(ry.FS.positionDiff, ["l_gripper", "table"])[0][2])
# print(C_copy.eval(ry.FS.position, ["block_blue"])[0][2])
# print(C_copy.eval(ry.FS.position, ["block_red"])[0][2])