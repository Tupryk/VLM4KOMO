import cma
import numpy as np
import robotic as ry
from high_level_funcs_old import RobotEnviroment

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.delFrame("panda_collCameraWrist")


for i in range(3):
    color = [0., 0., 0.]
    color[i%3] = 1.
    C.addFrame(f"block_{i}") \
        .setPosition([(i%3)*.15, (i//3)*.1+.1, .68]) \
        .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)

C.view(True)
    

C_copy = ry.Config()
C_copy.addConfigurationCopy(C)
qHome = C_copy.getJointState()
C_copy.view(True)

def build_bridge():
    env = RobotEnviroment(C_copy, visuals=False, verbose=0, compute_collisions=True, use_sim=True, sim_rt=True)

    # Hole die Objekteigenschaften der Blöcke
    red_block = env.getObj("block_0")
    green_block = env.getObj("block_1")
    blue_block = env.getObj("block_2")
    
    # Bestimme die Höhe eines Blocks
    block_height = red_block.size.z
    
    # Bestimme die Platzierungsposition auf dem Tisch
    base_x, base_y, base_z = red_block.pos.x, red_block.pos.y, red_block.pos.z
    table_z = base_z - (block_height / 2)  # Der Tisch liegt unter dem Block
    
    # Stapelpositionen berechnen
    place_red_z = table_z + (block_height / 2)
    place_green_z = place_red_z + block_height
    place_blue_z = place_green_z + block_height
    
    # Stapelprozess
    env.pick("block_0")
    env.place(base_x, base_y, place_red_z)
    
    env.pick("block_1")
    env.place(base_x, base_y, place_green_z)
    
    env.pick("block_2")
    env.place(base_x, base_y, place_blue_z)



build_bridge()

C_copy.view(True)