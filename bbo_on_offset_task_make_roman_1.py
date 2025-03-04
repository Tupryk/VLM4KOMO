import cma
import numpy as np
import robotic as ry

from bbo_on_offsets import LLM_OUT_BBO


if __name__ == "__main__":
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

    raw_out = """def build_bridge():
    # Get block objects
    red_block = getObj("block_0")
    green_block = getObj("block_1")
    blue_block = getObj("block_2")
    
    # Compute placement positions based on object sizes
    center_x, center_y = red_block.pos.x, red_block.pos.y
    
    # Placeholder optimization values
    horizontal_offset = PLACEHOLDER_FLOAT
    vertical_offset = PLACEHOLDER_FLOAT
    
    # Position for the bottom horizontal block
    bottom_x, bottom_y = center_x, center_y
    
    # Position for the vertical block
    vertical_x, vertical_y = center_x, center_y
    vertical_z = red_block.size.z + vertical_offset  # Adjust placement height
    
    # Position for the top horizontal block
    top_x, top_y = center_x, center_y
    top_z = vertical_z + green_block.size.z + horizontal_offset  # Adjust top block height
    
    # Pick and place the bottom horizontal block
    pick("block_0")
    place(bottom_x, bottom_y, rotated=True)
    
    # Pick and place the vertical block
    pick("block_1")
    place(vertical_x, vertical_y, z=vertical_z, rotated=False)
    
    # Pick and place the top horizontal block
    pick("block_2")
    place(top_x, top_y, z=top_z, rotated=True)

"""
    
    def cost_func(C: ry.Config):


        block_0 = C.getFrame("block_0")
        block_1 = C.getFrame("block_1")
        block_2 = C.getFrame("block_2")

        block_0_error = 0
        block_1_error = 0
        block_2_error = 0

        block_0_err_alignment = np.abs(np.round(C.eval(ry.FS.scalarProductZZ, ["block_0", "table"])[0][0], 1))
        block_1_err_alignment = np.abs(np.round(C.eval(ry.FS.scalarProductXZ, ["block_1", "table"])[0][0], 1))
        block_2_err_alignment = np.abs(np.round(C.eval(ry.FS.scalarProductZZ, ["block_2", "table"])[0][0], 1))
        
        block_1_err_height = 10*np.abs(block_1.getPosition()[2]-block_0.getPosition()[2]-.08) 

        block_2_err_height = 10*np.abs(block_2.getPosition()[2]-block_1.getPosition()[2]-.08) 

        block_01_err_alignment = np.abs(C.eval(ry.FS.scalarProductZZ, ["block_0", "block_1"])[0][0])
        block_02_err_alignment = np.abs(C.eval(ry.FS.scalarProductZZ, ["block_0", "block_2"])[0][0])

        block_0_error += block_0_err_alignment
        block_1_error += block_1_err_alignment + block_1_err_height #+ block_01_err_alignment
        block_2_error += block_2_err_alignment + block_2_err_height #+ block_02_err_alignment

        total_cost = block_0_error + block_1_error + block_2_error  

        print("+-------------------------------+")
        print("Block 0/2 error alignment:", block_02_err_alignment)
        
        print("Block red error alignment:", block_0_err_alignment)
        print("Block green error alignment:", block_1_err_alignment)
        print("Block blue error alignment:", block_2_err_alignment)

        print("Total cost: ", total_cost)
        print(np.abs(block_1.getPosition()[2]-block_0.getPosition()[2]) )
        if total_cost<.05:
            C.view(True)
        print("+-------------------------------+")
        return total_cost
    
    problem = LLM_OUT_BBO(cost_func, raw_out, C)
    inital_state = problem.get_initial_state()
    options = {
        'popsize': 7,
        'maxiter': 50,
        'maxfevals': 5000,
        'tolfun': 1e-4,
        'tolx': 1e-5
    }
    
    result = cma.fmin(problem.compute_cost, inital_state, sigma0=.1, options=options)
    print("-- result")
    print(result[0])
    