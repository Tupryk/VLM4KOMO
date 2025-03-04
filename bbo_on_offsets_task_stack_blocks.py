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
    red_block = getObj("block_0")

    support1_x = red_block.pos.x
    support_y = red_block.pos.y
    support_z = red_block.size.z
    
    a=PLACEHOLDER_FLOAT
    b=PLACEHOLDER_FLOAT
    
    pick(f"block_0")
    place(support1_x, support_y, support_z+.02, rotated=True)
    
    pick(f"block_1")
    place(a+support1_x, support_y, support_z+.07, rotated=True)
    
    pick(f"block_2")
    place(b+support1_x, support_y, support_z+.12, rotated=True)

"""
    
    def cost_func(C: ry.Config):
        
        block_0 = C.getFrame("block_0")
        block_1 = C.getFrame("block_1")
        block_2 = C.getFrame("block_2")

        block_0_error = 0
        block_1_error = 0
        block_2_error = 0


        # block_0_err_alignment = np.abs(C.eval(ry.FS.scalarProductZZ, ["block_0", "table"])[0][0])
        # block_1_err_alignment = np.abs(C.eval(ry.FS.scalarProductZZ, ["block_1", "table"])[0][0])
        # block_2_err_alignment = np.abs(C.eval(ry.FS.scalarProductZZ, ["block_2", "table"])[0][0])

        # block_1_err_z_diff = 10*np.abs(block_1.getPosition()[2]-block_0.getPosition()[2]-0.04)
        # block_2_err_z_diff = 10*np.abs(block_2.getPosition()[2]-block_1.getPosition()[2]-0.04)

        # block_1_err_x_diff = 10*np.abs(np.abs(block_1.getPosition()[0]-block_0.getPosition()[0])-0.06) 
        # block_2_err_x_diff = 10*np.abs(np.abs(block_2.getPosition()[0]-block_1.getPosition()[0])-0.06) 

        # block_0_error += block_0_err_alignment 
        # block_1_error += block_1_err_alignment + block_1_err_z_diff +  block_1_err_x_diff
        # block_2_error += block_2_err_alignment + block_2_err_z_diff +  block_2_err_x_diff

        block_2_err_z_diff = 10*np.abs(np.round(block_2.getPosition()[2]-block_1.getPosition()[2]-0.04, 3))+10*np.abs(np.round(block_2.getPosition()[2]-block_0.getPosition()[2]-0.08, 3))
        block_2_err_alignment = np.abs(np.round(C.eval(ry.FS.scalarProductZZ, ["block_2", "table"])[0][0], 1))
        block_1_err_x_diff = np.abs(np.abs(block_1.getPosition()[0]-block_0.getPosition()[0])-0.06) 
        block_2_err_x_diff = np.abs(np.abs(block_2.getPosition()[0]-block_1.getPosition()[0])-0.06) 

        total_cost = block_2_err_z_diff + block_2_err_alignment + block_1_err_x_diff + block_2_err_x_diff

        print("+-------------------------------+")
        # print("Block 0 error alignment:", block_0_err_alignment)
        # print("Block 1 error alignment:", block_1_err_alignment)
        # print("Block 1 error z diff:", block_1_err_z_diff)
        print("Block 1 error x diff:", block_1_err_x_diff)
        print("Block 2 error alignment:", block_2_err_alignment)
        print("Block 2 error z diff:", block_2_err_z_diff)
        print("Block 2 error x diff:", block_2_err_x_diff)
        print("Total cost: ", total_cost)

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
    