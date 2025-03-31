import cma
import numpy as np
import robotic as ry

from bbo_on_offsets_sim import LLM_OUT_BBO


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
            .setShape(ry.ST.ssBox, size=[.04, .12, .04, 0.005]) \
            .setColor(color) \
            .setContact(1) \
            .setMass(.1)

    raw_out = """def build_bridge():
    # Get block objects
    # Get objects
    red_block = getObj("block_0")
    green_block = getObj("block_1")
    blue_block = getObj("block_2")
    
    # Compute positions
    center_x = (red_block.pos.x + green_block.pos.x + blue_block.pos.x) / 3
    center_y = (red_block.pos.y + green_block.pos.y + blue_block.pos.y) / 3
    
    # Define triangle corner placements
    place_positions = [
        (red_block.pos.x + PLACEHOLDER_FLOAT, red_block.pos.y + PLACEHOLDER_FLOAT, red_block.pos.z, 0.0),
        (green_block.pos.x + PLACEHOLDER_FLOAT, green_block.pos.y + PLACEHOLDER_FLOAT, green_block.pos.z, 2.094),
        (blue_block.pos.x + PLACEHOLDER_FLOAT, blue_block.pos.y + PLACEHOLDER_FLOAT, blue_block.pos.z, -2.094)
    ]
    
    # Pick and place the blocks
    for block, (x, y, z, yaw) in zip(["block_0", "block_1", "block_2"], place_positions):
        place(block, x, y, z, yaw=yaw)

"""
    
    def cost_func(C: ry.Config):


        block_0 = C.getFrame("block_0")
        block_1 = C.getFrame("block_1")
        block_2 = C.getFrame("block_2")

        block_0_error = np.abs(C.eval(ry.FS.positionRel, ["block_0", "block_1"])[0][0]-C.eval(ry.FS.positionRel, ["block_0", "block_2"])[0][0])
        block_1_error = np.abs(C.eval(ry.FS.positionRel, ["block_1", "block_0"])[0][0]-C.eval(ry.FS.positionRel, ["block_1", "block_2"])[0][0])
        block_2_error = np.abs(C.eval(ry.FS.positionRel, ["block_2", "block_0"])[0][0]-C.eval(ry.FS.positionRel, ["block_2", "block_1"])[0][0])

        block_0_error += np.abs(C.eval(ry.FS.positionRel, ["block_0", "block_1"])[0][1]+C.eval(ry.FS.positionRel, ["block_0", "block_2"])[0][1])
        block_1_error += np.abs(C.eval(ry.FS.positionRel, ["block_1", "block_0"])[0][1]+C.eval(ry.FS.positionRel, ["block_1", "block_2"])[0][1])
        block_2_error += np.abs(C.eval(ry.FS.positionRel, ["block_2", "block_0"])[0][1]+C.eval(ry.FS.positionRel, ["block_2", "block_1"])[0][1])

        # block_1_error = np.abs(np.abs(C.eval(ry.FS.positionRel, ["block_1", "block_0"])[0])-np.abs(C.eval(ry.FS.positionRel, ["block_1", "block_2"])[0])))
        # block_2_error = np.abs(np.abs(C.eval(ry.FS.positionRel, ["block_2", "block_0"])[0])-np.abs(C.eval(ry.FS.positionRel, ["block_2", "block_1"])[0])))

        block_0_error += 30*(C.eval(ry.FS.negDistance, ["block_0", "block_1"])[0]+.01)**2
        block_1_error += 30*(C.eval(ry.FS.negDistance, ["block_1", "block_2"])[0]+.01)**2
        block_2_error += 30*(C.eval(ry.FS.negDistance, ["block_2", "block_1"])[0]+.01)**2

        total_cost = block_0_error + block_1_error + block_2_error
       
        print("+-------------------------------+")
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
    C.view(True)
    problem.C_copy.view(True)
    print(problem.C_copy.getFrame("block_0").getPosition())
    print(problem.C_copy.getFrame("block_1").getPosition())
    print(problem.C_copy.getFrame("block_2").getPosition())

    print("-- result")
    print(result[0])
    