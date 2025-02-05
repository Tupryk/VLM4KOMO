import cma
import numpy as np
import robotic as ry

from utils import cleanup_highlvl_func
from high_level_funcs_old import RobotEnviroment

class LLM_OUT_BBO:
    def __init__(self, cost_func, llm_out: str, C: ry.Config):
        self.cost_func = cost_func
        self.llm_out = cleanup_highlvl_func(llm_out, compute_collisions=True, visuals=False)
        self.C = C
        self.input_dim = llm_out.count("PLACEHOLDER_FLOAT")
        
    def get_initial_state(self) -> np.ndarray:
        initial_state = np.zeros((self.input_dim,))
        return initial_state
    
    def set_params(self, params: list[float]) -> str:
        filled_llm_output = str(self.llm_out)
        for value in params:
            filled_llm_output = filled_llm_output.replace("PLACEHOLDER_FLOAT", str(value), 1)
        return filled_llm_output

    def compute_cost(self, params: list[float]) -> float:
        
        global C_copy
        C_copy = ry.Config()
        C_copy.addConfigurationCopy(self.C)
        filled_llm_output = self.set_params(params)
        filled_llm_output = filled_llm_output.replace("(C,", "(C_copy,")
        with open("filled_llm_output.txt", "w") as file:
            # Write the string to the file
            file.write(filled_llm_output)

        # print("Executing:\n", filled_llm_output)
        exec(filled_llm_output, globals(), locals())
        cost = self.cost_func(C_copy)
        C_copy.view(False)
        
        del C_copy
        return cost


if __name__ == "__main__":
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

    raw_out = """def build_bridge():
    # Get object parameters
    red_block = getObj("block_red")
    green_block = getObj("block_green")
    blue_block = getObj("block_blue")
    
    # Determine positions for vertical support blocks
    support_x_offset = blue_block.size.x / 2 + red_block.size.x / 2 + PLACEHOLDER_FLOAT
    support1_x = blue_block.pos.x - support_x_offset
    support2_x = blue_block.pos.x + support_x_offset
    support_y = blue_block.pos.y + PLACEHOLDER_FLOAT
    support_z = red_block.size.z / 2  # Place directly on the table
    
    # Place vertical support blocks
    pick("block_red")
    place(support1_x, support_y, support_z)
    
    pick("block_green")
    place(support2_x, support_y, support_z)
    
    # Place horizontal block on top
    bridge_z = support_z + red_block.size.z / 2 + blue_block.size.z / 2 + PLACEHOLDER_FLOAT
    pick("block_blue")
    place(blue_block.pos.x, support_y, bridge_z, rotated=True, yaw=PLACEHOLDER_FLOAT)
"""
    
    def cost_func(C: ry.Config):
        
        red_block = C.getFrame("block_red")
        green_block = C.getFrame("block_green")
        blue_block = C.getFrame("block_blue")

        red_block_error = 0
        green_block_error = 0
        blue_block_error = 0

        # Positions
        green_block_error += np.abs(np.linalg.norm(green_block.getPosition() - red_block.getPosition()) - 0.12)
        # blue_block_error += np.linalg.norm(blue_block.getPosition() - (green_block.getPosition() + red_block.getPosition())*.5)
        blue_block_error += np.abs((blue_block.getPosition()[2] - red_block.getPosition()[2]) - .06 - .02)
        # blue_block_error = np.abs(blue_block.getPosition()[2]-.8)

        # Rotations
        # red_block_error += np.linalg.norm(C.eval(ry.FS.vectorZ, ["block_red"])[0][0] - np.array([0., 0., 1.]))
        # green_block_error += np.linalg.norm(C.eval(ry.FS.vectorZ, ["block_green"])[0][0] - np.array([0., 0., 1.]))
        blue_block_error += C.eval(ry.FS.scalarProductZZ, ["block_blue", "table"])[0][0]

        total_cost = red_block_error + green_block_error + blue_block_error

        print("+-------------------------------+")
        print("Red block error: ", red_block_error)
        print("Green block error: ", green_block_error)
        print("Blue block error: ", blue_block_error)
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
    