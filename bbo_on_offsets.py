import cma
import numpy as np
import robotic as ry

from utils import cleanup_highlvl_func
from high_level_funcs_old import RobotEnviroment


class LLM_OUT_BBO:
    def __init__(self, cost_func, llm_out: str, C: ry.Config):
        self.cost_func = cost_func
        self.llm_out = cleanup_highlvl_func(llm_out, compute_collisions=False, visuals=False)
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
        

        C_copy = ry.Config()
        C_copy.addConfigurationCopy(self.C)

        filled_llm_output = self.set_params(params)
        filled_llm_output = filled_llm_output.replace("(C,", "(C_copy,")

        exec(filled_llm_output)

        cost = self.cost_func(C_copy)
        C_copy.view(True)
        
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
        
    raw_out = f"""def build_bridge():
    # Get object properties
    red_block = getObj('block_red')
    green_block = getObj('block_green')
    blue_block = getObj('block_blue')

    # Determine the blocks to use as vertical supports and the horizontal block
    vertical_block1 = red_block
    vertical_block2 = green_block
    horizontal_block = blue_block

    # Calculate the distance for the vertical blocks
    vertical_spacing = horizontal_block.size.x + PLACEHOLDER_FLOAT  # Add a small buffer (_FLOAT_)

    # Calculate positions for the vertical blocks
    vertical1_x = vertical_block1.pos.x
    vertical1_y = vertical_block1.pos.y

    vertical2_x = vertical1_x + vertical_spacing
    vertical2_y = vertical1_y

    # Calculate position for the horizontal block
    horizontal_x = (vertical1_x + vertical2_x) / 2  # Centered between vertical blocks
    horizontal_y = vertical1_y
    horizontal_z = vertical_block1.size.z + horizontal_block.size.z / 2 + PLACEHOLDER_FLOAT  # On top of vertical blocks

    # Build the bridge
    # Place first vertical block
    pick('block_red')
    place(vertical1_x, vertical1_y, z=vertical_block1.size.z / 2)

    # Place second vertical block
    pick('block_green')
    place(vertical2_x, vertical2_y, z=vertical_block2.size.z / 2)

    # Place horizontal block
    pick('block_blue')
    place(horizontal_x, horizontal_y, z=horizontal_z, rotated=True)
"""
    
    def cost_func(C: ry.Config):
        
        red_block = C.getFrame("block_red")
        green_block = C.getFrame("block_green")
        blue_block = C.getFrame("block_blue")

        red_block_error = 0
        green_block_error = 0
        blue_block_error = 0

        # Positions
        green_block_error += np.linalg.norm(green_block.getPosition() - red_block.getPosition()) - 0.012 
        # blue_block_error += np.linalg.norm(blue_block.getPosition() - (green_block.getPosition() + red_block.getPosition())*.5)
        blue_block_error += np.abs(blue_block.getPosition()[0] - red_block.getPosition()[0] + .06 + .02)

        # Rotations
        # red_block_error += np.linalg.norm(C.eval(ry.FS.vectorZ, ["block_red"])[0][0] - np.array([0., 0., 1.]))
        # green_block_error += np.linalg.norm(C.eval(ry.FS.vectorZ, ["block_green"])[0][0] - np.array([0., 0., 1.]))
        # blue_block_error += C.eval(ry.FS.scalarProductZZ, ["block_blue", "table"])[0][0]

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
        'popsize': 32,
        'maxiter': 50,
        'maxfevals': 5000,
        'tolfun': 1e-4,
        'tolx': 1e-5
    }
    result = cma.fmin(problem.compute_cost, inital_state, sigma0=.1, options=options)
    print("-- result")
    print(result[0])
    