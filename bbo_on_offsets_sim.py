import cma
import numpy as np
import robotic as ry
import os 


from utils import cleanup_highlvl_func
from sim_functions import RobotEnviroment

class LLM_OUT_BBO:
    def __init__(self, cost_func, llm_out: str, C: ry.Config):
        self.cost_func = cost_func
        self.llm_out = cleanup_highlvl_func(llm_out, compute_collisions=True, visuals=False)
        self.C = C
        self.input_dim = llm_out.count("PLACEHOLDER_FLOAT")
        self.C_copy = None

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
        
        # Get the number of existing files in the outputs directory
        output_dir = "outputs"
        i = len([name for name in os.listdir(output_dir) if os.path.isfile(os.path.join(output_dir, name))])

        if cost < 0.01:
            with open(f"{output_dir}/filled_llm_output{i}.txt", "w") as file:
                file.write(filled_llm_output)


        self.C_copy = C_copy

        del C_copy
        return cost

