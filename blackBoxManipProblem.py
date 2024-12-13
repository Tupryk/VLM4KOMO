import robotic as ry
import re
import ast
import time
import numpy as np
import manipulation as manip

def str_to_np_array(text: str) -> np.ndarray:
    return np.array(ast.literal_eval(text), dtype=np.float32)

class BlackBoxManipProblem:

    def __init__(self,
                 C: ry.Config,
                 text: str,
                 scales: bool=False,
                 times: bool=False,
                 targets: bool=False,
                 features_to_be_optimized: list=["ry.FS.positionDiff"],
                 verbose = 0
                 ):
        
        lines = text.split("\n")


        for line in lines:
            if ".setup_" in line:
                
                pattern = r"\w+\.\w+\((.+)\)"
                match = re.search(pattern, line)

                params = match.group(1)
    
                # Split parameters by commas, considering possible key-value pairs
                param_list = [param.strip() for param in params.split(',')]
                print(param_list)

                start_idx = line.find(".setup_") + len(".setup_")  
                end_idx = line.find("(", start_idx)
                self.manipulation_type={line[start_idx:end_idx].strip(): param_list} # setuptype: its parameters as list

        self.scales = scales
        self.times = times
        self.targets = targets
        self.features_to_be_optimized = features_to_be_optimized
        self.C = C
        self.verbose = verbose
    
    def param_list_to_args_and_kwargs(self, params):

        # Convert strings to Python objects using `eval`
        evaluated_params = []
        for param in params:
            if "=" in param:  # For keyword arguments
                key, value = param.split("=", 1)
                evaluated_params.append(f"{key}={eval(value)}")
            else:  # For positional arguments
                evaluated_params.append(eval(param))

        # Separate positional and keyword arguments
        positional_args = []
        keyword_args = {}
        for param in evaluated_params:
            if isinstance(param, str) and "=" in param:
                key, value = param.split("=")
                keyword_args[key] = eval(value)
            else:
                positional_args.append(param)

        return positional_args, keyword_args
    
    def build_manip(self):
        man = manip.ManipulationModelling()
        key, value = self.manipulation_type.popitem()
        args, kwargs = self.param_list_to_args_and_kwargs(value[1:])

        if key == "inverse_kinematic":
            man.setup_inverse_kinematics(self.C, *args, **kwargs)

        if key == "sequence":
            man.setup_sequence(self.C, *args, **kwargs)

        if key == "motion":
            man.setup_motion(self.C, *args, **kwargs)

        if key == "pick_and_place_waypoints":
            man.setup_pick_and_place_waypoints(self.C, *args, **kwargs)

        if key == "point_to_point_motion":
            man.setup_point_to_point_motion(self.C, *args, **kwargs)

    


if __name__ == "__main__":
    komo_text = """
    man = manip.ManipulationModelling()
    man.setup_pick_and_place_waypoints(C, "l_gripper", "box", homing_scale=1e-1)
"""
    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    C.addFrame('refTarget'). setShape(ry.ST.marker, [.2]) .setPosition([.4, .4, .9])
    C.addFrame('box'). setShape(ry.ST.ssBox, [.02, .02, .02, .01]) .setPosition([.4, .4, .9])

    bbk = BlackBoxManipProblem(C, komo_text, targets=True, scales=True, verbose=3)
    bbk.build_manip()


