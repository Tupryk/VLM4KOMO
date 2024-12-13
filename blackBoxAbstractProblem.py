import robotic as ry
import re
import ast
import cma
import time
import numpy as np
from high_level_funcs import RobotEnviroment
from simulator import Simulator


class BlackBoxAbstractProblem:

    def __init__(self,
                 C: ry.Config,
                 text: str,
                 features_to_be_optimized: list=["push"],
                 verbose = 0
                 ):
        
        lines = text.split("\n")
        self.objectives = []

        for line in lines:
            if line.strip().startswith("push"):
                match = re.match(r'\s*push\s*\(\s*(["\']?.+?["\']?)\s*,\s*(.+?)\s*,\s*(.+?)\s*\)\s*', line)
                if match:
                    first_param_raw = match.group(1).strip()
                    if first_param_raw.startswith(("'", '"')) and first_param_raw.endswith(("'", '"')):
                        first_param = ast.literal_eval(first_param_raw)
                    else:
                        first_param = first_param_raw

                    x = ast.literal_eval(match.group(2).strip())
                    y = ast.literal_eval(match.group(3).strip())

                    objective_dict = {"feature": "push", "frame": first_param, "target": [x, y]}
                    self.objectives.append(objective_dict)
                else:
                    raise ValueError(f"String format does not match: {line}")


            elif line.strip().startswith("pick"):
                match = re.match(r'\s*pick\s*\(\s*(["\']?.+?["\']?)\s*(?:,\s*(\d+(?:\.\d+)?))?\s*\)\s*', line)
                if match:
                    frame_raw = match.group(1).strip()
                    if frame_raw.startswith(("'", '"')) and frame_raw.endswith(("'", '"')):
                        frame = ast.literal_eval(frame_raw)
                    else:
                        frame = frame_raw

                    angle = match.group(2)
                    if angle is not None:
                        angle = float(angle.strip())  # Convert to float if present

                    objective_dict={"feature":"pick", "frame":frame, "yaw":angle}
                    self.objectives.append(objective_dict)
                else:
                    raise ValueError(f"String format does not match: {line}")

            elif line.startswith("place"):
                match = re.match(r'\s*place\s*\(\s*(.+?)\s*,\s*(.+?)\s*(?:,\s*(.+?))?\s*\)\s*', line)

                if match:
                    x = ast.literal_eval(match.group(1).strip())
                    y = ast.literal_eval(match.group(2).strip())

                    # Handle the optional z parameter
                    z = ast.literal_eval(match.group(3).strip()) if match.group(3) else .69

                    objective_dict = {"feature": "place", "target": [x,y], "z": z}
                    self.objectives.append(objective_dict)

                else:
                    raise ValueError("String format does not match.")



        self.features_to_be_optimized = features_to_be_optimized
        self.C = C
        self.verbose = verbose
    
    def get_cost(self, C) -> float:

        #C.getFrame("box").setPosition(xs[-1][-1][:3])

        return np.linalg.norm(C.getFrame("refTarget").getPosition()-C.getFrame("box").getPosition())


    def build_abstract(self, C: ry.Config) -> ry.KOMO:
        Motion = RobotEnviroment(C, visuals=True, verbose=1)

        for obj in self.objectives:
            if obj["feature"] == "pick":
                Motion.pick(obj["frame"])
            elif obj["feature"] == "place":
                Motion.place(x=obj["target"][0], y=obj["target"][1], z=obj["z"])
            elif obj["feature"] == "push":
                Motion.push(obj["frame"], relative_x=obj["target"][0], relative_y=obj["target"][1])
        
        return Motion.path

    def run_high_level(self) -> np.ndarray:
        
        C2 = ry.Config()
        C2.addConfigurationCopy(self.C)
        
        path = self.build_abstract(C2)

        # sim = Simulator(C2)
        # xs, qs, xdots, qdots = sim.run_trajectory(path, 2, real_time=True)

        observation = self.get_cost(C2)
        del C2
        print(observation)
        return observation

    def reset(self) -> tuple[np.ndarray]:
        
        action = np.array([])

        for obj in self.objectives:
            if obj["feature"] in self.features_to_be_optimized:

                if "target" in obj.keys():
                    action = np.concatenate((action, obj["target"]))

                # if "z" in obj.keys():
                #     action = np.concatenate((action, obj["z"]))

                # if "yaw" in obj.keys():
                #     action = np.concatenate((action, obj["yaw"]))

        observation = self.run_high_level()
        return action, observation
    
    def set_action(self, action: np.ndarray):
        idx = 0
        for i, obj in enumerate(self.objectives):
            if obj["feature"] in self.features_to_be_optimized:

                if "target" in obj.keys():
                    size = len(obj["target"])
                    obj["target"] = action[idx:idx+size]
                    idx += size

                # if "z" in obj.keys():
                #     size = 1
                #     obj["z"] = action[idx:idx+size]
                #     idx += size
                    
                # if "yaw" in obj.keys():
                #     size = 1
                #     obj["yaw"] = action[idx:idx+size]
                #     idx += size
                
                self.objectives[i] = obj
        
    def step(self, action: np.ndarray) -> np.ndarray:
        self.set_action(action)
        observation = self.run_high_level()
        return observation
    


if __name__ == "__main__":
    komo_text = """
push(box, .05, .05)
"""


    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    C.addFrame('refTarget'). setShape(ry.ST.marker, [.2]) .setPosition([.3, .3, .69])
    #C.addFrame('lol'). setShape(ry.ST.marker, [.2]) .setPosition([.1, .1, .69])

    C.addFrame("box") \
        .setPosition([.3, 0.05, 0.72]) \
        .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.04, 0.001]) \
        .setColor([1., 0., 0.]) \
        .setContact(1) \
        .setMass(.1)
    C.view()

    bbk = BlackBoxAbstractProblem(C, komo_text, verbose=3)
    print(bbk.objectives)
    action, observation = bbk.reset()
    print(action)
    #bbk.step(action)
    options = {
    'popsize': 7,
    'maxiter': 50,
    'maxfevals': 5000,
    'tolfun': 1e-4,
    'tolx': 1e-5
    }

    result = cma.fmin(bbk.step, action, sigma0=.1, options=options)
    