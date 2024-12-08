import robotic as ry
import re
import ast
import cma
import time
import numpy as np
#from simulator import Simulator


class BlackBoxAbstractProblem:

    def __init__(self,
                 C: ry.Config,
                 text: str,
                 features_to_be_optimized: list=["target"],
                 verbose = 0
                 ):
        
        lines = text.split("\n")
        self.objectives = []

        for line in lines:
            if line.strip().startswith("push"):
                match = re.match(r'\s*push\s*\(\s*(["\']?.+?["\']?)\s*,\s*(\[.*?\])\s*\)\s*', line)
                if match:
                    first_param_raw = match.group(1).strip()
                    if first_param_raw.startswith(("'", '"')) and first_param_raw.endswith(("'", '"')):
                        first_param = ast.literal_eval(first_param_raw)
                    else:
                        first_param = first_param_raw

                    target = ast.literal_eval(match.group(2))

                    objective_dict={"feature":"push", "frame":first_param, "target":target}
                    self.objectives.append(objective_dict)
                else:
                    raise ValueError(f"String format does not match: {line}")

            if line.strip().startswith("pick"):
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
                match = re.match(r"\s*(?:\w+\s*)?\(\s*(\[.*?\])\s*(?:,\s*(.+?))?\s*\)\s*", line)
    
                if match:
                    target = ast.literal_eval(match.group(1))
                    
                    second_param = match.group(2)
                    if second_param:
                        second_param = float(second_param.strip())  # Convert to float if present
                    else:
                        second_param = .0

                    objective_dict={"feature":"place", "target":target, "z": second_param}

                    self.objectives.append(objective_dict)

                else:
                    raise ValueError("String format does not match.")


        self.features_to_be_optimized = features_to_be_optimized
        self.C = C
        self.verbose = verbose
    
    def get_cost(self, C, xs) -> float:

        C.getFrame("box").setPosition(xs[-1][-1][:3])

        return np.linalg.norm(C.getFrame("target").getPosition()-C.getFrame("box").getPosition())


    def build_abstract(self, C: ry.Config) -> ry.KOMO:
        #Motion = AbstractMotion(self.C, *self.komo_init_params)

        # for obj in self.objectives:
        #     if obj["feature"] == "pick":
        #         Motion.pick(obj["frame"], yaw=obj["yaw"])
        #     elif obj["feature"] == "place":
        #         Motion.place(obj["frame"], target=obj["target"], z_offset=obj["z"])
        #     elif obj["feature"] == "push":
        #         Motion.push(obj["frame"], target=obj["target"])
        
        # return Motion
        pass

    # def run_komo(self) -> np.ndarray:
    #     C2 = ry.Config()
        # C2.addConfigurationCopy(self.C)
        
        # komo = self.build_abstract(C2)

        # ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
        # q = komo.getPath()

        # sim = Simulator(C2)
        # xs, qs, xdots, qdots = sim.run_trajectory(q, 2, real_time=False)

        # observation = self.get_cost(C2, xs)
        # del C2
        # print(observation)
        # return observation
    
    def reset(self) -> tuple[np.ndarray]:
        
        action = np.array([])

        for obj in self.objectives:
            if obj["feature"] in self.features_to_be_optimized:

                if "target" in obj.keys():
                    action = np.concatenate((action, obj["target"]))

                if "z":
                    action = np.concatenate((action, obj["z"]))

                if "yaw" in obj.keys():
                    action = np.concatenate((action, obj["yaw"]))

        observation = self.run_komo()
        return action, observation
    
    def set_action(self, action: np.ndarray):
        idx = 0
        for i, obj in enumerate(self.objectives):
            if obj["feature"] in self.features_to_be_optimized:

                if "target" in obj.keys():
                    size = obj["target"].shape[0]
                    obj["target"] = action[idx:idx+size]
                    idx += size

                if "z" in obj.keys():
                    size = obj["z"].shape[0]
                    obj["z"] = action[idx:idx+size]
                    idx += size
                    
                if "yaw" in obj.keys():
                    size = obj["yaw"].shape[0]
                    obj["yaw"] = action[idx:idx+size]
                    idx += size
                
                self.objectives[i] = obj
        
    def step(self, action: np.ndarray) -> np.ndarray:
        self.set_action(action)
        observation = self.run_komo()
        return observation
    


if __name__ == "__main__":
    komo_text = """
pick(box)
place([.1, 0])
push(box, [.05, .05])
"""
    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    C.addFrame('refTarget'). setShape(ry.ST.marker, [.2]) .setPosition([.4, .4, .9])

    bbk = BlackBoxAbstractProblem(C, komo_text, verbose=3)
    print(bbk.objectives)
    #action, observation = bbk.reset()


    # options = {
    # 'popsize': 7,
    # 'maxiter': 50,
    # 'maxfevals': 5000,
    # 'tolfun': 1e-4,
    # 'tolx': 1e-5
    # }

    # result = cma.fmin(bbk.step, action, sigma0=.1, options=options)
    