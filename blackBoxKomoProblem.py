import re
import ast
import numpy as np
import robotic as ry


def str_to_np_array(text: str) -> np.ndarray:
    return np.array(ast.literal_eval(text), dtype=np.float32)

class BlackBoxKomoProblem:

    def __init__(self, text: str,
                 scales: bool=False,
                 times: bool=False,
                 targets: bool=False,
                 features_to_be_optimized: list=["ry.FS.position"]):
        
        lines = text.split("\n")
        self.objectives = []
        for line in lines:
            if line.startswith("komo."):
                lists = re.findall(r'\[.*?\]', line)
                params_in_objective = re.split(r',\s*(?![^\[\]]*\])', line)

                objective_as_dict = {
                    "times": str_to_np_array(lists[0]),
                    "feature": params_in_objective[1].replace(" ", ""),
                    "frames": ast.literal_eval(lists[1]),
                    "type": params_in_objective[3].replace(" ", ""),
                }
                if not objective_as_dict["feature"] in ["ry.FS.accumulatedCollisions", "ry.FS.jointLimits"]:
                    objective_as_dict["scale"] = str_to_np_array(lists[2])
                    if len(lists) > 3:
                        objective_as_dict["target"] = params_in_objective[-1].replace(" ", "").replace(")", "")
                        if objective_as_dict["target"].startswith("q"):
                            objective_as_dict["target"] = np.zeros((7,))
                        else:
                            objective_as_dict["target"] = str_to_np_array(objective_as_dict["target"])

                self.objectives.append(objective_as_dict)

            elif line.startswith("komo = "):
                init_params = line.split(",")
                self.komo_init_params = [float(p) for p in init_params[1:-1]]
                self.komo_init_params.append(bool(init_params[-1].replace(")", "")))

        self.scales = scales
        self.times = times
        self.targets = targets
        self.features_to_be_optimized = features_to_be_optimized
        self.C = None

    def run_komo(self) -> np.ndarray:

        komo = ry.KOMO(self.C, *self.komo_init_params)

        for obj in self.objectives:
            if not "scale" in obj.keys():
                komo.addObjective(obj["times"], ast.literal_eval(obj["feature"]), obj["frames"], ast.literal_eval(obj["type"]))
            elif not "target" in obj.keys():
                komo.addObjective(obj["times"], ast.literal_eval(obj["feature"]), obj["frames"], ast.literal_eval(obj["type"]), obj["scale"])
            else:
                komo.addObjective(obj["times"], ast.literal_eval(obj["feature"]), obj["frames"], ast.literal_eval(obj["type"]), obj["scale"], obj["target"])

        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()

        observation = np.array([])
        return observation
    
    def reset(self) -> tuple[np.ndarray]:
        
        action = np.array([])

        if self.scales:
            for obj in self.objectives:
                if obj["feature"] in self.features_to_be_optimized:
                    action = np.concatenate((action, obj["scale"]))
        if self.times:
            for obj in self.objectives:
                if obj["feature"] in self.features_to_be_optimized:
                    action = np.concatenate((action, obj["times"]))
        if self.targets:
            for obj in self.objectives:
                if obj["feature"] in self.features_to_be_optimized:
                    action = np.concatenate((action, obj["target"]))

        observation = self.run_komo()
        return action, observation
        
    def step(self, action: np.ndarray) -> np.ndarray:

        idx = 0
        for obj in self.objectives:
            if obj["feature"] in self.features_to_be_optimized:
                if self.scales:
                    size = obj["scale"].shape[0]
                    obj["scale"] = action[idx:idx+size]
                    idx += size

                if self.times:
                    size = obj["times"].shape[0]
                    obj["times"] = action[idx:idx+size]
                    idx += size
                    
                if self.targets:
                    size = obj["target"].shape[0]
                    obj["target"] = action[idx:idx+size]
                    idx += size

        observation = self.run_komo()
        return observation
    

if __name__ == "__main__":
    komo_text = """
komo = ry.KOMO(C, 1, 1, 0, True)

komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

komo.addObjective([], ry.FS.positionDiff, ['l_gripper', 'box'], ry.OT.eq, [1e1])
komo.addObjective([], ry.FS.scalarProductXX, ['l_gripper', 'box'], ry.OT.eq, [1e1], [0])
komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'box'], ry.OT.eq, [1e1], [0])
komo.addObjective([], ry.FS.distance, ['l_palm', 'box'], ry.OT.ineq, [1e1])
"""
    bbk = BlackBoxKomoProblem(komo_text)
    action, observation = bbk.reset()
    observation = bbk.step(action)
    