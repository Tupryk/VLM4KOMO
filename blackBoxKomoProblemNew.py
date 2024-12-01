import robotic as ry
import re
import ast
import cma
import time
import numpy as np
from line_profiler import profile
from simulator import Simulator


def str_to_np_array(text: str) -> np.ndarray:
    return np.array(ast.literal_eval(text), dtype=np.float32)

class BlackBoxKomoProblem:

    def __init__(self,
                 C: ry.Config,
                 text: str,
                 scales: bool=False,
                 times: bool=False,
                 targets: bool=False,
                 features_to_be_optimized: list=["ry.FS.positionDiff"],
                 engine: ry.SimulationEngine = ry.SimulationEngine.physx,
                 verbose = 0
                 ):
        
        lines = text.split("\n")
        self.objectives = []
        self.ctrl_objectives = []

        for line in lines:
            if line.startswith("komo."):
                # TODO for other paramers (target, deltaFromSlice, deltaToSlice), make work if scale is not given
                if line.startswith("komo.addControlObjective"):

                    match = re.search(r'\((\[.*?\]|[^,]*),\s*([^,]*),\s*([^)]*)', line)
                    param1, param2, param3 = match.groups()

                    ctrl_objective_as_dict = {
                        "times": ast.literal_eval(param1),
                        "order": int(param2),
                        "scale": float(param3),
                        # "target":
                        # "deltaFromSlice":
                        # "deltaToSlice":                        
                    }
                    self.ctrl_objectives.append(ctrl_objective_as_dict)
                    continue
                lists = re.findall(r'\[.*?\]', line)
                params_in_objective = re.split(r',\s*(?![^\[\]]*\])', line)

                objective_as_dict = {
                    "times": str_to_np_array(lists[0]),
                    "feature": params_in_objective[1].replace(" ", ""),
                    "frames": ast.literal_eval(lists[1]),
                    "type": params_in_objective[3].replace(" ", "").replace(")", ""),
                }
                if not objective_as_dict["feature"] in ["ry.FS.accumulatedCollisions", "ry.FS.jointLimits"]:
                    objective_as_dict["scale"] = str_to_np_array(lists[2].replace(" ", "").replace(")", ""))
                    if len(lists) > 3:
                        objective_as_dict["target"] = params_in_objective[-1].replace(" ", "").replace(")", "")
                        if objective_as_dict["target"].startswith("q"):
                            objective_as_dict["target"] = np.zeros((7,))
                        else:
                            objective_as_dict["target"] = str_to_np_array(objective_as_dict["target"])

                self.objectives.append(objective_as_dict)

            elif line.startswith("komo = "):
                init_params = line.split(",")
                self.komo_init_params = [int(p) for p in init_params[1:-1]]
                self.komo_init_params[0] = float(self.komo_init_params[0])
                self.komo_init_params.append(bool(init_params[-1].replace(")", "")))

        self.scales = scales
        self.times = times
        self.targets = targets
        self.features_to_be_optimized = features_to_be_optimized
        self.C = C
        self.verbose = verbose

        #self._sim = ry.Simulation(C, engine, verbose=verbose)
        #self.init_state = self._sim.getState()
    
    def get_cost(self, C) -> float:
        return np.linalg.norm(C.getFrame("target").getPosition()-C.getFrame("blob").getPosition())

    def run_komo(self) -> np.ndarray:
        C2 = ry.Config()
        C2.addConfigurationCopy(self.C)
        komo  = ry.KOMO(C2, 2, 64, 2, True)
        
        for ctrObj in self.ctrl_objectives:
            komo.addControlObjective(ctrObj["times"], ctrObj["order"], ctrObj["scale"])

        for obj in self.objectives:
            if not "scale" in obj.keys():
                komo.addObjective(obj["times"], eval(obj["feature"]), obj["frames"], eval(obj["type"]))
            elif not "target" in obj.keys():

                komo.addObjective(obj["times"], eval(obj["feature"]), obj["frames"], eval(obj["type"]), obj["scale"])
            else:
                komo.addObjective(obj["times"], eval(obj["feature"]), obj["frames"], eval(obj["type"]), obj["scale"], obj["target"])

        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
        q = komo.getPath()

        #C2.view(True)
        #self.run_trajectory(q, 128, real_time=True)
        sim = Simulator(C2)
        xs, qs, xdots, qdots = sim.run_trajectory(q, 2, real_time=True)

        #xs[]
        C2.getFrame("blob").setPosition(xs[-1][-1][:3])

        observation = self.get_cost(C2)
        del C2
        return observation
    
    def reset(self) -> tuple[np.ndarray]:
        
        action = np.array([])

        for obj in self.objectives:
            if obj["feature"] in self.features_to_be_optimized:

                if self.scales and "scale" in obj.keys():
                    action = np.concatenate((action, obj["scale"]))

                if self.times:
                    action = np.concatenate((action, obj["times"]))

                if self.targets and "target" in obj.keys():
                    action = np.concatenate((action, obj["target"]))

        observation = self.run_komo()
        return action, observation
    
        
    def step(self, action: np.ndarray) -> np.ndarray:

        idx = 0
        for i, obj in enumerate(self.objectives):
            if obj["feature"] in self.features_to_be_optimized:

                if self.scales and "scale" in obj.keys():
                    size = obj["scale"].shape[0]
                    obj["scale"] = action[idx:idx+size]
                    idx += size

                if self.times:
                    size = obj["times"].shape[0]
                    obj["times"] = action[idx:idx+size]
                    idx += size
                    
                if self.targets and "target" in obj.keys():
                    size = obj["target"].shape[0]
                    obj["target"] = action[idx:idx+size]
                    idx += size
                
                self.objectives[i] = obj

        observation = self.run_komo()
        return observation
    
    


if __name__ == "__main__":
    komo_text = """
komo = ry.KOMO(C, 1, 1, 0, True)

komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

komo.addObjective([1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e-3], [.1, .1, .8])

"""
    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    C.addFrame('refTarget'). setShape(ry.ST.marker, [.2]) .setPosition([.4, .4, .9])

    bbk = BlackBoxKomoProblem(C, komo_text, targets=True, scales=True, verbose=3)
    action, observation = bbk.reset()


    options = {
    'popsize': 7,
    'maxiter': 50,
    'maxfevals': 5000,
    'tolfun': 1e-4,
    'tolx': 1e-5
    }

    result = cma.fmin(bbk.step, action, sigma0=.1, options=options)
    