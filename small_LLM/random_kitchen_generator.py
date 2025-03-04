import time
import rowan
import numpy as np
import robotic as ry
from load_random_plys import load_rand_objs, load_single_from_type


def generate_random_kitchen(seed: int=None, root_path: str="") -> ry.Config:

    if seed == None:
        seed = np.random.randint(0, 100_000)
    print("Seed:", seed)
    np.random.seed(seed)

    C = ry.Config()
    C.addFile(ry.raiPath("scenarios/pandaOmnibase.g"))

    z_offset = .5
    omnibase_frame = C.getFrame("omnibase_world")
    omnibase_frame_z = omnibase_frame.getPosition()[2]
    omnibase_frame.setPosition([0, 0, omnibase_frame_z+z_offset])

    #C = load_single_from_type(C, pos=[1., 1., .5], root_path="rai_jointed/fixtures/stovetops")
    
    # C = load_single_from_type(C, pos=[1., -1.5, .8], root_path="rai_jointed/fixtures/toasters")
    C = load_single_from_type(C, pos=[1., 1., .5+z_offset], rot=rowan.from_euler(0, 0, 0), root_path=f"{root_path}rai_jointed/fixtures/stoves")
    C = load_single_from_type(C, pos=[2., 1., .9+z_offset], rot=rowan.from_euler(0, 0, 0), root_path=f"{root_path}rai_jointed/fixtures/sinks")
    C.addFrame("counter_sink") \
    .setPosition([2., 1., .36+z_offset]) \
    .setShape(ry.ST.ssBox, [1.2, .8, .7, .001])


    C = load_single_from_type(C, pos=[3., -1., .5+z_offset], rot=rowan.from_euler(np.pi, 0, 0), root_path=f"{root_path}rai_jointed/fixtures/ovens")
    C = load_single_from_type(C, pos=[2., -1., .5+z_offset], rot=rowan.from_euler(np.pi, 0, 0), root_path=f"{root_path}rai_jointed/fixtures/dishwashers")

   # Counter with random objects
    C = load_rand_objs(C, dims=[1., .6, 0.], pos=[1., -1., .8+z_offset], root_path=root_path)
    C.addFrame("counter") \
        .setPosition([1., -1, .35+z_offset]) \
        .setShape(ry.ST.ssBox, [1.2, .8, .7, .001])
    
    C.addFrame("floor") \
        .setPosition([0, 0, -.025+z_offset]) \
        .setShape(ry.ST.ssBox, [10, 10, .05, .001]) \
        .setContact(1)
    
    return C


if __name__ == "__main__":

    C = generate_random_kitchen(seed=84979, root_path="/home/denis/rai-robotModels/robocasa/")

    C.view(True)
    # C.animate()
    # C.view(True)
    
    # S = ry.Simulation(C, ry.SimulationEngine.physx, verbose=0)

    # tau=.01
    # for i in range(200):
    #     time.sleep(tau)
    #     S.step([], tau,  ry.ControlMode.none)
    #     C.view()
