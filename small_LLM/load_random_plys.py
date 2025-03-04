import os
import time
import numpy as np
import robotic as ry


def load_single_from_type(C: ry.Config, new_name: str="", root_path: str="rai_jointed", pos: list[float]=[0., 0., 0.], rot: list[float]=[1., 0., 0., 0.]):

    g_files = []
    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file.endswith('.g'):
                g_path = os.path.join(root, file)
                with open(g_path, "r") as file:
                    g_files.append(g_path)

    if not len(g_files):
        print("No files found inside of", root_path)
        return C
    
    print(f"Files found inside {root_path}: {len(g_files)}")

    selected_file = np.random.choice(g_files)

    if not new_name:
        new_name = os.path.basename(os.path.normpath(root_path))
        
    with open(selected_file, "r") as file:
        content = file.read()

    updated_content = content.replace("object(", f"{new_name}(")
    updated_content = updated_content.replace("object ", f"{new_name} ")
    updated_content = updated_content.replace("(object)", f"({new_name})")

    with open(selected_file, "w") as file:
        file.write(updated_content)
    
    C.addFile(selected_file)
    C.getFrame(new_name).setPosition(pos).setQuaternion(rot)

    return C

def load_rand_objs(C: ry.Config, amount: int=5, dims: list[float]=[.5, .5, .5], pos: list[float]=[0., 0., .5], root_path: str=""):
    
    root_path += "rai_plys/aigen_objs"
    
    ply_files = []
    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file.endswith('.ply'):
                ply_path = os.path.join(root, file)
                ply_files.append(ply_path)

    if len(ply_files) > amount:
        selected_files = np.random.choice(ply_files, amount, replace=False)
    else:
        selected_files = ply_files.copy()

    lines = []
    for ply_path in selected_files:
        folder_path = os.path.dirname(ply_path)
        name = os.path.basename(folder_path)
        
        x = np.random.uniform(pos[0]-dims[0]*.5, pos[0]+dims[0]*.5)
        y = np.random.uniform(pos[1]-dims[1]*.5, pos[1]+dims[1]*.5)
        z = np.random.uniform(pos[2]-dims[2]*.5, pos[2]+dims[2]*.5)
        
        line = f"""{name} {{X: "t({x} {y} {z})", shape: mesh, mesh: "{ply_path}", mass: 1.}}\n"""
        lines.append(line)

    with open("./random_objs.g", "w") as file:
        file.writelines(lines)

    C.addFile("./random_objs.g")
        
    return C


if __name__ == "__main__":

    C = ry.Config()
    C = load_rand_objs(C)
    C.view(True)

    S = ry.Simulation(C, ry.SimulationEngine.physx, verbose=0)

    tau=.01
    for i in range(200):
        time.sleep(tau)
        S.step([], tau,  ry.ControlMode.none)
        C.view()
