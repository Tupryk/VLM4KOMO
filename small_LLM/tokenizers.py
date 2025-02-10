import re
import ast
from lookup import features, frame_names, contraint_types


max_times = 10

def tokenize_komo(text: str) -> list:
    
    tokens = []
    feature_list = list(features.keys())

    for line in text.splitlines():
        if "komo.addObjective(" in line:

            if not ("accumulatedCollisions" in line or "jointLimits" in line):
                lists = re.findall(r'\[.*?\]', line)
                params_in_objective = re.split(r',\s*(?![^\[\]]*\])', line)

                times = int(lists[0].replace("[", "").replace("]", ""))
                assert times < 10
                tokens.append(times)

                current_offset = max_times

                feature = params_in_objective[1].replace(" ", "").replace("ry.FS.", "")
                feature_idx = feature_list.index(feature) + current_offset
                tokens.append(feature_idx)

                current_offset += len(feature_list)

                frames = ast.literal_eval(lists[1])
                for f in frames:
                    frame_idx = frame_names.index(f) + current_offset
                    tokens.append(frame_idx)

                current_offset += len(frame_names)

                type_ = params_in_objective[3].replace(" ", "").replace(")", "").replace("ry.OT.", "")
                type_idx = contraint_types.index(type_) + current_offset
                tokens.append(type_idx)

                current_offset += len(contraint_types)

                scale = ast.literal_eval(lists[2].replace(" ", "").replace(")", ""))
                scale_idx_neg = current_offset
                scale_idx_pos = scale_idx_neg + 1

                current_offset += 2

                for s in scale:
                    if s:
                        tokens.append(scale_idx_pos)
                    else:
                        tokens.append(scale_idx_neg)

                if len(lists) > 3:
                    target = ast.literal_eval(params_in_objective[-1].replace(" ", "").replace(")", ""))
                    for _ in target:
                        target_idx = current_offset
                        tokens.append(target_idx)

    return tokens


def komo_from_indices(indices: list[int]) -> str:
    
    text = ""
    feature_list = list(features.keys())
    prev_type = ""
    had_mode_switch = False
    
    for i in indices:
        
        if i < max_times:
            if prev_type == "target":
                text += "])\n"
            elif prev_type != "":
                text += ")\n"
            text += f"komo.addObjective([{i}], "
            prev_type = "times"
            continue

        i -= max_times
        if i < len(feature_list):
            text += f"ry.FS.{feature_list[i]}, "
            prev_type = "feature"
            continue

        i -= len(feature_list)
        if i < len(frame_names):
            if prev_type != "frame":
                text += "["
            else:
                text += ", "
            text += f"'{frame_names[i]}'"
            prev_type = "frame"
            continue

        i -= len(frame_names)
        if i < len(contraint_types):
            if prev_type != "frame":
                text += "["
            text += f"], ry.OT.{contraint_types[i]}, "
            prev_type = "type"
            continue

        i -= len(contraint_types)
        if i < 2:
            if prev_type != "scale":
                text += "["
            else:
                text += ", "
            if i:
                text += f"1e0"
            else:
                text += f"0"
            prev_type = "scale"
            continue
        
        if prev_type != "target":
            text += "], ["
        else:
            text += ", "
        text += f"0.0"
        prev_type = "target"

    if prev_type == "target":
        text += "]"
    text += ")\n"
    return text


if __name__ == "__main__":
    text = """komo = ry.KOMO(C, 4, 32, 2, False)
komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
komo.addObjective([1], ry.FS.scalarProductZZ, ['handle', 'box1'], ry.OT.eq, [1e2], [4.232])
komo.addObjective([1], ry.FS.insideBox, ['handle', 'l_gripper'], ry.OT.eq, [1e1], [2.488, -2.336, -3.359, -0.581, -4.351, -2.188])
komo.addObjective([1], ry.FS.angularVel, ['handle'], ry.OT.sos, [1e0], [1.251, 1.564, 5.736])
komo.addObjective([1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], [-1.545, -3.036, -0.439])
komo.addObjective([2], ry.FS.aboveBox, ['box2', 'box1'], ry.OT.eq, [1e1, 1e1, 0, 0], [-1.872, -4.047, -1.732, -4.917])
komo.addObjective([3], ry.FS.scalarProductYY, ['l_gripper', 'box2'], ry.OT.eq, [1e-1], [-3.765])
komo.addObjective([3], ry.FS.qItself, [], ry.OT.ineq, [0, 1e1, 0, 0, 1e1, 0, 1e1, 1e1, 0, 1e1], [1.286, 3.407, 5.601, -5.017, 2.019, -5.365, 1.953, 2.045, 1.306, 4.496])
komo.addObjective([4], ry.FS.scalarProductXZ, ['box2', 'handle'], ry.OT.ineq, [1e0], [-0.993])
komo.addObjective([4], ry.FS.quaternion, ['l_gripper'], ry.OT.ineq, [1e1], [3.345, 0.657, 2.76, -0.874])"""
    
    tokens = tokenize_komo(text)
    print(tokens)
    komo_reconstruct = komo_from_indices(tokens)
    print(komo_reconstruct)
